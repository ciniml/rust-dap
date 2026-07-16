// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! GDB debugger milestone M3: a standalone gdbserver on the probe RP2040.
//!
//! Presents a USB-CDC serial port that speaks the GDB Remote Serial Protocol
//! (via the `gdbstub` crate, no_std/no-alloc). GDB attaches directly to the
//! probe — no OpenOCD/probe-rs needed:
//!
//!   gdb-multiarch -ex 'target remote /dev/ttyACMx' \
//!                 -ex 'set architecture armv4t' -ex 'info registers'
//!
//! The gdbstub `Target` is backed by the `arm-debug` ADIv5 + Cortex-M stack
//! over a bit-banging SWD transport. Supports: read/write registers, read/write
//! memory, continue, single-step, Ctrl-C interrupt, and RAM software
//! breakpoints (BKPT patching). Pins: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET.

#![no_std]
#![no_main]

#[cfg(feature = "gdb-target-rp2040")]
use arm_debug::rp2040;
use arm_debug::{cortex_m as cm, ArmDebug, HaltReason, WatchAccess};
use core::convert::Infallible;
use rp_pico::hal;

use gdbstub::common::{Signal, Tid};
use gdbstub::conn::Connection;
use gdbstub::outputln;
use gdbstub::stub::state_machine::GdbStubStateMachine;
use gdbstub::stub::{GdbStubBuilder, MultiThreadStopReason};
use gdbstub::target::ext::base::multithread::{
    MultiThreadBase, MultiThreadResume, MultiThreadResumeOps, MultiThreadSchedulerLocking,
    MultiThreadSchedulerLockingOps, MultiThreadSingleStep, MultiThreadSingleStepOps,
};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::target::ext::breakpoints::{
    Breakpoints, BreakpointsOps, HwBreakpoint, HwBreakpointOps, HwWatchpoint, HwWatchpointOps,
    SwBreakpoint, SwBreakpointOps, WatchKind,
};
use gdbstub::target::ext::monitor_cmd::{ConsoleOutput, MonitorCmd, MonitorCmdOps};
use gdbstub::target::{Target, TargetError, TargetResult};
use gdbstub_arch::arm::reg::ArmCoreRegs;
use gdbstub_arch::arm::{ArmBreakpointKind, Armv4t};

use hal::clocks::Clock;
use hal::pac;
use rust_dap::{
    DapConfig, DapIdentity, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON,
};
use rust_dap_rp2040::bitbang::{CortexMDelay, PicoBidirPin, SwdIoSet};
use usb_device::prelude::*;
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

type Swd = SwdIoSet<hal::gpio::bank0::Gpio2, hal::gpio::bank0::Gpio3, hal::gpio::bank0::Gpio4>;

// --- gdbstub Target backed by arm-debug -----------------------------------

const BKPT: u16 = 0xBE00; // Thumb BKPT #0

/// How a GDB "software" (Z0) breakpoint is realized.
enum SwBpKind {
    /// RAM: original halfword saved, BKPT patched in.
    Ram { orig: u16 },
    /// ROM/flash (code region): silently promoted to an FPB comparator —
    /// BKPT patching is impossible there (XIP writes are ignored), which
    /// used to make plain `break` on flash a silent no-op.
    Fpb,
}

struct SwBp {
    addr: u32,
    kind: SwBpKind,
}

fn core_tid(core: usize) -> Tid {
    Tid::new(core + 1).unwrap()
}
fn tid_core(tid: Tid) -> usize {
    (tid.get() - 1).min(MAX_CORES - 1)
}

/// Requested vCont action for one core. `Default` means "no explicit
/// action": per the gdbstub contract that is *continue*, unless GDB engaged
/// scheduler locking (no wildcard action), in which case it is "stay halted".
#[derive(Clone, Copy, PartialEq, Eq)]
enum CoreAction {
    Default,
    Continue,
    Step,
}

struct RpTarget {
    arm: ArmDebug<Swd>,
    family: Family,
    breakpoints: heapless::Vec<SwBp, 8>,
    /// Pending vCont actions per core.
    actions: [CoreAction; MAX_CORES],
    /// Cores currently running (resumed and not yet re-halted).
    running: [bool; MAX_CORES],
    /// Core whose last resume was a single-step (stop reported as DoneStep).
    stepped: Option<usize>,
    /// GDB engaged scheduler locking: cores without an explicit action stay
    /// halted instead of continuing.
    sched_lock: bool,
    /// Internal diagnostics, readable from GDB as memory at 0xF000_0000
    /// (`x/8wx 0xF0000000`) even when the SWD link is dead:
    /// [0] connect_and_halt invocations
    /// [1] attempts used by the last connect_and_halt (0xdead = all failed)
    /// [2] last connect_multidrop result (0xa11600d = ok, else error code)
    /// [3] last halt result
    /// [4] last verify-read result
    /// [5] session op errors since last connect
    /// [6] last successful DPIDR
    /// [7] marker 0xd1a6d1a6
    /// [8] reset site of the previous boot (which reset_self call fired)
    /// [9] reset count (survives sys_reset via watchdog scratch)
    /// [10] sessions ended (clean disconnect or recovered stub error)
    diag: [u32; 11],
    /// RTT bridge state.
    rtt: RttState,
}

const DIAG_BASE: u32 = 0xF000_0000;
const DIAG_OK: u32 = 0x0a11_600d;

// --- Flash writing (M5): GDB `load`/memory writes into the XIP window are
// implemented by calling the target's bootrom flash routines.
#[cfg(feature = "gdb-target-rp2040")]
const RP2040_FLASH_BASE: u32 = rp2040::FLASH_BASE;
#[cfg(feature = "gdb-target-rp2040")]
const RP2040_FLASH_SIZE: u32 = 2 * 1024 * 1024; // Pico W25Q16 (2 MiB)
#[cfg(feature = "gdb-target-rp2040")]
const RP2040_FLASH_SECTOR: u32 = 4096;
/// Staging buffer in target RAM for flash_range_program's source data.
#[cfg(feature = "gdb-target-rp2040")]
const TARGET_STAGE: u32 = 0x2001_0000;
/// Stack for remote bootrom calls (grows down).
#[cfg(feature = "gdb-target-rp2040")]
const TARGET_CALL_SP: u32 = 0x2000_8000;
/// BKPT return trampoline for remote calls.
#[cfg(feature = "gdb-target-rp2040")]
const TARGET_TRAMPOLINE: u32 = 0x2000_7000;
/// DHCSR poll budgets: a 4 KiB sector erase takes tens of ms.
#[cfg(feature = "gdb-target-rp2040")]
const POLLS_CALL: u32 = 100_000;
const POLLS_ERASE: u32 = 400_000;

/// Bootrom flash routine addresses, looked up once per boot.
#[cfg(feature = "gdb-target-rp2040")]
#[derive(Clone, Copy)]
struct RomFlashFns {
    connect: u32,
    exit_xip: u32,
    erase: u32,
    program: u32,
    flush: u32,
    enter_xip: u32,
}

// --- SEGGER RTT (M-RTT1): control-block discovery + halt-time dump --------
// Layout: 16-byte ID "SEGGER RTT" (zero-padded), MaxNumUpBuffers (u32),
// MaxNumDownBuffers (u32), then descriptors of 24 bytes each — all up
// buffers, then all down buffers: { pName, pBuffer, SizeOfBuffer, WrOff,
// RdOff, Flags }. Up buffers: target writes WrOff, we consume via RdOff.

/// Max control blocks reported by an RTT scan.
const RTT_MAX_FOUND: usize = 4;

/// The 16-byte RTT control-block ID, assembled at runtime so the probe's own
/// binary never contains the literal (which a RAM scan could false-hit on).
fn rtt_id() -> [u8; 16] {
    let mut id = [0u8; 16];
    for (i, b) in b"TTR REGGES".iter().rev().enumerate() {
        id[i] = *b;
    }
    id
}

/// RTT bridge state (selected control block + channel).
struct RttState {
    cb: Option<u32>,
    channel: u32,
    /// Cached descriptor addresses for the selected channel (recomputed on
    /// attach / channel change so the streaming poll avoids header reads).
    up_desc: Option<u32>,
    down_desc: Option<u32>,
}

fn err_code(e: &arm_debug::ArmError) -> u32 {
    use arm_debug::ArmError::*;
    match e {
        Transfer(ack) => 0x100 | *ack as u32,
        Wait => 1,
        NotSupported => 2,
        PowerUpTimeout => 3,
        CoreTimeout => 4,
        NoTarget => 5,
        Internal => 6,
    }
}

// --- Target-family abstraction --------------------------------------------
// One family is selected at compile time (gdb-target-rp2040 / -nrf52). The
// architecture-common core debug (halt/step/registers/breakpoints/reset)
// lives in RpTarget; the family provides only what differs: the SWD connect,
// core selection, flash programming, and the memory map.

/// Upper bound on cores across families (RP2040 has 2); active count is
/// `Family::NUM_CORES`.
const MAX_CORES: usize = 2;

trait TargetFamily {
    const NUM_CORES: usize;
    /// Flash window base/size (writes here are routed to `flash_write`).
    const FLASH_BASE: u32;
    const FLASH_SIZE: u32;
    /// Target RAM range (RTT control-block scan).
    const RAM_START: u32;
    const RAM_END: u32;

    // Runtime accessors for the const memory-map/topology (needed once the
    // family is chosen at runtime by auto-detection).
    fn num_cores(&self) -> usize {
        Self::NUM_CORES
    }
    fn flash_base(&self) -> u32 {
        Self::FLASH_BASE
    }
    fn flash_size(&self) -> u32 {
        Self::FLASH_SIZE
    }
    fn ram_start(&self) -> u32 {
        Self::RAM_START
    }
    fn ram_end(&self) -> u32 {
        Self::RAM_END
    }

    fn new() -> Self;
    /// Raw SWD connect to core 0; returns DPIDR. Retry/halt/diagnostics are
    /// handled by RpTarget. (Only the non-auto path calls this directly.)
    #[cfg_attr(feature = "gdb-target-auto", allow(dead_code))]
    fn connect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError>;
    /// Point the link at `core`'s DP (cheap no-op when already there or when
    /// single-core). Owns the current-core cache.
    fn select_core(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        core: usize,
    ) -> Result<(), arm_debug::ArmError>;
    /// Forget the cached current core (call before a fresh connect).
    fn forget_core(&mut self);
    /// Erase + program flash for the given XIP/flash-window address.
    fn flash_write(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError>;
    /// Leave any flash-command mode so the flash window reads/executes again.
    fn flash_finish(&mut self, arm: &mut ArmDebug<Swd>);
    /// Reset per-session flash bookkeeping on reconnect.
    fn reset_flash_state(&mut self);
    /// Whether flash is currently in command mode (skip RTT polling then).
    fn in_flash_mode(&self) -> bool;
}

/// The compiled-in target families. With `gdb-target-auto`, connect probes
/// the wire and picks the matching variant at runtime; otherwise the single
/// enabled family is used. Either way, which families exist is feature-gated.
enum Family {
    #[cfg(feature = "gdb-target-rp2040")]
    Rp2040(Rp2040Family),
    #[cfg(feature = "gdb-target-nrf52")]
    Nrf52(Nrf52Family),
    #[cfg(feature = "gdb-target-nrf54")]
    Nrf54(Nrf54Family),
}

/// Run `$body` (which references the bound inner family `$f`) on whichever
/// variant is active. cfg-gated arms vanish with their feature, so a
/// single-family build compiles to a one-arm (exhaustive) match.
macro_rules! on_family {
    ($self:expr, $f:ident => $body:expr) => {
        match $self {
            #[cfg(feature = "gdb-target-rp2040")]
            Family::Rp2040($f) => $body,
            #[cfg(feature = "gdb-target-nrf52")]
            Family::Nrf52($f) => $body,
            #[cfg(feature = "gdb-target-nrf54")]
            Family::Nrf54($f) => $body,
        }
    };
}

impl Family {
    /// Family used before the first connect / when auto-detection is off:
    /// the first compiled-in variant (RP2040 > nRF52 > nRF54).
    fn default_family() -> Self {
        #[cfg(feature = "gdb-target-rp2040")]
        {
            Family::Rp2040(Rp2040Family::new())
        }
        #[cfg(all(not(feature = "gdb-target-rp2040"), feature = "gdb-target-nrf52"))]
        {
            Family::Nrf52(Nrf52Family::new())
        }
        #[cfg(all(
            not(feature = "gdb-target-rp2040"),
            not(feature = "gdb-target-nrf52"),
            feature = "gdb-target-nrf54"
        ))]
        {
            Family::Nrf54(Nrf54Family::new())
        }
    }

    /// Whether the active family is nRF52 (gates nRF52-only monitor commands).
    #[cfg(feature = "gdb-target-nrf52")]
    fn is_nrf52(&self) -> bool {
        matches!(self, Family::Nrf52(_))
    }

    /// Whether the active family is nRF54 (gates nRF54-only monitor commands).
    #[cfg(feature = "gdb-target-nrf54")]
    fn is_nrf54(&self) -> bool {
        matches!(self, Family::Nrf54(_))
    }

    /// Connect, auto-detecting the target when `gdb-target-auto` is set. On
    /// success `*self` becomes the detected family. Detection order: the nRF
    /// parts over plain SW-DP (matched by exact DPIDR), then RP2040 over
    /// multidrop.
    fn connect_detect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError> {
        #[cfg(all(
            feature = "gdb-target-auto",
            any(feature = "gdb-target-nrf52", feature = "gdb-target-nrf54")
        ))]
        {
            // One plain SW-DP bring-up serves both nRF families; dispatch on
            // the returned DPIDR.
            if let Ok(id) = arm.connect_swd() {
                #[cfg(feature = "gdb-target-nrf52")]
                if id == arm_debug::nrf52::DPIDR {
                    *self = Family::Nrf52(Nrf52Family::new());
                    return Ok(id);
                }
                #[cfg(feature = "gdb-target-nrf54")]
                if id == arm_debug::nrf54::DPIDR {
                    *self = Family::Nrf54(Nrf54Family::new());
                    return Ok(id);
                }
            }
        }
        #[cfg(all(feature = "gdb-target-auto", feature = "gdb-target-rp2040"))]
        {
            // Multidrop connect only succeeds (valid DPIDR) on an RP2040.
            if let Ok(id) = arm.connect_multidrop(rp2040::CORE0_TARGETSEL) {
                let mut fam = Rp2040Family::new();
                fam.cur_core = Some(0);
                *self = Family::Rp2040(fam);
                return Ok(id);
            }
        }
        #[cfg(feature = "gdb-target-auto")]
        {
            Err(arm_debug::ArmError::NoTarget)
        }
        // Single-family build: just connect the fixed family.
        #[cfg(not(feature = "gdb-target-auto"))]
        {
            self.connect(arm)
        }
    }

    fn num_cores(&self) -> usize {
        on_family!(self, f => f.num_cores())
    }
    fn flash_base(&self) -> u32 {
        on_family!(self, f => f.flash_base())
    }
    fn flash_size(&self) -> u32 {
        on_family!(self, f => f.flash_size())
    }
    fn ram_start(&self) -> u32 {
        on_family!(self, f => f.ram_start())
    }
    fn ram_end(&self) -> u32 {
        on_family!(self, f => f.ram_end())
    }
    #[cfg_attr(feature = "gdb-target-auto", allow(dead_code))]
    fn connect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError> {
        on_family!(self, f => f.connect(arm))
    }
    fn select_core(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        on_family!(self, f => f.select_core(arm, core))
    }
    fn forget_core(&mut self) {
        on_family!(self, f => f.forget_core())
    }
    fn flash_write(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        on_family!(self, f => f.flash_write(arm, addr, data))
    }
    fn flash_finish(&mut self, arm: &mut ArmDebug<Swd>) {
        on_family!(self, f => f.flash_finish(arm))
    }
    fn reset_flash_state(&mut self) {
        on_family!(self, f => f.reset_flash_state())
    }
    fn in_flash_mode(&self) -> bool {
        on_family!(self, f => f.in_flash_mode())
    }
}

// --- RP2040 family --------------------------------------------------------

/// SWD multidrop TARGETSEL per core.
#[cfg(feature = "gdb-target-rp2040")]
const CORE_TARGETSEL: [u32; 2] = [rp2040::CORE0_TARGETSEL, rp2040::CORE1_TARGETSEL];

#[cfg(feature = "gdb-target-rp2040")]
struct Rp2040Family {
    cur_core: Option<usize>,
    flash_fns: Option<RomFlashFns>,
    flash_mode: bool,
    erased: [u32; (RP2040_FLASH_SIZE / RP2040_FLASH_SECTOR / 32) as usize],
}

#[cfg(feature = "gdb-target-rp2040")]
impl Rp2040Family {
    fn rom_flash_fns(
        &mut self,
        arm: &mut ArmDebug<Swd>,
    ) -> Result<RomFlashFns, arm_debug::ArmError> {
        if let Some(f) = self.flash_fns {
            return Ok(f);
        }
        let mut get = |code| -> Result<u32, arm_debug::ArmError> {
            arm.rom_func_lookup(code)?
                .ok_or(arm_debug::ArmError::Internal)
        };
        let fns = RomFlashFns {
            connect: get(rp2040::FN_CONNECT_INTERNAL_FLASH)?,
            exit_xip: get(rp2040::FN_FLASH_EXIT_XIP)?,
            erase: get(rp2040::FN_FLASH_RANGE_ERASE)?,
            program: get(rp2040::FN_FLASH_RANGE_PROGRAM)?,
            flush: get(rp2040::FN_FLASH_FLUSH_CACHE)?,
            enter_xip: get(rp2040::FN_FLASH_ENTER_CMD_XIP)?,
        };
        self.flash_fns = Some(fns);
        Ok(fns)
    }

    fn rom_call(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        fn_addr: u32,
        args: [u32; 4],
        polls: u32,
    ) -> Result<u32, arm_debug::ArmError> {
        self.select_core(arm, 0)?; // bootrom calls always run on core 0
        arm.call_function(fn_addr, &args, TARGET_CALL_SP, TARGET_TRAMPOLINE, polls)
    }

    fn flash_enter(&mut self, arm: &mut ArmDebug<Swd>) -> Result<(), arm_debug::ArmError> {
        if self.flash_mode {
            return Ok(());
        }
        self.select_core(arm, 0)?;
        let f = self.rom_flash_fns(arm)?;
        self.rom_call(arm, f.connect, [0; 4], POLLS_CALL)?;
        self.rom_call(arm, f.exit_xip, [0; 4], POLLS_CALL)?;
        self.erased = Default::default();
        self.flash_mode = true;
        Ok(())
    }
}

#[cfg(feature = "gdb-target-rp2040")]
impl TargetFamily for Rp2040Family {
    const NUM_CORES: usize = 2;
    const FLASH_BASE: u32 = RP2040_FLASH_BASE;
    const FLASH_SIZE: u32 = RP2040_FLASH_SIZE;
    const RAM_START: u32 = 0x2000_0000;
    const RAM_END: u32 = 0x2004_2000;

    fn new() -> Self {
        Self {
            cur_core: None,
            flash_fns: None,
            flash_mode: false,
            erased: Default::default(),
        }
    }

    fn connect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError> {
        let dpidr = arm.connect_multidrop(rp2040::CORE0_TARGETSEL)?;
        self.cur_core = Some(0);
        Ok(dpidr)
    }

    fn select_core(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        if self.cur_core == Some(core) {
            return Ok(());
        }
        self.cur_core = None;
        arm.reselect(CORE_TARGETSEL[core])?;
        self.cur_core = Some(core);
        Ok(())
    }

    fn forget_core(&mut self) {
        self.cur_core = None;
    }

    fn flash_write(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        let off = addr - RP2040_FLASH_BASE;
        let end = off + data.len() as u32;
        if end > RP2040_FLASH_SIZE {
            return Err(arm_debug::ArmError::Internal);
        }
        self.flash_enter(arm)?;
        let f = self.rom_flash_fns(arm)?;
        for sector in off / RP2040_FLASH_SECTOR..=(end - 1) / RP2040_FLASH_SECTOR {
            let (word, bit) = ((sector / 32) as usize, sector % 32);
            if self.erased[word] & (1 << bit) == 0 {
                self.rom_call(
                    arm,
                    f.erase,
                    [
                        sector * RP2040_FLASH_SECTOR,
                        RP2040_FLASH_SECTOR,
                        1 << 16,
                        0xD8,
                    ],
                    POLLS_ERASE,
                )?;
                self.erased[word] |= 1 << bit;
            }
        }
        let mut buf = [0xFFu8; 1024];
        let mut pos = off & !0xFF;
        let span_end = (end + 0xFF) & !0xFF;
        while pos < span_end {
            let n = buf.len().min((span_end - pos) as usize);
            buf[..n].fill(0xFF);
            for (i, slot) in buf[..n].iter_mut().enumerate() {
                let a = pos + i as u32;
                if a >= off && a < end {
                    *slot = data[(a - off) as usize];
                }
            }
            arm.write_mem(TARGET_STAGE, &buf[..n])?;
            self.rom_call(
                arm,
                f.program,
                [pos, TARGET_STAGE, n as u32, 0],
                POLLS_ERASE,
            )?;
            pos += n as u32;
        }
        Ok(())
    }

    fn flash_finish(&mut self, arm: &mut ArmDebug<Swd>) {
        if !self.flash_mode {
            return;
        }
        if self.select_core(arm, 0).is_err() {
            self.flash_mode = false;
            return;
        }
        if let Ok(f) = self.rom_flash_fns(arm) {
            let _ = self.rom_call(arm, f.flush, [0; 4], POLLS_CALL);
            let _ = self.rom_call(arm, f.enter_xip, [0; 4], POLLS_CALL);
        }
        self.flash_mode = false;
    }

    fn reset_flash_state(&mut self) {
        self.flash_mode = false;
    }

    fn in_flash_mode(&self) -> bool {
        self.flash_mode
    }
}

// --- nRF52 family ---------------------------------------------------------

#[cfg(feature = "gdb-target-nrf52")]
struct Nrf52Family {
    /// Pages erased this session (1 bit per 4 KiB page; 512 KiB max).
    erased: [u32; 4],
}

#[cfg(feature = "gdb-target-nrf52")]
impl TargetFamily for Nrf52Family {
    const NUM_CORES: usize = 1;
    const FLASH_BASE: u32 = arm_debug::nrf52::FLASH_BASE;
    const FLASH_SIZE: u32 = 512 * 1024; // nRF52832: 512 KiB
    const RAM_START: u32 = 0x2000_0000;
    const RAM_END: u32 = 0x2001_0000; // nRF52832: 64 KiB

    fn new() -> Self {
        Self { erased: [0; 4] }
    }

    fn connect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError> {
        arm.connect_swd()
    }

    fn select_core(
        &mut self,
        _arm: &mut ArmDebug<Swd>,
        _core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        Ok(()) // single core
    }

    fn forget_core(&mut self) {}

    fn flash_write(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        use arm_debug::nrf52;
        let end = addr + data.len() as u32;
        if end > Self::FLASH_SIZE {
            return Err(arm_debug::ArmError::Internal);
        }
        // Erase each not-yet-erased page the write touches.
        let first = addr / nrf52::FLASH_PAGE;
        let last = (end - 1) / nrf52::FLASH_PAGE;
        for page in first..=last {
            let (w, b) = ((page / 32) as usize, page % 32);
            if self.erased[w] & (1 << b) == 0 {
                arm.nrf52_erase_page(page * nrf52::FLASH_PAGE, POLLS_ERASE)?;
                self.erased[w] |= 1 << b;
            }
        }
        // Program the word-aligned span, padding edges with 0xFF (NVMC only
        // clears bits, and an erased page reads 0xFF, so padding is a no-op).
        // Build each ≤256-word batch and program it at its own base address.
        let span = addr & !3;
        let span_end = (end + 3) & !3;
        let mut base = span;
        while base < span_end {
            let mut words: heapless::Vec<u32, 256> = heapless::Vec::new();
            let mut a = base;
            while a < span_end && !words.is_full() {
                let mut bytes = [0xFFu8; 4];
                for (i, slot) in bytes.iter_mut().enumerate() {
                    let ba = a + i as u32;
                    if ba >= addr && ba < end {
                        *slot = data[(ba - addr) as usize];
                    }
                }
                let _ = words.push(u32::from_le_bytes(bytes));
                a += 4;
            }
            arm.nrf52_program(base, &words, POLLS_ERASE)?;
            base = a;
        }
        Ok(())
    }

    fn flash_finish(&mut self, _arm: &mut ArmDebug<Swd>) {}

    fn reset_flash_state(&mut self) {
        self.erased = [0; 4];
    }

    fn in_flash_mode(&self) -> bool {
        false
    }
}

// --- nRF54L family (Cortex-M33 / RRAM) ------------------------------------

#[cfg(feature = "gdb-target-nrf54")]
struct Nrf54Family;

#[cfg(feature = "gdb-target-nrf54")]
impl TargetFamily for Nrf54Family {
    const NUM_CORES: usize = 1; // the M33 application core
    const FLASH_BASE: u32 = arm_debug::nrf54::RRAM_BASE;
    const FLASH_SIZE: u32 = arm_debug::nrf54::RRAM_SIZE;
    const RAM_START: u32 = 0x2000_0000;
    const RAM_END: u32 = 0x2003_C000; // nRF54L15: 240 KiB SRAM

    fn new() -> Self {
        Self
    }

    fn connect(&mut self, arm: &mut ArmDebug<Swd>) -> Result<u32, arm_debug::ArmError> {
        // nRF54L is DPv2 single-drop; the plain SW-DP bring-up works.
        arm.connect_swd()
    }

    fn select_core(
        &mut self,
        _arm: &mut ArmDebug<Swd>,
        _core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        Ok(())
    }

    fn forget_core(&mut self) {}

    fn flash_write(
        &mut self,
        arm: &mut ArmDebug<Swd>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        let end = addr + data.len() as u32;
        if end > Self::FLASH_SIZE {
            return Err(arm_debug::ArmError::Internal);
        }
        // RRAM is resistive: no erase, direct overwrite. Program word-aligned
        // batches; edges read the existing word so partial writes preserve
        // neighbours (RRAM can set bits either way, unlike NOR flash).
        let span = addr & !3;
        let span_end = (end + 3) & !3;
        let mut base = span;
        while base < span_end {
            let mut words: heapless::Vec<u32, 256> = heapless::Vec::new();
            let mut a = base;
            while a < span_end && !words.is_full() {
                let word = if a >= addr && a + 4 <= end {
                    u32::from_le_bytes([
                        data[(a - addr) as usize],
                        data[(a - addr + 1) as usize],
                        data[(a - addr + 2) as usize],
                        data[(a - addr + 3) as usize],
                    ])
                } else {
                    // Ragged edge: read-modify-write to keep the other bytes.
                    let mut bytes = arm.read_word(a)?.to_le_bytes();
                    for (i, b) in bytes.iter_mut().enumerate() {
                        let ba = a + i as u32;
                        if ba >= addr && ba < end {
                            *b = data[(ba - addr) as usize];
                        }
                    }
                    u32::from_le_bytes(bytes)
                };
                let _ = words.push(word);
                a += 4;
            }
            arm.nrf54_program(base, &words, POLLS_ERASE)?;
            base = a;
        }
        Ok(())
    }

    fn flash_finish(&mut self, _arm: &mut ArmDebug<Swd>) {}
    fn reset_flash_state(&mut self) {}
    fn in_flash_mode(&self) -> bool {
        false
    }
}

impl RpTarget {
    fn map_err<E>(_e: E) -> TargetError<Infallible> {
        // arm-debug errors surface to GDB as a generic non-fatal error.
        TargetError::NonFatal
    }

    /// Count a failed session operation (diag[5]).
    fn diag_err<T, E>(&mut self, r: Result<T, E>) -> Result<T, E> {
        if r.is_err() {
            self.diag[5] = self.diag[5].wrapping_add(1);
        }
        r
    }

    /// Point the SWD link at `core`'s DP (delegated to the family).
    fn select_core(&mut self, core: usize) -> Result<(), arm_debug::ArmError> {
        self.family.select_core(&mut self.arm, core)
    }

    /// Establish the SWD link and halt both cores, retrying from scratch
    /// until a DHCSR read confirms the link is actually usable. A connect
    /// issued on a live link has been observed to fail on hardware; the
    /// failed attempt leaves the target in a state from which a later
    /// attempt succeeds. Records what happened into `diag`.
    fn connect_and_halt(&mut self) {
        self.diag[0] = self.diag[0].wrapping_add(1);
        self.diag[5] = 0;
        self.family.reset_flash_state();
        self.running = [false; MAX_CORES];
        for attempt in 1u32..=4 {
            self.family.forget_core();
            self.diag[2] = match self.family.connect_detect(&mut self.arm) {
                Ok(dpidr) => {
                    self.diag[6] = dpidr;
                    DIAG_OK
                }
                Err(e) => err_code(&e),
            };
            self.diag[3] = match self.arm.halt() {
                Ok(()) => DIAG_OK,
                Err(e) => err_code(&e),
            };
            self.diag[4] = match self.arm.read_word(cm::DHCSR) {
                Ok(_) => DIAG_OK,
                Err(e) => err_code(&e),
            };
            if self.diag[2] == DIAG_OK && self.diag[3] == DIAG_OK && self.diag[4] == DIAG_OK {
                self.diag[1] = attempt;
                // Comparators survive in target hardware across sessions;
                // start each session without leftover HW break/watchpoints.
                let _ = self.arm.debug_units_clear();
                // Bring the other cores to the same state, then return to 0.
                for core in 1..self.family.num_cores() {
                    if self.select_core(core).is_ok() {
                        let _ = self.arm.halt();
                        let _ = self.arm.debug_units_clear();
                    }
                }
                let _ = self.select_core(0);
                return;
            }
        }
        self.diag[1] = 0xdead;
    }

    /// System reset via AIRCR.SYSRESETREQ (resets both cores + peripherals),
    /// then re-establish the SWD link with everything halted. With `halt`,
    /// core 0 is caught at the reset vector via DEMCR.VC_CORERESET; without
    /// it, the target runs briefly until the reconnect halts it. Returns
    /// core 0's pc after the reset.
    fn target_reset(&mut self, halt: bool) -> Result<u32, arm_debug::ArmError> {
        self.flash_finish();
        self.select_core(0)?;
        let demcr = self.arm.read_word(cm::DEMCR)?;
        let demcr = (demcr & !cm::DEMCR_VC_CORERESET) | cm::DEMCR_DWTENA;
        self.arm.write_word(
            cm::DEMCR,
            demcr | if halt { cm::DEMCR_VC_CORERESET } else { 0 },
        )?;
        // The chip drops mid-transaction; the response never arrives.
        let _ = self.arm.write_word(cm::AIRCR, cm::AIRCR_SYSRESETREQ);
        // Give the bootrom time to come up before reconnecting.
        cortex_m::asm::delay(2_000_000); // ~16 ms @ 125 MHz
        self.connect_and_halt();
        if self.diag[1] == 0xdead {
            return Err(arm_debug::ArmError::NoTarget);
        }
        if halt {
            // Vector catch served its purpose; don't halt future resets.
            let _ = self.arm.write_word(cm::DEMCR, cm::DEMCR_DWTENA);
        }
        self.select_core(0)?;
        self.arm.read_core_reg(cm::PC)
    }

    /// Leave flash-command mode via the family (best-effort, core 0).
    fn flash_finish(&mut self) {
        if self.family.in_flash_mode() {
            let _ = self.select_core(0);
        }
        self.family.flash_finish(&mut self.arm);
    }

    /// Classify why `core` stopped, for GDB's stop reply. DFSR (cleared by
    /// the read) distinguishes watchpoints from breakpoints; an FPB
    /// comparator covering the PC distinguishes hardware from software
    /// breakpoints. Assumes the link already addresses `core`.
    fn stop_reason(&mut self, core: usize) -> MultiThreadStopReason<u32> {
        let tid = core_tid(core);
        if self.stepped == Some(core) {
            return MultiThreadStopReason::DoneStep;
        }
        match self.arm.halt_reason() {
            Ok(HaltReason::Watchpoint(addr, access)) => MultiThreadStopReason::Watch {
                tid,
                kind: match access {
                    WatchAccess::Read => WatchKind::Read,
                    WatchAccess::Write => WatchKind::Write,
                    WatchAccess::ReadWrite => WatchKind::ReadWrite,
                },
                addr,
            },
            Ok(HaltReason::Breakpoint) => {
                let pc = self.arm.read_core_reg(cm::PC).unwrap_or(0);
                // A promoted Z0 breakpoint lives in the FPB but must still be
                // reported as a software breakpoint for GDB to match it.
                if !self.promoted_sw_at(pc) && self.arm.hw_breakpoint_at(pc).unwrap_or(false) {
                    MultiThreadStopReason::HwBreak(tid)
                } else {
                    MultiThreadStopReason::SwBreak(tid)
                }
            }
            _ => MultiThreadStopReason::SignalWithThread {
                tid,
                signal: Signal::SIGTRAP,
            },
        }
    }

    /// Halt every core still marked running (all-stop semantics).
    fn halt_running(&mut self, except: Option<usize>) {
        for core in 0..self.family.num_cores() {
            if Some(core) != except && self.running[core] && self.select_core(core).is_ok() {
                let _ = self.arm.halt();
            }
        }
        self.running = [false; 2];
    }

    /// While running, check whether any resumed core has halted. If one has,
    /// halt the others (all-stop) and return the stop reason to report.
    fn poll_stopped(&mut self) -> Option<MultiThreadStopReason<u32>> {
        // A step completed synchronously inside resume(): report it as soon
        // as the state machine asks (the stepped core is already halted).
        // DoneStep carries no thread id — gdbstub would attribute it to its
        // idea of the current thread — so report an explicit SIGTRAP on the
        // stepped core instead.
        if let Some(core) = self.stepped.take() {
            self.halt_running(None);
            let _ = self.select_core(core);
            return Some(MultiThreadStopReason::SignalWithThread {
                tid: core_tid(core),
                signal: Signal::SIGTRAP,
            });
        }
        let mut stopped = None;
        for core in 0..self.family.num_cores() {
            if !self.running[core] {
                continue;
            }
            if self.select_core(core).is_err() {
                continue;
            }
            if self.arm.is_halted().unwrap_or(false) {
                stopped = Some(core);
                break;
            }
        }
        let core = stopped?;
        self.halt_running(Some(core));
        let _ = self.select_core(core);
        Some(self.stop_reason(core))
    }
}

impl Target for RpTarget {
    type Arch = Armv4t;
    type Error = Infallible;

    fn base_ops(&mut self) -> BaseOps<'_, Self::Arch, Self::Error> {
        BaseOps::MultiThread(self)
    }

    fn support_breakpoints(&mut self) -> Option<BreakpointsOps<'_, Self>> {
        Some(self)
    }

    fn support_monitor_cmd(&mut self) -> Option<MonitorCmdOps<'_, Self>> {
        Some(self)
    }
}

/// Parse a hex address argument ("20001234" or "0x20001234").
fn parse_hex(s: &[u8]) -> Option<u32> {
    let s = s.strip_prefix(b"0x").unwrap_or(s);
    if s.is_empty() || s.len() > 8 {
        return None;
    }
    let mut v = 0u32;
    for &b in s {
        v = (v << 4) | (b as char).to_digit(16)?;
    }
    Some(v)
}

impl RpTarget {
    /// Read one RTT buffer descriptor: (pName, pBuffer, Size, WrOff, RdOff).
    fn rtt_desc(&mut self, desc: u32) -> Result<(u32, u32, u32, u32, u32), arm_debug::ArmError> {
        let mut raw = [0u8; 20];
        self.arm.read_mem(desc, &mut raw)?;
        let w = |i: usize| u32::from_le_bytes(raw[i * 4..i * 4 + 4].try_into().unwrap());
        Ok((w(0), w(1), w(2), w(3), w(4)))
    }

    /// Address of the selected channel's descriptor, if a CB is attached.
    /// `up` selects the target→host (true) or host→target (false) array.
    fn rtt_channel_desc(&mut self, up: bool) -> Result<Option<u32>, arm_debug::ArmError> {
        let Some(cb) = self.rtt.cb else {
            return Ok(None);
        };
        let mut counts = [0u8; 8];
        self.arm.read_mem(cb + 16, &mut counts)?;
        let max_up = u32::from_le_bytes(counts[0..4].try_into().unwrap());
        let max_down = u32::from_le_bytes(counts[4..8].try_into().unwrap());
        let (base, count) = if up {
            (cb + 24, max_up)
        } else {
            (cb + 24 + 24 * max_up, max_down)
        };
        if self.rtt.channel >= count {
            return Ok(None);
        }
        Ok(Some(base + 24 * self.rtt.channel))
    }

    /// Scan target RAM for RTT control blocks; report each via `out`.
    fn rtt_scan(&mut self, out: &mut ConsoleOutput<'_>) {
        let id = rtt_id();
        let mut found = 0usize;
        let mut buf = [0u8; 1024 + 15];
        let ram_start = self.family.ram_start();
        let ram_end = self.family.ram_end();
        let mut addr = ram_start;
        while addr < ram_end && found < RTT_MAX_FOUND {
            let n = 1024.min((ram_end - addr) as usize) + 15;
            let n = n.min(buf.len()).min((ram_end - addr) as usize);
            if self.arm.read_mem(addr, &mut buf[..n]).is_err() {
                outputln!(out, "rtt: RAM read failed at 0x{:08x}", addr);
                return;
            }
            let mut off = 0;
            while off + 16 <= n {
                if buf[off..off + 16] == id {
                    let cb = addr + off as u32;
                    self.rtt_report_cb(cb, out);
                    found += 1;
                }
                off += 4;
            }
            addr += 1024;
        }
        if found == 0 {
            outputln!(out, "rtt: no control block found in RAM");
        }
    }

    /// Print one control block's summary (address, channels, validity).
    fn rtt_report_cb(&mut self, cb: u32, out: &mut ConsoleOutput<'_>) {
        let mut counts = [0u8; 8];
        if self.arm.read_mem(cb + 16, &mut counts).is_err() {
            return;
        }
        let max_up = u32::from_le_bytes(counts[0..4].try_into().unwrap());
        let max_down = u32::from_le_bytes(counts[4..8].try_into().unwrap());
        if max_up == 0 || max_up > 16 || max_down > 16 {
            outputln!(out, "cb at 0x{:08x}: implausible header, skipped", cb);
            return;
        }
        // Validity heuristic from up[0]: offsets inside the buffer, buffer in RAM.
        let valid = match self.rtt_desc(cb + 24) {
            Ok((_, pbuf, size, wr, rd)) => {
                size > 0
                    && size <= 0x10000
                    && (self.family.ram_start()..self.family.ram_end()).contains(&pbuf)
                    && wr < size
                    && rd < size
            }
            Err(_) => false,
        };
        outputln!(
            out,
            "cb at 0x{:08x}: up={} down={} [{}]",
            cb,
            max_up,
            max_down,
            if valid { "valid" } else { "stale?" }
        );
    }

    /// Drain the selected up buffer (bounded) and print it as text.
    fn rtt_dump(&mut self, out: &mut ConsoleOutput<'_>) {
        let desc = match self.rtt_channel_desc(true) {
            Ok(Some(d)) => d,
            Ok(None) => {
                outputln!(
                    out,
                    "rtt: no control block attached (use monitor rtt scan/attach)"
                );
                return;
            }
            Err(e) => {
                outputln!(out, "rtt: read failed: 0x{:x}", err_code(&e));
                return;
            }
        };
        let Ok((_, pbuf, size, wr, mut rd)) = self.rtt_desc(desc) else {
            outputln!(out, "rtt: descriptor read failed");
            return;
        };
        if size == 0 || wr >= size || rd >= size {
            outputln!(
                out,
                "rtt: descriptor looks corrupt (size={} wr={} rd={})",
                size,
                wr,
                rd
            );
            return;
        }
        if wr == rd {
            outputln!(out, "rtt: no new data");
            return;
        }
        let mut total = 0usize;
        while rd != wr && total < 1024 {
            let run = if wr > rd { wr - rd } else { size - rd };
            let mut chunk = [0u8; 64];
            let n = (run as usize).min(chunk.len()).min(1024 - total);
            if self.arm.read_mem(pbuf + rd, &mut chunk[..n]).is_err() {
                break;
            }
            // Print lossily as ASCII (RTT terminal data is normally text).
            let mut line = heapless::String::<192>::new();
            for &b in &chunk[..n] {
                let c = if (0x20..0x7f).contains(&b) || b == b'\n' || b == b'\t' {
                    b as char
                } else {
                    '.'
                };
                let _ = line.push(c);
            }
            gdbstub::output!(*out, "{}", line);
            rd = (rd + n as u32) % size;
            total += n;
        }
        // Consume: write RdOff back (descriptor offset +16).
        let _ = self.arm.write_word(desc + 16, rd);
        outputln!(out, "");
        outputln!(out, "rtt: {} bytes", total);
    }
}

impl RpTarget {
    /// Recompute the cached up/down descriptor addresses for the selected
    /// control block + channel.
    fn rtt_cache_descs(&mut self) {
        self.rtt.up_desc = self.rtt_channel_desc(true).ok().flatten();
        self.rtt.down_desc = self.rtt_channel_desc(false).ok().flatten();
    }

    /// One bounded streaming step: move up-buffer bytes into `tx` and `rx`
    /// bytes into the down buffer. Called from the session loop while the
    /// target runs (RAM is readable via the MEM-AP regardless of core state).
    fn rtt_stream(
        &mut self,
        tx: &mut heapless::spsc::Producer<'static, u8, RTT_TX_QUEUE_SIZE>,
        rx: &mut heapless::spsc::Consumer<'static, u8, RTT_RX_QUEUE_SIZE>,
    ) -> bool {
        if self.family.in_flash_mode() {
            return false; // don't interleave with flash programming
        }
        let mut moved = false;
        // Up: target -> host CDC. Read as large a contiguous run as fits the
        // TX queue in one bulk MEM-AP transfer (256 B), so the per-poll SWD
        // round-trips — descriptor read + RdOff write-back — are amortised
        // over many more bytes than a 64 B chunk, lifting throughput.
        if let Some(desc) = self.rtt.up_desc {
            let room = tx.capacity() - tx.len();
            if room >= 64 {
                if let Ok((_, pbuf, size, wr, rd)) = self.rtt_desc(desc) {
                    if size > 0 && wr < size && rd < size && wr != rd {
                        let run = if wr > rd { wr - rd } else { size - rd };
                        let mut chunk = [0u8; 256];
                        let n = (run as usize).min(chunk.len()).min(room);
                        if self.arm.read_mem(pbuf + rd, &mut chunk[..n]).is_ok() {
                            for &b in &chunk[..n] {
                                let _ = tx.enqueue(b);
                            }
                            let _ = self.arm.write_word(desc + 16, (rd + n as u32) % size);
                            moved = true;
                        }
                    }
                }
            }
        }
        // Down: host CDC -> target.
        if let Some(desc) = self.rtt.down_desc {
            if rx.ready() {
                if let Ok((_, pbuf, size, wr, rd)) = self.rtt_desc(desc) {
                    if size > 0 && wr < size && rd < size {
                        let free = (rd + size - wr - 1) % size;
                        let run = free.min(size - wr).min(64);
                        let mut chunk = [0u8; 64];
                        let mut n = 0usize;
                        while (n as u32) < run {
                            match rx.dequeue() {
                                Some(b) => {
                                    chunk[n] = b;
                                    n += 1;
                                }
                                None => break,
                            }
                        }
                        if n > 0 && self.arm.write_mem(pbuf + wr, &chunk[..n]).is_ok() {
                            let _ = self.arm.write_word(desc + 12, (wr + n as u32) % size);
                            moved = true;
                        }
                    }
                }
            }
        }
        moved
    }
}

impl MonitorCmd for RpTarget {
    fn handle_monitor_cmd(
        &mut self,
        cmd: &[u8],
        mut out: ConsoleOutput<'_>,
    ) -> Result<(), Self::Error> {
        match cmd {
            b"reset" => match self.target_reset(false) {
                Ok(pc) => outputln!(out, "target reset; halted at pc=0x{:08x}", pc),
                Err(e) => outputln!(out, "reset failed: 0x{:x}", err_code(&e)),
            },
            b"reset halt" => match self.target_reset(true) {
                Ok(pc) => outputln!(out, "target reset, caught at reset vector; pc=0x{:08x}", pc),
                Err(e) => outputln!(out, "reset halt failed: 0x{:x}", err_code(&e)),
            },
            #[cfg(feature = "gdb-target-nrf52")]
            b"approtect" if self.family.is_nrf52() => match self.arm.nrf52_approtect_status() {
                Ok((open, _)) => outputln!(
                    out,
                    "approtect: {}",
                    if open {
                        "OPEN (debug enabled)"
                    } else {
                        "CLOSED (locked)"
                    }
                ),
                Err(e) => outputln!(out, "approtect read failed: 0x{:x}", err_code(&e)),
            },
            #[cfg(feature = "gdb-target-nrf52")]
            b"erase_all" if self.family.is_nrf52() => {
                // CTRL-AP ERASEALL: wipes flash+UICR+RAM, the only APPROTECT
                // unlock. Follow with a reconnect so the session is usable.
                outputln!(out, "erasing entire chip (flash+UICR+RAM)...");
                match self.arm.nrf52_erase_all(2_000_000) {
                    Ok(()) => {
                        self.connect_and_halt();
                        outputln!(out, "erase_all done; reconnected + halted");
                    }
                    Err(e) => outputln!(out, "erase_all failed: 0x{:x}", err_code(&e)),
                }
            }
            #[cfg(feature = "gdb-target-nrf54")]
            b"approtect" if self.family.is_nrf54() => match self.arm.nrf54_approtect_status() {
                Ok(open) => outputln!(
                    out,
                    "approtect: {}",
                    if open {
                        "OPEN (debug enabled)"
                    } else {
                        "CLOSED (locked)"
                    }
                ),
                Err(e) => outputln!(out, "approtect read failed: 0x{:x}", err_code(&e)),
            },
            #[cfg(feature = "gdb-target-nrf54")]
            b"erase_all" if self.family.is_nrf54() => {
                outputln!(out, "erasing entire chip (RRAM+RAM)...");
                match self.arm.nrf54_erase_all(2_000_000) {
                    Ok(()) => {
                        self.connect_and_halt();
                        outputln!(out, "erase_all done; reconnected + halted");
                    }
                    Err(e) => outputln!(out, "erase_all failed: 0x{:x}", err_code(&e)),
                }
            }
            b"rtt scan" => self.rtt_scan(&mut out),
            b"rtt status" => match self.rtt.cb {
                Some(cb) => outputln!(
                    out,
                    "rtt: attached to cb 0x{:08x}, channel {}",
                    cb,
                    self.rtt.channel
                ),
                None => outputln!(out, "rtt: not attached"),
            },
            b"rtt dump" => self.rtt_dump(&mut out),
            b"rtt stop" => {
                self.rtt.cb = None;
                self.rtt.up_desc = None;
                self.rtt.down_desc = None;
                outputln!(out, "rtt: detached");
            }
            _ if cmd.starts_with(b"rtt attach ") || cmd.starts_with(b"rtt setup ") => {
                let arg = cmd.split(|&b| b == b' ').last().unwrap_or(b"");
                match parse_hex(arg) {
                    Some(addr) => {
                        // Verify the ID before accepting the address.
                        let mut id = [0u8; 16];
                        if self.arm.read_mem(addr, &mut id).is_ok() && id == rtt_id() {
                            self.rtt.cb = Some(addr);
                            self.rtt_cache_descs();
                            outputln!(out, "rtt: attached to cb 0x{:08x}", addr);
                            self.rtt_report_cb(addr, &mut out);
                        } else {
                            outputln!(out, "rtt: no control block at 0x{:08x}", addr);
                        }
                    }
                    None => outputln!(out, "rtt: bad address"),
                }
            }
            _ if cmd.starts_with(b"rtt channel ") => {
                let arg = cmd.split(|&b| b == b' ').last().unwrap_or(b"");
                match parse_hex(arg) {
                    Some(ch) if ch < 16 => {
                        self.rtt.channel = ch;
                        self.rtt_cache_descs();
                        outputln!(out, "rtt: channel {}", ch);
                    }
                    _ => outputln!(out, "rtt: bad channel"),
                }
            }
            _ => {
                outputln!(out, "unknown command; available:");
                outputln!(out, "  monitor reset / reset halt");
                #[cfg(feature = "gdb-target-nrf52")]
                outputln!(out, "  monitor approtect / erase_all");
                outputln!(
                    out,
                    "  monitor rtt scan|attach <addr>|setup <addr>|channel <n>|dump|status|stop"
                );
            }
        }
        Ok(())
    }
}

impl MultiThreadBase for RpTarget {
    fn read_registers(&mut self, regs: &mut ArmCoreRegs, tid: Tid) -> TargetResult<(), Self> {
        self.select_core(tid_core(tid)).map_err(Self::map_err)?;
        for (i, r) in regs.r.iter_mut().enumerate() {
            *r = self.arm.read_core_reg(i as u8).map_err(Self::map_err)?;
        }
        regs.sp = self.arm.read_core_reg(cm::SP).map_err(Self::map_err)?;
        regs.lr = self.arm.read_core_reg(cm::LR).map_err(Self::map_err)?;
        regs.pc = self.arm.read_core_reg(cm::PC).map_err(Self::map_err)?;
        regs.cpsr = self.arm.read_core_reg(cm::XPSR).map_err(Self::map_err)?;
        Ok(())
    }

    fn write_registers(&mut self, regs: &ArmCoreRegs, tid: Tid) -> TargetResult<(), Self> {
        self.select_core(tid_core(tid)).map_err(Self::map_err)?;
        for (i, r) in regs.r.iter().enumerate() {
            self.arm
                .write_core_reg(i as u8, *r)
                .map_err(Self::map_err)?;
        }
        self.arm
            .write_core_reg(cm::SP, regs.sp)
            .map_err(Self::map_err)?;
        self.arm
            .write_core_reg(cm::LR, regs.lr)
            .map_err(Self::map_err)?;
        self.arm
            .write_core_reg(cm::PC, regs.pc)
            .map_err(Self::map_err)?;
        self.arm
            .write_core_reg(cm::XPSR, regs.cpsr)
            .map_err(Self::map_err)?;
        Ok(())
    }

    #[inline(always)]
    fn list_active_threads(
        &mut self,
        thread_is_active: &mut dyn FnMut(Tid),
    ) -> Result<(), Self::Error> {
        for core in 0..self.family.num_cores() {
            thread_is_active(core_tid(core));
        }
        Ok(())
    }

    // Memory is shared between the cores; serve reads/writes through
    // whichever DP the link currently addresses.
    fn read_addrs(&mut self, start: u32, data: &mut [u8], _tid: Tid) -> TargetResult<usize, Self> {
        // Diagnostic window: serve reads of 0xF000_0000.. from `diag` instead
        // of the target, so internals are visible even with a dead SWD link.
        let diag_len = (self.diag.len() * 4) as u32;
        if start >= DIAG_BASE && start.wrapping_sub(DIAG_BASE) < diag_len {
            let off = (start - DIAG_BASE) as usize;
            let mut bytes = [0u8; 44];
            for (i, w) in self.diag.iter().enumerate() {
                bytes[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            let n = data.len().min(bytes.len() - off);
            data[..n].copy_from_slice(&bytes[off..off + n]);
            return Ok(n);
        }
        // Reading the flash window requires leaving flash-command mode.
        if self.family.in_flash_mode()
            && start >= self.family.flash_base()
            && start < self.family.flash_base() + self.family.flash_size()
        {
            let _ = self.select_core(0);
            self.family.flash_finish(&mut self.arm);
        }
        let r = self.arm.read_mem(start, data);
        self.diag_err(r).map_err(Self::map_err)?;
        Ok(data.len())
    }

    fn write_addrs(&mut self, start: u32, data: &[u8], _tid: Tid) -> TargetResult<(), Self> {
        if data.is_empty() {
            return Ok(());
        }
        // Writes into the flash window are programmed by the family.
        if (self.family.flash_base()..self.family.flash_base() + self.family.flash_size())
            .contains(&start)
        {
            let sel = self.select_core(0);
            let r = sel.and_then(|_| self.family.flash_write(&mut self.arm, start, data));
            return self.diag_err(r).map_err(Self::map_err);
        }
        let r = self.arm.write_mem(start, data);
        self.diag_err(r).map_err(Self::map_err)
    }

    fn support_resume(&mut self) -> Option<MultiThreadResumeOps<'_, Self>> {
        Some(self)
    }
}

impl MultiThreadResume for RpTarget {
    fn clear_resume_actions(&mut self) -> Result<(), Self::Error> {
        self.actions = [CoreAction::Default; 2];
        self.sched_lock = false;
        Ok(())
    }

    fn set_resume_action_continue(
        &mut self,
        tid: Tid,
        _signal: Option<Signal>,
    ) -> Result<(), Self::Error> {
        self.actions[tid_core(tid)] = CoreAction::Continue;
        Ok(())
    }

    fn resume(&mut self) -> Result<(), Self::Error> {
        self.flash_finish(); // code may run from the flash window
        self.stepped = None;
        // Steps complete synchronously (arm.step waits for the re-halt), so
        // stepped cores are not marked running; the stop poll then reports
        // DoneStep.
        for core in 0..self.family.num_cores() {
            let action = match self.actions[core] {
                CoreAction::Default if self.sched_lock => continue, // stay halted
                CoreAction::Default => CoreAction::Continue,
                explicit => explicit,
            };
            match action {
                CoreAction::Default => unreachable!(),
                CoreAction::Step => {
                    if self.select_core(core).is_ok() {
                        self.stepped = Some(core);
                        let _ = self.arm.step();
                    }
                }
                CoreAction::Continue => {
                    if self.select_core(core).is_ok() && self.arm.resume().is_ok() {
                        self.running[core] = true;
                    }
                }
            }
        }
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<MultiThreadSingleStepOps<'_, Self>> {
        Some(self)
    }

    fn support_scheduler_locking(&mut self) -> Option<MultiThreadSchedulerLockingOps<'_, Self>> {
        Some(self)
    }
}

impl MultiThreadSchedulerLocking for RpTarget {
    fn set_resume_action_scheduler_lock(&mut self) -> Result<(), Self::Error> {
        self.sched_lock = true;
        Ok(())
    }
}

impl MultiThreadSingleStep for RpTarget {
    fn set_resume_action_step(
        &mut self,
        tid: Tid,
        _signal: Option<Signal>,
    ) -> Result<(), Self::Error> {
        self.actions[tid_core(tid)] = CoreAction::Step;
        Ok(())
    }
}

impl Breakpoints for RpTarget {
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        Some(self)
    }
    fn support_hw_breakpoint(&mut self) -> Option<HwBreakpointOps<'_, Self>> {
        Some(self)
    }
    fn support_hw_watchpoint(&mut self) -> Option<HwWatchpointOps<'_, Self>> {
        Some(self)
    }
}

impl HwBreakpoint for RpTarget {
    fn add_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        // FPB comparators are per-core; arm both so either core traps.
        self.fpb_set_both(addr)
    }

    fn remove_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.fpb_clear_both(addr)
    }
}

fn watch_access(kind: WatchKind) -> WatchAccess {
    match kind {
        WatchKind::Read => WatchAccess::Read,
        WatchKind::Write => WatchAccess::Write,
        WatchKind::ReadWrite => WatchAccess::ReadWrite,
    }
}

impl HwWatchpoint for RpTarget {
    fn add_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        // DWT comparators are per-core; arm both so either core traps.
        for core in 0..self.family.num_cores() {
            self.select_core(core).map_err(Self::map_err)?;
            let ok = self
                .arm
                .watchpoint_set(addr, len, watch_access(kind))
                .map_err(Self::map_err)?;
            if !ok {
                if core == 1 {
                    let _ = self.select_core(0);
                    let _ = self.arm.watchpoint_clear(addr, len, watch_access(kind));
                }
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn remove_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        let mut any = false;
        for core in 0..self.family.num_cores() {
            self.select_core(core).map_err(Self::map_err)?;
            any |= self
                .arm
                .watchpoint_clear(addr, len, watch_access(kind))
                .map_err(Self::map_err)?;
        }
        Ok(any)
    }
}

impl RpTarget {
    /// Arm an FPB comparator for `addr` on both cores (rolls back on partial
    /// failure). Returns false when out of comparators / not breakable.
    fn fpb_set_both(&mut self, addr: u32) -> TargetResult<bool, Self> {
        for core in 0..self.family.num_cores() {
            self.select_core(core).map_err(Self::map_err)?;
            if !self.arm.hw_breakpoint_set(addr).map_err(Self::map_err)? {
                if core == 1 {
                    let _ = self.select_core(0);
                    let _ = self.arm.hw_breakpoint_clear(addr);
                }
                return Ok(false);
            }
        }
        Ok(true)
    }

    fn fpb_clear_both(&mut self, addr: u32) -> TargetResult<bool, Self> {
        let mut any = false;
        for core in 0..self.family.num_cores() {
            self.select_core(core).map_err(Self::map_err)?;
            any |= self.arm.hw_breakpoint_clear(addr).map_err(Self::map_err)?;
        }
        Ok(any)
    }

    /// Whether pc is covered by a Z0 breakpoint that was promoted to FPB
    /// (so its stop must be reported as SwBreak, not HwBreak).
    fn promoted_sw_at(&self, pc: u32) -> bool {
        self.breakpoints
            .iter()
            .any(|b| b.addr == pc && matches!(b.kind, SwBpKind::Fpb))
    }
}

impl SwBreakpoint for RpTarget {
    fn add_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        if self.breakpoints.is_full() {
            return Ok(false);
        }
        // ROM/flash can't be BKPT-patched: promote to an FPB comparator so a
        // plain `break` works there too.
        let kind = if addr < 0x2000_0000 {
            if !self.fpb_set_both(addr)? {
                return Ok(false);
            }
            SwBpKind::Fpb
        } else {
            // RAM: save the original halfword and patch a Thumb BKPT.
            let mut orig = [0u8; 2];
            self.arm.read_mem(addr, &mut orig).map_err(Self::map_err)?;
            self.arm
                .write_mem(addr, &BKPT.to_le_bytes())
                .map_err(Self::map_err)?;
            SwBpKind::Ram {
                orig: u16::from_le_bytes(orig),
            }
        };
        let _ = self.breakpoints.push(SwBp { addr, kind });
        Ok(true)
    }

    fn remove_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        let Some(pos) = self.breakpoints.iter().position(|b| b.addr == addr) else {
            return Ok(false);
        };
        let bp = self.breakpoints.swap_remove(pos);
        match bp.kind {
            SwBpKind::Ram { orig } => {
                self.arm
                    .write_mem(addr, &orig.to_le_bytes())
                    .map_err(Self::map_err)?;
                Ok(true)
            }
            SwBpKind::Fpb => self.fpb_clear_both(addr),
        }
    }
}

// --- USB-CDC as a gdbstub Connection --------------------------------------

const RX_QUEUE_SIZE: usize = 1024;
const TX_QUEUE_SIZE: usize = 2048;
/// RTT bridge queues (second CDC port <-> target ring buffers).
const RTT_TX_QUEUE_SIZE: usize = 2048;
const RTT_RX_QUEUE_SIZE: usize = 256;

/// The idle-side view of the USB connection: lock-free SPSC queues shared
/// with the USBCTRL_IRQ task, which owns the USB device and services it at
/// interrupt priority. This keeps USB responsive during long blocking SWD
/// operations (flash erases, connect sequences) that previously starved the
/// polled `pump()` loop.
struct QueueConn {
    rx: heapless::spsc::Consumer<'static, u8, RX_QUEUE_SIZE>,
    tx: heapless::spsc::Producer<'static, u8, TX_QUEUE_SIZE>,
    /// One byte un-read by drop_stray_acks, delivered by the next read_byte.
    pushback: Option<u8>,
}

impl QueueConn {
    /// Nudge the USB task so it drains the TX queue without waiting for the
    /// next host-initiated bus event.
    fn kick() {
        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ);
    }
    /// Try to read one incoming byte.
    fn read_byte(&mut self) -> Option<u8> {
        if let Some(b) = self.pushback.take() {
            return Some(b);
        }
        self.rx.dequeue()
    }
    /// Drop leading ack bytes ('+'/'-') left over from the previous session.
    /// Anything else — e.g. the next session's opening '$' packet, which can
    /// arrive while the SWD link is being re-established — is pushed back for
    /// the state machine, NOT discarded (a blind purge here loses the new
    /// session's qSupported and hangs that GDB).
    fn drop_stray_acks(&mut self) {
        for _ in 0..500_000 {
            match self.read_byte() {
                Some(b'+') | Some(b'-') => {}
                Some(other) => {
                    self.pushback = Some(other);
                    return;
                }
                None => {}
            }
        }
    }
    /// Flush pending TX (bounded, in case the host stopped reading), then
    /// drop any unread RX, so a new GDB session starts with a clean channel
    /// (stale detach responses desync the next session's RSP).
    fn purge(&mut self) {
        for _ in 0..500_000 {
            if self.tx.len() == 0 {
                break;
            }
            Self::kick();
        }
        self.pushback = None;
        while self.rx.dequeue().is_some() {}
    }
}

impl Connection for QueueConn {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        // If the queue is full, keep nudging the USB task to make room (it
        // preempts this priority, so progress only stalls while the host
        // isn't reading).
        loop {
            match self.tx.enqueue(byte) {
                Ok(()) => return Ok(()),
                Err(_) => Self::kick(),
            }
        }
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Self::kick();
        Ok(())
    }
}

/// Borrow of the connection for one GDB session. The gdbstub state machine
/// carries per-session protocol state (notably no-ack mode negotiated via
/// QStartNoAckMode), so it cannot be reused across sessions: the next GDB
/// starts in ack mode and its leading '+' errors a stub still in no-ack mode.
/// Handing the machine a reborrow lets the session loop rebuild it per
/// session while keeping ownership of the queues.
struct ConnRef<'b>(&'b mut QueueConn);

impl Connection for ConnRef<'_> {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        Connection::write(self.0, byte)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Connection::flush(self.0)
    }
}

/// `#[rtic::app]` bypasses `#[rp2040_hal::entry]`, so the SIO spinlocks that
/// the hal entry point would normally release must be released here.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    rust_dap_rp2040::clear_spinlocks();
}

/// Set by the USB task once the host has configured the device. A plain
/// atomic (thumbv6 supports load/store) instead of an RTIC shared resource.
static USB_CONFIGURED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[rtic::app(device = rp_pico::hal::pac, peripherals = true)]
mod app {
    use super::*;
    use usb_device::class_prelude::UsbBusAllocator;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        // --- USBCTRL_IRQ task ---
        usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
        serial: SerialPort<'static, hal::usb::UsbBus>,
        rtt_serial: SerialPort<'static, hal::usb::UsbBus>,
        rx_prod: heapless::spsc::Producer<'static, u8, RX_QUEUE_SIZE>,
        tx_cons: heapless::spsc::Consumer<'static, u8, TX_QUEUE_SIZE>,
        rtt_tx_cons: heapless::spsc::Consumer<'static, u8, RTT_TX_QUEUE_SIZE>,
        rtt_rx_prod: heapless::spsc::Producer<'static, u8, RTT_RX_QUEUE_SIZE>,
        // --- idle (GDB session loop) ---
        conn: QueueConn,
        target: RpTarget,
        rtt_tx_prod: heapless::spsc::Producer<'static, u8, RTT_TX_QUEUE_SIZE>,
        rtt_rx_cons: heapless::spsc::Consumer<'static, u8, RTT_RX_QUEUE_SIZE>,
    }

    #[init(local = [
        rx_queue: heapless::spsc::Queue<u8, RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        tx_queue: heapless::spsc::Queue<u8, TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        rtt_tx_queue: heapless::spsc::Queue<u8, RTT_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        rtt_rx_queue: heapless::spsc::Queue<u8, RTT_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        USB_ALLOCATOR: Option<UsbBusAllocator<hal::usb::UsbBus>> = None,
    ])]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = rp_pico::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let usb_allocator =
            ctx.local
                .USB_ALLOCATOR
                .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                    ctx.device.USBCTRL_REGS,
                    ctx.device.USBCTRL_DPRAM,
                    clocks.usb_clock,
                    true,
                    &mut resets,
                )));
        // Interface order fixes host tty numbering: first CDC = RSP,
        // second CDC = RTT terminal.
        let serial = SerialPort::new(usb_allocator);
        let rtt_serial = SerialPort::new(usb_allocator);
        let usb_dev = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
            .manufacturer("fugafuga.org")
            .product("rust-dap GDB server")
            .serial_number("raspberry-pi-pico-gdb")
            // Two CDC-ACM functions -> IAD composite device.
            .device_class(USB_CLASS_MISCELLANEOUS)
            .device_sub_class(USB_SUBCLASS_COMMON)
            .device_protocol(USB_PROTOCOL_IAD)
            .build();

        // Bit-banging SWD transport + arm-debug.
        let swclk = PicoBidirPin::new(pins.gpio2.into_floating_input());
        let swdio = PicoBidirPin::new(pins.gpio3.into_floating_input());
        let reset = PicoBidirPin::new(pins.gpio4.into_floating_input());
        let swd = SwdIoSet::new(swclk, swdio, reset, CortexMDelay);
        let config = DapConfig::new(
            DapIdentity {
                serial_number: "raspberry-pi-pico-gdb",
                ..DapIdentity::default()
            },
            clocks.system_clock.freq().to_Hz(),
        );
        let arm = ArmDebug::new(swd, config);

        let target = RpTarget {
            arm,
            breakpoints: heapless::Vec::new(),
            family: Family::default_family(),
            actions: [CoreAction::Default; MAX_CORES],
            running: [false; MAX_CORES],
            stepped: None,
            sched_lock: false,
            diag: [
                0,
                0,
                0,
                0,
                0,
                0,
                0,
                0xd1a6_d1a6,
                // Reset site + count of the previous boot (survive sys_reset).
                unsafe { WATCHDOG_SCRATCH0.read_volatile() },
                unsafe { WATCHDOG_SCRATCH1.read_volatile() },
                0,
            ],
            rtt: RttState {
                cb: None,
                channel: 0,
                up_desc: None,
                down_desc: None,
            },
        };

        let (rx_prod, rx_cons) = ctx.local.rx_queue.split();
        let (tx_prod, tx_cons) = ctx.local.tx_queue.split();
        let (rtt_tx_prod, rtt_tx_cons) = ctx.local.rtt_tx_queue.split();
        let (rtt_rx_prod, rtt_rx_cons) = ctx.local.rtt_rx_queue.split();
        let conn = QueueConn {
            rx: rx_cons,
            tx: tx_prod,
            pushback: None,
        };

        (
            Shared {},
            Local {
                usb_dev,
                serial,
                rtt_serial,
                rx_prod,
                tx_cons,
                rtt_tx_cons,
                rtt_rx_prod,
                conn,
                target,
                rtt_tx_prod,
                rtt_rx_cons,
            },
            init::Monotonics(),
        )
    }

    /// Service USB at interrupt priority: enumeration and CDC transfers stay
    /// responsive while idle blocks in long SWD operations.
    #[task(binds = USBCTRL_IRQ, priority = 2, local = [usb_dev, serial, rtt_serial, rx_prod, tx_cons, rtt_tx_cons, rtt_rx_prod])]
    fn usb_irq(ctx: usb_irq::Context) {
        let usb_dev = ctx.local.usb_dev;
        let serial = ctx.local.serial;
        let rtt_serial = ctx.local.rtt_serial;
        usb_dev.poll(&mut [serial, rtt_serial]);
        // 1200 bps touch → reboot into the bootloader (reflash without BOOTSEL).
        rust_dap_rp2040::util::bootsel_on_1200bps_touch(serial);
        USB_CONFIGURED.store(
            usb_dev.state() == UsbDeviceState::Configured,
            core::sync::atomic::Ordering::Relaxed,
        );
        // RX: CDC → queue (drop on overflow; RSP retransmits via its acks).
        let mut buf = [0u8; 64];
        while let Ok(n) = serial.read(&mut buf) {
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                let _ = ctx.local.rx_prod.enqueue(b);
            }
        }
        // TX: queue → CDC. Unlike the old polled loop there is no "next
        // iteration" to push the write buffer out, so flush explicitly —
        // without it a partial packet sits in usbd-serial's buffer forever
        // (no endpoint armed → no further IRQ → deadlock).
        while let Some(&b) = ctx.local.tx_cons.peek() {
            match serial.write(&[b]) {
                Ok(1) => {
                    ctx.local.tx_cons.dequeue();
                }
                _ => break,
            }
        }
        let _ = serial.flush();
        // RTT CDC: drain the up-stream queue to the host, collect host input.
        while let Some(&b) = ctx.local.rtt_tx_cons.peek() {
            match rtt_serial.write(&[b]) {
                Ok(1) => {
                    ctx.local.rtt_tx_cons.dequeue();
                }
                _ => break,
            }
        }
        let _ = rtt_serial.flush();
        while let Ok(n) = rtt_serial.read(&mut buf) {
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                let _ = ctx.local.rtt_rx_prod.enqueue(b);
            }
        }
    }

    #[idle(local = [conn, target, rtt_tx_prod, rtt_rx_cons])]
    fn idle(ctx: idle::Context) -> ! {
        let conn = ctx.local.conn;
        let target = ctx.local.target;
        let rtt_tx = ctx.local.rtt_tx_prod;
        let rtt_rx = ctx.local.rtt_rx_cons;
        // Decimate RTT polling: only on quiet iterations (no RSP byte), and
        // only every N of those, so RSP throughput is unaffected.
        let mut rtt_tick: u32 = 0;
        boot_progress(1); // idle entered

        // Wait for USB enumeration before touching the target, then connect
        // + halt so GDB attaches to stopped cores.
        while !USB_CONFIGURED.load(core::sync::atomic::Ordering::Relaxed) {}
        boot_progress(2); // USB configured
        target.connect_and_halt();
        boot_progress(3); // SWD connected

        // One iteration per GDB session: the gdbstub state machine holds
        // per-session protocol state (e.g. negotiated no-ack mode), so it
        // must be rebuilt from scratch after every disconnect — reusing it
        // makes the next session's opening ack error the stub.
        let mut packet_buffer = [0u8; 1024];
        loop {
            let gdb = match GdbStubBuilder::new(ConnRef(conn))
                .with_packet_buffer(&mut packet_buffer)
                .build()
            {
                Ok(g) => g,
                Err(_) => reset_self(1),
            };
            let mut sm = gdb
                .run_state_machine(target)
                .unwrap_or_else(|_| reset_self(2));
            boot_progress(4); // session loop live

            // Drive this session until GDB disconnects. A gdbstub error
            // (e.g. a new GDB attaching while the previous session's state
            // machine is still Running) ends the session the same way — the
            // outer loop rebuilds a fresh stub — instead of rebooting.
            loop {
                let next = match sm {
                    GdbStubStateMachine::Idle(mut inner) => {
                        match inner.borrow_conn().0.read_byte() {
                            Some(b) => inner.incoming_data(target, b).ok(),
                            None => {
                                rtt_tick = rtt_tick.wrapping_add(1);
                                if rtt_tick % 64 == 0 && target.rtt_stream(rtt_tx, rtt_rx) {
                                    QueueConn::kick();
                                }
                                Some(GdbStubStateMachine::Idle(inner))
                            }
                        }
                    }
                    GdbStubStateMachine::Running(mut inner) => {
                        if let Some(b) = inner.borrow_conn().0.read_byte() {
                            // Typically a Ctrl-C (0x03) to interrupt.
                            inner.incoming_data(target, b).ok()
                        } else if let Some(reason) = target.poll_stopped() {
                            // A core stopped on its own (breakpoint /
                            // watchpoint / step done); the others were
                            // halted with it.
                            inner.report_stop(target, reason).ok()
                        } else {
                            rtt_tick = rtt_tick.wrapping_add(1);
                            if rtt_tick % 4 == 0 && target.rtt_stream(rtt_tx, rtt_rx) {
                                QueueConn::kick();
                            }
                            Some(GdbStubStateMachine::Running(inner))
                        }
                    }
                    GdbStubStateMachine::CtrlCInterrupt(inner) => {
                        target.halt_running(None);
                        inner
                            .interrupt_handled(
                                target,
                                Some(MultiThreadStopReason::SignalWithThread {
                                    tid: core_tid(0),
                                    signal: Signal::SIGINT,
                                }),
                            )
                            .ok()
                    }
                    GdbStubStateMachine::Disconnected(_) => None,
                };
                match next {
                    Some(s) => sm = s,
                    None => break,
                }
            }
            target.diag[10] = target.diag[10].wrapping_add(1); // sessions ended
                                                               // GDB detached: the state machine (and its borrow of conn) is
                                                               // dropped. Flush the detach response and drop stale RX,
                                                               // re-establish the SWD link + halt so the next `target remote`
                                                               // sees a clean target state, then swallow the detach ack — but
                                                               // keep any packet bytes: a new session may attach while the SWD
                                                               // link is still being re-established.
            conn.purge();
            target.connect_and_halt();
            conn.drop_stray_acks();
        }
    }
}

// Watchdog scratch registers survive SYSRESETREQ; used to carry the reset
// site + count across reset_self for the diagnostic window.
const WATCHDOG_SCRATCH0: *mut u32 = 0x4005_800c as *mut u32;
const WATCHDOG_SCRATCH1: *mut u32 = 0x4005_8010 as *mut u32;

/// Boot-progress marker at the top of SRAM: survives a 1200bps-touch reboot,
/// so `picotool save -r 0x20041f00 0x20041f08` can show how far the firmware
/// got even when RSP is dead. Written as 0xb007_00XX stage codes.
const BOOT_PROGRESS: *mut u32 = 0x2004_1f00 as *mut u32;

fn boot_progress(stage: u32) {
    unsafe { BOOT_PROGRESS.write_volatile(0xb007_0000 | stage) };
}

/// Record panics like reset sites (0xfa) instead of hanging silently with
/// interrupts still enabled (which keeps USB alive but the stub dead).
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_self(0xfa)
}

/// Last-resort recovery: a gdbstub protocol/state error leaves the stub
/// unusable, so reboot the whole firmware instead of going dead (the old
/// `loop_forever` stopped USB polling, wedging the port until replug).
/// `site` identifies the caller in the diagnostic window (diag[8]).
fn reset_self(site: u32) -> ! {
    unsafe {
        WATCHDOG_SCRATCH0.write_volatile(0x5e1f_0000 | site);
        WATCHDOG_SCRATCH1.write_volatile(WATCHDOG_SCRATCH1.read_volatile().wrapping_add(1));
    }
    // Detach from USB cleanly first — rebooting mid-enumeration can wedge
    // the host's hub port (see util::usb_detach_for_reset).
    rust_dap_rp2040::util::usb_detach_for_reset();
    cortex_m::peripheral::SCB::sys_reset();
}
