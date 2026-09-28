// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Board-independent GDB Remote Serial Protocol target.
//!
//! The gdbstub `Target` implementation backed by the `arm-debug`
//! ADIv5 + Cortex-M stack, generic over any `rust_dap::DapTransport` (e.g. a
//! bit-banged SWD pin set) and any `rust_dap::Delay` provider. Probe firmware
//! (RP2040 RTIC binary, Xous process, ...) supplies the transport, the
//! `Connection` to GDB, and the session loop; everything protocol- and
//! target-chip-related lives here.
//!
//! Target-chip families (RP2040 / nRF52 / nRF54L) are selected with the
//! `gdb-target-*` cargo features, mirroring the original in-binary layout.

#![no_std]

#[cfg(feature = "gdb-target-rp2040")]
use arm_debug::rp2040;
use arm_debug::{cortex_m as cm, ArmDebug, HaltReason, WatchAccess};
use core::convert::Infallible;

use gdbstub::common::{Signal, Tid};
use gdbstub::outputln;
use gdbstub::stub::MultiThreadStopReason;
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

use rust_dap::{DapTransport, Delay};

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

pub fn core_tid(core: usize) -> Tid {
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

pub struct GdbTarget<T: DapTransport, D: Delay> {
    arm: ArmDebug<T>,
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
    /// Busy-wait delay provider (probe-side; used to let the target's bootrom
    /// come up after a system reset).
    delay: D,
    /// Cycles to wait after SYSRESETREQ before reconnecting.
    reset_settle_cycles: u32,
}

/// CC13x2: RAM used for remote ROM calls (all inside the smallest part's
/// 80 KiB SRAM). Stage one 8 KiB flash sector per FlashProgram call.
#[cfg(feature = "gdb-target-cc13x2")]
const CC13X2_TRAMPOLINE: u32 = 0x2000_7000;
#[cfg(feature = "gdb-target-cc13x2")]
const CC13X2_CALL_SP: u32 = 0x2000_8000;
#[cfg(feature = "gdb-target-cc13x2")]
const CC13X2_STAGE: u32 = 0x2000_8000;

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
/// Stack for remote bootrom calls (grows down).
#[cfg(feature = "gdb-target-rp2040")]
const TARGET_CALL_SP: u32 = 0x2000_8000;
/// BKPT return trampoline for remote calls.
#[cfg(feature = "gdb-target-rp2040")]
const TARGET_TRAMPOLINE: u32 = 0x2000_7000;
/// Resident async flash loader (see `RP2040_LOADER`): code, descriptor and
/// data ring in target RAM. The loader runs on core 0 while the host keeps
/// streaming the next blocks into the ring through the AP, so erase/program
/// time overlaps the SWD transfer instead of serialising with it.
#[cfg(feature = "gdb-target-rp2040")]
const LOADER_CODE: u32 = 0x2000_E000;
#[cfg(feature = "gdb-target-rp2040")]
const LOADER_DESC: u32 = 0x2000_F000;
#[cfg(feature = "gdb-target-rp2040")]
const LOADER_DATA: u32 = 0x2001_0000;
#[cfg(feature = "gdb-target-rp2040")]
const LOADER_SLOTS: u32 = 4;
/// Descriptor layout (words): 0 wp (host), 1 rp (target), 2 stop (host),
/// 3 status, 4 fn_erase, 5 fn_program, 6 data_base, 7 reserved,
/// 8.. hdr[LOADER_SLOTS] = {flash_offset, len, flags(bit0 = erase sector), pad}.
#[cfg(feature = "gdb-target-rp2040")]
const LOADER_DESC_HDR: u32 = LOADER_DESC + 32;
/// Thumb-1 machine code of the loader (assembled from `doc/rp2040-loader.s`):
///   r7 = desc; loop { if wp == rp { if stop { bkpt } continue }
///     hdr = &hdr[rp & 3]; if hdr.flags & 1 { fn_erase(hdr.off & ~0xfff, 4096, 1<<16, 0xD8) }
///     fn_program(hdr.off, data_base + (rp & 3) * 4096, hdr.len); rp += 1 }
#[cfg(feature = "gdb-target-rp2040")]
const RP2040_LOADER: [u32; 22] = [
    0x68384607, 0x42886879, 0x2203d020, 0x0113400a, 0x332019db, 0x68b0461e, 0xd00907c0, 0x0b006830,
    0x21010300, 0x22010309, 0x23d80412, 0x47a0693c, 0x69b96830, 0x2303687a, 0x0312401a, 0x68721889,
    0x47a0697c, 0x31016879, 0xe7da6079, 0x280068b8, 0xbe00d0d7, 0x0000e7fe,
];
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

const HEX: &[u8; 16] = b"0123456789abcdef";

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

/// Minimal fixed-size `fmt::Write` sink for transport diag reports (no_std/no alloc).
mod heapless_fmt {
    pub struct String {
        buf: [u8; 512],
        len: usize,
    }
    impl String {
        pub fn new() -> Self {
            Self {
                buf: [0; 512],
                len: 0,
            }
        }
        pub fn as_str(&self) -> &str {
            core::str::from_utf8(&self.buf[..self.len]).unwrap_or("")
        }
    }
    impl core::fmt::Write for String {
        fn write_str(&mut self, s: &str) -> core::fmt::Result {
            let b = s.as_bytes();
            let n = b.len().min(self.buf.len() - self.len);
            self.buf[self.len..self.len + n].copy_from_slice(&b[..n]);
            self.len += n;
            Ok(())
        }
    }
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
// lives in GdbTarget; the family provides only what differs: the SWD connect,
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
    /// handled by GdbTarget. (Only the non-auto path calls this directly.)
    #[cfg_attr(feature = "gdb-target-auto", allow(dead_code))]
    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError>;
    /// Point the link at `core`'s DP (cheap no-op when already there or when
    /// single-core). Owns the current-core cache.
    fn select_core<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        core: usize,
    ) -> Result<(), arm_debug::ArmError>;
    /// Forget the cached current core (call before a fresh connect).
    fn forget_core(&mut self);
    /// Erase + program flash for the given XIP/flash-window address.
    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError>;
    /// Leave any flash-command mode so the flash window reads/executes again.
    fn flash_finish<T: DapTransport>(&mut self, arm: &mut ArmDebug<T>);
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
    #[cfg(feature = "gdb-target-cc13x2")]
    Cc13x2(Cc13x2Family),
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
            #[cfg(feature = "gdb-target-cc13x2")]
            Family::Cc13x2($f) => $body,
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
        #[cfg(all(
            not(feature = "gdb-target-rp2040"),
            not(feature = "gdb-target-nrf52"),
            not(feature = "gdb-target-nrf54"),
            feature = "gdb-target-cc13x2"
        ))]
        {
            Family::Cc13x2(Cc13x2Family::new())
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
    fn connect_detect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        #[cfg(all(
            feature = "gdb-target-auto",
            any(feature = "gdb-target-nrf52", feature = "gdb-target-nrf54")
        ))]
        {
            // Identify the DP with a read-only probe first and only bring
            // up (ABORT/SELECT/power-up writes) a DP we recognise as nRF.
            // Bringing up whatever answers is not safe: on a multidrop part
            // such as the RP2040 every DP answers after a plain line reset,
            // and the power-up request also reaches the Rescue DP, which
            // resets the chip into a state only SRST recovers from. That is
            // why a live RP2040 needed the SRST attempts of connect_and_halt
            // (and so was reset on every GDB attach) before this check.
            let probed = arm.probe_swd().ok();
            let is_nrf = matches!(
                probed,
                Some(id) if (cfg!(feature = "gdb-target-nrf52") && id == arm_debug::nrf52::DPIDR)
                    || (cfg!(feature = "gdb-target-nrf54") && id == arm_debug::nrf54::DPIDR)
            );
            if is_nrf {
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
    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        on_family!(self, f => f.connect(arm))
    }
    fn select_core<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        on_family!(self, f => f.select_core(arm, core))
    }
    fn forget_core(&mut self) {
        on_family!(self, f => f.forget_core())
    }
    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        on_family!(self, f => f.flash_write(arm, addr, data))
    }
    fn flash_finish<T: DapTransport>(&mut self, arm: &mut ArmDebug<T>) {
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
    /// Register frame of the running resident loader (`Some` while it runs).
    loader: Option<arm_debug::CallFrame>,
    /// Blocks queued to the loader so far (its `wp`).
    loader_wp: u32,
}

#[cfg(feature = "gdb-target-rp2040")]
impl Rp2040Family {
    fn rom_flash_fns<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
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

    fn rom_call<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        fn_addr: u32,
        args: [u32; 4],
        polls: u32,
    ) -> Result<u32, arm_debug::ArmError> {
        self.select_core(arm, 0)?; // bootrom calls always run on core 0
        arm.call_function(fn_addr, &args, TARGET_CALL_SP, TARGET_TRAMPOLINE, polls)
    }

    fn flash_enter<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<(), arm_debug::ArmError> {
        if self.flash_mode {
            return Ok(());
        }
        self.select_core(arm, 0)?;
        let f = self.rom_flash_fns(arm)?;
        self.rom_call(arm, f.connect, [0; 4], POLLS_CALL)?;
        self.rom_call(arm, f.exit_xip, [0; 4], POLLS_CALL)?;
        self.erased = Default::default();
        self.flash_mode = true;
        // Plant and start the resident loader; it idles until blocks arrive.
        let mut code = [0u8; RP2040_LOADER.len() * 4];
        for (chunk, w) in code.chunks_exact_mut(4).zip(RP2040_LOADER) {
            chunk.copy_from_slice(&w.to_le_bytes());
        }
        arm.write_mem(LOADER_CODE, &code)?;
        let desc = [0u32, 0, 0, 0, f.erase, f.program, LOADER_DATA, 0];
        let mut desc_bytes = [0u8; 32];
        for (chunk, w) in desc_bytes.chunks_exact_mut(4).zip(desc) {
            chunk.copy_from_slice(&w.to_le_bytes());
        }
        arm.write_mem(LOADER_DESC, &desc_bytes)?;
        self.loader_wp = 0;
        let frame = arm.start_function(
            LOADER_CODE | 1,
            &[LOADER_DESC, 0, 0, 0],
            TARGET_CALL_SP,
            TARGET_TRAMPOLINE,
        )?;
        self.loader = Some(frame);
        Ok(())
    }

    /// Queue one block (≤ 4 KiB, inside one sector) to the resident loader,
    /// waiting for a free ring slot first. `off` is the flash offset.
    fn loader_push<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        off: u32,
        data: &[u8],
        erase: bool,
    ) -> Result<(), arm_debug::ArmError> {
        if self.loader.is_none() {
            return Err(arm_debug::ArmError::Internal);
        }
        let mut polls = 0u32;
        loop {
            let rp = arm.read_word(LOADER_DESC + 4)?;
            if self.loader_wp.wrapping_sub(rp) < LOADER_SLOTS {
                break;
            }
            // The loader halting on its own means it faulted (or hit a stray
            // breakpoint); a full ring that never drains is a dead loader.
            if arm.is_halted()? {
                return Err(arm_debug::ArmError::CoreTimeout);
            }
            polls += 1;
            if polls > POLLS_ERASE {
                return Err(arm_debug::ArmError::CoreTimeout);
            }
        }
        let slot = self.loader_wp % LOADER_SLOTS;
        arm.write_mem(LOADER_DATA + slot * RP2040_FLASH_SECTOR, data)?;
        let hdr = LOADER_DESC_HDR + slot * 16;
        arm.write_word(hdr, off)?;
        arm.write_word(hdr + 4, data.len() as u32)?;
        arm.write_word(hdr + 8, erase as u32)?;
        self.loader_wp = self.loader_wp.wrapping_add(1);
        arm.write_word(LOADER_DESC, self.loader_wp)?;
        Ok(())
    }

    /// Tell the loader to finish the queue and exit, then wait for its BKPT.
    fn loader_stop<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<(), arm_debug::ArmError> {
        let Some(frame) = self.loader.take() else {
            return Ok(());
        };
        arm.write_word(LOADER_DESC + 8, 1)?;
        // Up to LOADER_SLOTS sector erase+program cycles may still be queued.
        let res = arm.finish_function(frame, POLLS_ERASE.saturating_mul(LOADER_SLOTS));
        if res.is_err() {
            let _ = arm.halt();
        }
        res?;
        if arm.read_word(LOADER_DESC + 4)? != self.loader_wp {
            return Err(arm_debug::ArmError::Internal);
        }
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
            loader: None,
            loader_wp: 0,
        }
    }

    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        let dpidr = arm.connect_multidrop(rp2040::CORE0_TARGETSEL)?;
        self.cur_core = Some(0);
        Ok(dpidr)
    }

    fn select_core<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
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

    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        let off = addr - RP2040_FLASH_BASE;
        let end = off + data.len() as u32;
        if end > RP2040_FLASH_SIZE {
            return Err(arm_debug::ArmError::Internal);
        }
        self.flash_enter(arm)?;
        // Split into 256 B-aligned spans that stay inside one 4 KiB sector and
        // queue each to the resident loader, which erases the sector on its
        // first touch and programs the span. Padding bytes are 0xFF, which
        // programming leaves untouched, so adjacent spans written by separate
        // calls (section boundaries inside a sector) compose correctly.
        let mut pos = off & !0xFF;
        let span_end = (end + 0xFF) & !0xFF;
        let mut buf = [0xFFu8; RP2040_FLASH_SECTOR as usize];
        while pos < span_end {
            let sector_end = (pos & !(RP2040_FLASH_SECTOR - 1)) + RP2040_FLASH_SECTOR;
            let n = (span_end.min(sector_end) - pos) as usize;
            buf[..n].fill(0xFF);
            for (i, slot) in buf[..n].iter_mut().enumerate() {
                let a = pos + i as u32;
                if a >= off && a < end {
                    *slot = data[(a - off) as usize];
                }
            }
            let sector = pos / RP2040_FLASH_SECTOR;
            let (word, bit) = ((sector / 32) as usize, sector % 32);
            let erase = self.erased[word] & (1 << bit) == 0;
            self.erased[word] |= 1 << bit;
            self.loader_push(arm, pos, &buf[..n], erase)?;
            pos += n as u32;
        }
        Ok(())
    }

    fn flash_finish<T: DapTransport>(&mut self, arm: &mut ArmDebug<T>) {
        if !self.flash_mode {
            return;
        }
        if self.select_core(arm, 0).is_err() {
            self.flash_mode = false;
            return;
        }
        let _ = self.loader_stop(arm);
        if let Ok(f) = self.rom_flash_fns(arm) {
            let _ = self.rom_call(arm, f.flush, [0; 4], POLLS_CALL);
            let _ = self.rom_call(arm, f.enter_xip, [0; 4], POLLS_CALL);
        }
        self.flash_mode = false;
    }

    fn reset_flash_state(&mut self) {
        self.flash_mode = false;
        self.loader = None;
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

    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        arm.connect_swd()
    }

    fn select_core<T: DapTransport>(
        &mut self,
        _arm: &mut ArmDebug<T>,
        _core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        Ok(()) // single core
    }

    fn forget_core(&mut self) {}

    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
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

    fn flash_finish<T: DapTransport>(&mut self, _arm: &mut ArmDebug<T>) {}

    fn reset_flash_state(&mut self) {
        self.erased = [0; 4];
    }

    fn in_flash_mode(&self) -> bool {
        false
    }
}

// --- TI CC13x2 / CC26x2 family (Cortex-M4F over cJTAG + ICEPick) ---------

/// The debug port comes up in 2-pin cJTAG (no SWD) with the CPU DAP hidden
/// behind TI's ICEPick router. Connect = activate the transport (OScan1),
/// read the ICEPick IDCODE, link the DAP into the chain, then bring up the
/// JTAG-DP. Flash is programmed synchronously through the ROM API
/// (FlashSectorErase / FlashProgram) with the VIMS cache off.
#[cfg(feature = "gdb-target-cc13x2")]
struct Cc13x2Family {
    /// Sectors erased this session (1 bit per 8 KiB sector; 704 KiB max).
    erased: [u32; 3],
    /// True while VIMS is off for programming.
    flash_mode: bool,
    /// Saved VIMS:CTL to restore at flash_finish.
    vims_ctl: u32,
    /// ROM function addresses, resolved once per session.
    rom_fns: Option<(u32, u32)>,
    /// Actual flash / SRAM size read from the part (fallback: family max).
    flash_size: u32,
    ram_end: u32,
}

#[cfg(feature = "gdb-target-cc13x2")]
impl TargetFamily for Cc13x2Family {
    const NUM_CORES: usize = 1;
    const FLASH_BASE: u32 = arm_debug::cc13x2::FLASH_BASE;
    const FLASH_SIZE: u32 = arm_debug::cc13x2::FLASH_MAX;
    const RAM_START: u32 = arm_debug::cc13x2::SRAM_BASE;
    const RAM_END: u32 = arm_debug::cc13x2::SRAM_BASE + arm_debug::cc13x2::SRAM_MAX;

    fn flash_size(&self) -> u32 {
        self.flash_size
    }
    fn ram_end(&self) -> u32 {
        self.ram_end
    }

    fn new() -> Self {
        Self {
            erased: [0; 3],
            flash_mode: false,
            vims_ctl: 0,
            rom_fns: None,
            flash_size: Self::FLASH_SIZE,
            ram_end: Self::RAM_END,
        }
    }

    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        use arm_debug::cc13x2::*;
        use rust_dap::ConnectPort;
        arm.transport_connect(ConnectPort::Jtag)?;
        // Only the ICEPick is visible after a TAP reset.
        arm.jtag_chain(&[ICEPICK_IR_LEN], 0);
        let mut ids = [0u32; 1];
        arm.jtag_idcodes_after_reset(&mut ids)?;
        if ids[0] & ICEPICK_IDCODE_MASK != ICEPICK_IDCODE & ICEPICK_IDCODE_MASK {
            return Err(arm_debug::ArmError::NoTarget);
        }
        arm.icepick_enable_debug_tap(0)?;
        let (ir_lengths, dap_index) = arm.jtag_probe_icepick_chain()?;
        arm.jtag_chain(&ir_lengths, dap_index);
        let dpidr = arm.connect_jtag()?;
        self.rom_fns = None;
        // Size the memory map from the part (best effort).
        if let Ok(v) = arm.read_word(FLASH_SIZE_REG) {
            let sectors = v & 0xff;
            if sectors != 0 && sectors * FLASH_SECTOR <= FLASH_MAX {
                self.flash_size = sectors * FLASH_SECTOR;
            }
        }
        if let Ok(v) = arm.read_word(SRAM_SIZE_INFO_REG) {
            // OpenOCD's reading of this word: 0..3 -> 32/64/80(?)/144 KiB is
            // undocumented; only trust it to shrink the map, never to grow.
            let kib = match v & 3 {
                0 => 32,
                1 => 64,
                2 => 80,
                _ => 144,
            };
            self.ram_end = SRAM_BASE + kib * 1024;
        }
        Ok(dpidr)
    }

    fn select_core<T: DapTransport>(
        &mut self,
        _arm: &mut ArmDebug<T>,
        _core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        Ok(())
    }

    fn forget_core(&mut self) {}

    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
        addr: u32,
        data: &[u8],
    ) -> Result<(), arm_debug::ArmError> {
        use arm_debug::cc13x2::*;
        let end = addr + data.len() as u32;
        if end > self.flash_size {
            return Err(arm_debug::ArmError::Internal);
        }
        self.flash_enter(arm)?;
        let (erase, program) = self.rom_fns(arm)?;
        // Erase every sector the write touches (once per session), then
        // program in sector-sized, word-aligned spans padded with 0xFF
        // (programming only clears bits, so padding is a no-op).
        let mut pos = addr & !3;
        let span_end = (end + 3) & !3;
        let mut buf = [0xFFu8; FLASH_SECTOR as usize];
        while pos < span_end {
            let sector = pos / FLASH_SECTOR;
            let (w, b) = ((sector / 32) as usize, sector % 32);
            if self.erased[w] & (1 << b) == 0 {
                let status = arm.call_function(
                    erase,
                    &[sector * FLASH_SECTOR, 0, 0, 0],
                    CC13X2_CALL_SP,
                    CC13X2_TRAMPOLINE,
                    POLLS_ERASE,
                )?;
                if status != 0 {
                    return Err(arm_debug::ArmError::Internal);
                }
                self.erased[w] |= 1 << b;
            }
            let sector_end = (sector + 1) * FLASH_SECTOR;
            let n = (span_end.min(sector_end) - pos) as usize;
            buf[..n].fill(0xFF);
            for (i, slot) in buf[..n].iter_mut().enumerate() {
                let a = pos + i as u32;
                if a >= addr && a < end {
                    *slot = data[(a - addr) as usize];
                }
            }
            arm.write_mem(CC13X2_STAGE, &buf[..n])?;
            let status = arm.call_function(
                program,
                &[CC13X2_STAGE, pos, n as u32, 0],
                CC13X2_CALL_SP,
                CC13X2_TRAMPOLINE,
                POLLS_ERASE,
            )?;
            if status != 0 {
                return Err(arm_debug::ArmError::Internal);
            }
            pos += n as u32;
        }
        Ok(())
    }

    fn flash_finish<T: DapTransport>(&mut self, arm: &mut ArmDebug<T>) {
        use arm_debug::cc13x2::*;
        if !self.flash_mode {
            return;
        }
        // ROM flash calls leave standby disabled; re-enable, then restore
        // the cache mode saved at flash_enter.
        if let Ok(cfg) = arm.read_word(FLASH_CFG_REG) {
            let _ = arm.write_word(FLASH_CFG_REG, cfg & !FLASH_CFG_DIS_STANDBY);
        }
        let _ = arm.write_word(VIMS_CTL, self.vims_ctl);
        let _ = Self::vims_wait(arm);
        self.flash_mode = false;
    }

    fn reset_flash_state(&mut self) {
        self.erased = [0; 3];
        self.flash_mode = false;
        self.rom_fns = None;
    }

    fn in_flash_mode(&self) -> bool {
        self.flash_mode
    }
}

#[cfg(feature = "gdb-target-cc13x2")]
impl Cc13x2Family {
    /// Wait for VIMS to finish a mode change (bounded).
    fn vims_wait<T: DapTransport>(arm: &mut ArmDebug<T>) -> Result<(), arm_debug::ArmError> {
        use arm_debug::cc13x2::*;
        for _ in 0..10_000 {
            if arm.read_word(VIMS_STAT)? & VIMS_STAT_MODE_CHANGING == 0 {
                return Ok(());
            }
        }
        Err(arm_debug::ArmError::CoreTimeout)
    }

    /// Turn the flash cache off (as TI's loader does) before programming.
    fn flash_enter<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<(), arm_debug::ArmError> {
        use arm_debug::cc13x2::*;
        if self.flash_mode {
            return Ok(());
        }
        Self::vims_wait(arm)?;
        self.vims_ctl = arm.read_word(VIMS_CTL)?;
        arm.write_word(VIMS_CTL, self.vims_ctl | VIMS_CTL_OFF)?;
        Self::vims_wait(arm)?;
        self.flash_mode = true;
        Ok(())
    }

    /// (FlashSectorErase, FlashProgram) from the ROM API table.
    fn rom_fns<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<(u32, u32), arm_debug::ArmError> {
        use arm_debug::cc13x2::*;
        if let Some(f) = self.rom_fns {
            return Ok(f);
        }
        let table = arm.read_word(ROM_API_TABLE + ROM_API_FLASH_TABLE_INDEX * 4)?;
        if table == 0 || table == 0xffff_ffff {
            return Err(arm_debug::ArmError::Internal);
        }
        let erase = arm.read_word(table + ROM_FLASH_SECTOR_ERASE_INDEX * 4)?;
        let program = arm.read_word(table + ROM_FLASH_PROGRAM_INDEX * 4)?;
        if erase & 1 == 0 || program & 1 == 0 {
            return Err(arm_debug::ArmError::Internal); // must be Thumb addresses
        }
        self.rom_fns = Some((erase, program));
        Ok((erase, program))
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

    fn connect<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
    ) -> Result<u32, arm_debug::ArmError> {
        // nRF54L is DPv2 single-drop; the plain SW-DP bring-up works.
        arm.connect_swd()
    }

    fn select_core<T: DapTransport>(
        &mut self,
        _arm: &mut ArmDebug<T>,
        _core: usize,
    ) -> Result<(), arm_debug::ArmError> {
        Ok(())
    }

    fn forget_core(&mut self) {}

    fn flash_write<T: DapTransport>(
        &mut self,
        arm: &mut ArmDebug<T>,
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

    fn flash_finish<T: DapTransport>(&mut self, _arm: &mut ArmDebug<T>) {}
    fn reset_flash_state(&mut self) {}
    fn in_flash_mode(&self) -> bool {
        false
    }
}

impl<T: DapTransport, D: Delay> GdbTarget<T, D> {
    fn map_err<E>(_e: E) -> TargetError<Infallible> {
        // arm-debug errors surface to GDB as a generic non-fatal error.
        TargetError::NonFatal
    }

    /// Snapshot of the connect/session diagnostics (see `connect_and_halt`).
    pub fn diag(&self) -> [u32; 11] {
        self.diag
    }

    /// Mutable access to the underlying transport (e.g. for transport-specific
    /// diagnostics at startup).
    pub fn transport_mut(&mut self) -> &mut T {
        self.arm.transport()
    }

    /// Count a failed session operation (diag[5]).
    fn diag_err<R, E>(&mut self, r: Result<R, E>) -> Result<R, E> {
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
    pub fn connect_and_halt(&mut self) {
        self.diag[0] = self.diag[0].wrapping_add(1);
        self.diag[5] = 0;
        self.family.reset_flash_state();
        self.running = [false; MAX_CORES];
        for attempt in 1u32..=4 {
            // Attempts 1-2 connect non-destructively: a connect on a live SWD
            // link has been seen to fail once and succeed on the next try
            // without any reset, so a healthy running target must not be reset
            // (it would destroy the state GDB is attaching to observe). Only
            // if those also fail has the target's program genuinely wedged the
            // debug port — attempts 3-4 pulse the hardware SRST line to force a
            // fresh boot state before retrying.
            if attempt >= 3 {
                let _ = self.arm.reset_pulse(2_000);
            }
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
        self.delay.delay_cycles(self.reset_settle_cycles);
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
    pub fn halt_running(&mut self, except: Option<usize>) {
        for core in 0..self.family.num_cores() {
            if Some(core) != except && self.running[core] && self.select_core(core).is_ok() {
                let _ = self.arm.halt();
            }
        }
        self.running = [false; 2];
    }

    /// While running, check whether any resumed core has halted. If one has,
    /// halt the others (all-stop) and return the stop reason to report.
    pub fn poll_stopped(&mut self) -> Option<MultiThreadStopReason<u32>> {
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

impl<T: DapTransport, D: Delay> Target for GdbTarget<T, D> {
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

impl<T: DapTransport, D: Delay> GdbTarget<T, D> {
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

impl<T: DapTransport, D: Delay> GdbTarget<T, D> {
    /// Recompute the cached up/down descriptor addresses for the selected
    /// control block + channel.
    fn rtt_cache_descs(&mut self) {
        self.rtt.up_desc = self.rtt_channel_desc(true).ok().flatten();
        self.rtt.down_desc = self.rtt_channel_desc(false).ok().flatten();
    }

    /// Take up to `buf.len()` bytes from the attached RTT up channel (target ->
    /// host) and advance the target's RdOff. One contiguous run per call, read
    /// in a single bulk MEM-AP transfer so the per-poll SWD round-trips
    /// (descriptor read + RdOff write-back) are amortised over many bytes.
    /// Returns 0 when nothing is attached, the target ring is empty, or flash
    /// programming is in progress.
    pub fn rtt_read_up(&mut self, buf: &mut [u8]) -> usize {
        if self.family.in_flash_mode() || buf.is_empty() {
            return 0;
        }
        let Some(desc) = self.rtt.up_desc else {
            return 0;
        };
        let Ok((_, pbuf, size, wr, rd)) = self.rtt_desc(desc) else {
            return 0;
        };
        if !(size > 0 && wr < size && rd < size && wr != rd) {
            return 0;
        }
        let run = if wr > rd { wr - rd } else { size - rd };
        let n = (run as usize).min(buf.len());
        if self.arm.read_mem(pbuf + rd, &mut buf[..n]).is_err() {
            return 0;
        }
        let _ = self.arm.write_word(desc + 16, (rd + n as u32) % size);
        n
    }

    /// Forward pending RTT up-channel text to GDB as console output (`O`
    /// packets), for probes whose transport has no spare CDC port for an RTT
    /// terminal. The RSP allows `O` packets at any time while the target is
    /// running, so call this from the session loop in the Running state
    /// (gdbstub ignores a stray ack in that state). Not valid while GDB is
    /// waiting for a command reply — keep the halted case on `monitor rtt
    /// dump`; data that accumulates in the target ring while halted is
    /// forwarded on the next poll after `continue`.
    /// Returns true if a packet was written.
    pub fn rtt_emit_console<C: gdbstub::conn::Connection>(&mut self, conn: &mut C) -> bool {
        const CHUNK: usize = 128;
        let mut data = [0u8; CHUNK];
        let n = self.rtt_read_up(&mut data);
        if n == 0 {
            return false;
        }
        // $O<hex>#<cs>: checksum covers the payload between '$' and '#'.
        let mut pkt = [0u8; 2 + 2 * CHUNK + 3];
        pkt[0] = b'$';
        pkt[1] = b'O';
        let mut len = 2;
        for &b in &data[..n] {
            pkt[len] = HEX[(b >> 4) as usize];
            pkt[len + 1] = HEX[(b & 0xf) as usize];
            len += 2;
        }
        let cs = pkt[1..len].iter().fold(0u8, |a, &b| a.wrapping_add(b));
        pkt[len] = b'#';
        pkt[len + 1] = HEX[(cs >> 4) as usize];
        pkt[len + 2] = HEX[(cs & 0xf) as usize];
        len += 3;
        conn.write_all(&pkt[..len]).is_ok() && conn.flush().is_ok()
    }

    /// One bounded streaming step: move up-buffer bytes into `tx` and `rx`
    /// bytes into the down buffer. Called from the session loop while the
    /// target runs (RAM is readable via the MEM-AP regardless of core state).
    pub fn rtt_stream<const TXN: usize, const RXN: usize>(
        &mut self,
        tx: &mut heapless::spsc::Producer<'_, u8, TXN>,
        rx: &mut heapless::spsc::Consumer<'_, u8, RXN>,
    ) -> bool {
        if self.family.in_flash_mode() {
            return false; // don't interleave with flash programming
        }
        let mut moved = false;
        // Up: target -> host CDC. Read as large a contiguous run as fits the
        // TX queue in one bulk MEM-AP transfer (256 B), so the per-poll SWD
        // round-trips — descriptor read + RdOff write-back — are amortised
        // over many more bytes than a 64 B chunk, lifting throughput.
        let room = tx.capacity() - tx.len();
        if room >= 64 {
            let mut chunk = [0u8; 256];
            let n = room.min(chunk.len());
            let got = self.rtt_read_up(&mut chunk[..n]);
            for &b in &chunk[..got] {
                let _ = tx.enqueue(b);
            }
            moved |= got > 0;
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

impl<T: DapTransport, D: Delay> MonitorCmd for GdbTarget<T, D> {
    fn handle_monitor_cmd(
        &mut self,
        cmd: &[u8],
        mut out: ConsoleOutput<'_>,
    ) -> Result<(), Self::Error> {
        self.flash_finish(); // the resident flash loader may still be running
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
                let arg = cmd.split(|&b| b == b' ').next_back().unwrap_or(b"");
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
                let arg = cmd.split(|&b| b == b' ').next_back().unwrap_or(b"");
                match parse_hex(arg) {
                    Some(ch) if ch < 16 => {
                        self.rtt.channel = ch;
                        self.rtt_cache_descs();
                        outputln!(out, "rtt: channel {}", ch);
                    }
                    _ => outputln!(out, "rtt: bad channel"),
                }
            }
            b"diag" => {
                // Connect/session diagnostics kept by connect_and_halt(), plus
                // whatever the transport wants to report about itself.
                let d = self.diag;
                outputln!(
                    out,
                    "connect: attempts={} ok_attempt={:#x} detect={:#x} halt={:#x} dhcsr={:#x}",
                    d[0],
                    d[1],
                    d[2],
                    d[3],
                    d[4]
                );
                outputln!(
                    out,
                    "session: failed_ops={} dpidr={:#010x} (DIAG_OK={:#x})",
                    d[5],
                    d[6],
                    DIAG_OK
                );
                let mut buf = heapless_fmt::String::new();
                self.arm.transport().diag_report(&mut buf);
                for line in buf.as_str().lines() {
                    outputln!(out, "{}", line);
                }
            }
            _ => {
                outputln!(out, "unknown command; available:");
                outputln!(out, "  monitor diag");
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

impl<T: DapTransport, D: Delay> MultiThreadBase for GdbTarget<T, D> {
    fn read_registers(&mut self, regs: &mut ArmCoreRegs, tid: Tid) -> TargetResult<(), Self> {
        self.flash_finish(); // the resident flash loader may still be running
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
        self.flash_finish(); // the resident flash loader may still be running
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
        if self.family.in_flash_mode()
            && !(start >= DIAG_BASE && start.wrapping_sub(DIAG_BASE) < 64)
        {
            self.flash_finish(); // the resident flash loader may still be running
        }
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
        let flash_window =
            self.family.flash_base()..self.family.flash_base() + self.family.flash_size();
        if !flash_window.contains(&start) {
            self.flash_finish(); // the resident flash loader may still be running
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

impl<T: DapTransport, D: Delay> MultiThreadResume for GdbTarget<T, D> {
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

impl<T: DapTransport, D: Delay> MultiThreadSchedulerLocking for GdbTarget<T, D> {
    fn set_resume_action_scheduler_lock(&mut self) -> Result<(), Self::Error> {
        self.sched_lock = true;
        Ok(())
    }
}

impl<T: DapTransport, D: Delay> MultiThreadSingleStep for GdbTarget<T, D> {
    fn set_resume_action_step(
        &mut self,
        tid: Tid,
        _signal: Option<Signal>,
    ) -> Result<(), Self::Error> {
        self.flash_finish(); // the resident flash loader may still be running
        self.actions[tid_core(tid)] = CoreAction::Step;
        Ok(())
    }
}

impl<T: DapTransport, D: Delay> Breakpoints for GdbTarget<T, D> {
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

impl<T: DapTransport, D: Delay> HwBreakpoint for GdbTarget<T, D> {
    fn add_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.flash_finish(); // the resident flash loader may still be running
                             // FPB comparators are per-core; arm both so either core traps.
        self.fpb_set_both(addr)
    }

    fn remove_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.flash_finish(); // the resident flash loader may still be running
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

impl<T: DapTransport, D: Delay> HwWatchpoint for GdbTarget<T, D> {
    fn add_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        self.flash_finish(); // the resident flash loader may still be running
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

impl<T: DapTransport, D: Delay> GdbTarget<T, D> {
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

impl<T: DapTransport, D: Delay> SwBreakpoint for GdbTarget<T, D> {
    fn add_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.flash_finish(); // the resident flash loader may still be running
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

// --- Construction ----------------------------------------------------------

impl<T: DapTransport, D: Delay> GdbTarget<T, D> {
    /// Wrap an established `ArmDebug` link into a GDB target.
    ///
    /// `reset_settle_cycles` is the busy-wait (in `delay` cycles) between
    /// issuing SYSRESETREQ and reconnecting, long enough for the target's
    /// bootrom to come up (~16 ms). `prev_reset_site` / `prev_reset_count`
    /// seed the diagnostic window with probe-reset breadcrumbs when the probe
    /// can preserve them across its own resets (pass 0 otherwise).
    pub fn new(
        arm: ArmDebug<T>,
        delay: D,
        reset_settle_cycles: u32,
        prev_reset_site: u32,
        prev_reset_count: u32,
    ) -> Self {
        Self {
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
                prev_reset_site,
                prev_reset_count,
                0,
            ],
            rtt: RttState {
                cb: None,
                channel: 0,
                up_desc: None,
                down_desc: None,
            },
            delay,
            reset_settle_cycles,
        }
    }

    /// Count a finished GDB session in the diagnostic window (diag[10]).
    pub fn note_session_end(&mut self) {
        self.diag[10] = self.diag[10].wrapping_add(1);
    }
}
