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

use panic_halt as _;

use arm_debug::{cortex_m as cm, rp2040, ArmDebug, HaltReason, WatchAccess};
use core::convert::Infallible;
use rp_pico::hal;

use gdbstub::common::{Signal, Tid};
use gdbstub::conn::Connection;
use gdbstub::stub::state_machine::GdbStubStateMachine;
use gdbstub::stub::{GdbStubBuilder, MultiThreadStopReason};
use gdbstub::target::ext::base::multithread::{
    MultiThreadBase, MultiThreadResume, MultiThreadResumeOps, MultiThreadSchedulerLocking,
    MultiThreadSchedulerLockingOps, MultiThreadSingleStep, MultiThreadSingleStepOps,
};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::outputln;
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
use rust_dap::{DapConfig, DapIdentity};
use rust_dap_rp2040::bitbang::{CortexMDelay, PicoBidirPin, SwdIoSet};
use usb_device::class_prelude::UsbBusAllocator;
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

/// SWD multidrop TARGETSEL per core; GDB thread ids are `core + 1`.
const CORE_TARGETSEL: [u32; 2] = [rp2040::CORE0_TARGETSEL, rp2040::CORE1_TARGETSEL];

fn core_tid(core: usize) -> Tid {
    Tid::new(core + 1).unwrap()
}
fn tid_core(tid: Tid) -> usize {
    (tid.get() - 1).min(1)
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
    breakpoints: heapless::Vec<SwBp, 8>,
    /// Which core's DP the SWD link currently addresses (None = unknown).
    cur_core: Option<usize>,
    /// Pending vCont actions per core.
    actions: [CoreAction; 2],
    /// Cores currently running (resumed and not yet re-halted).
    running: [bool; 2],
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
    /// Cached bootrom flash routine addresses.
    flash_fns: Option<RomFlashFns>,
    /// Whether the target is in flash-command mode (XIP disabled).
    flash_mode: bool,
    /// Sectors erased since entering flash mode (1 bit per 4 KiB sector).
    erased: [u32; (FLASH_SIZE / FLASH_SECTOR / 32) as usize],
}

const DIAG_BASE: u32 = 0xF000_0000;
const DIAG_OK: u32 = 0x0a11_600d;

// --- Flash writing (M5): GDB `load`/memory writes into the XIP window are
// implemented by calling the target's bootrom flash routines.
const FLASH_BASE: u32 = rp2040::FLASH_BASE;
const FLASH_SIZE: u32 = 2 * 1024 * 1024; // Pico W25Q16 (2 MiB)
const FLASH_SECTOR: u32 = 4096;
/// Staging buffer in target RAM for flash_range_program's source data.
const TARGET_STAGE: u32 = 0x2001_0000;
/// Stack for remote bootrom calls (grows down).
const TARGET_CALL_SP: u32 = 0x2000_8000;
/// BKPT return trampoline for remote calls.
const TARGET_TRAMPOLINE: u32 = 0x2000_7000;
/// DHCSR poll budgets: a 4 KiB sector erase takes tens of ms.
const POLLS_CALL: u32 = 100_000;
const POLLS_ERASE: u32 = 400_000;

/// Bootrom flash routine addresses, looked up once per boot.
#[derive(Clone, Copy)]
struct RomFlashFns {
    connect: u32,
    exit_xip: u32,
    erase: u32,
    program: u32,
    flush: u32,
    enter_xip: u32,
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

    /// Point the SWD link at `core`'s DP (cheap no-op when already there).
    fn select_core(&mut self, core: usize) -> Result<(), arm_debug::ArmError> {
        if self.cur_core == Some(core) {
            return Ok(());
        }
        self.cur_core = None; // unknown while switching
        self.arm.reselect(CORE_TARGETSEL[core])?;
        self.cur_core = Some(core);
        Ok(())
    }

    /// Establish the SWD link and halt both cores, retrying from scratch
    /// until a DHCSR read confirms the link is actually usable. A connect
    /// issued on a live link has been observed to fail on hardware; the
    /// failed attempt leaves the target in a state from which a later
    /// attempt succeeds. Records what happened into `diag`.
    fn connect_and_halt(&mut self) {
        self.diag[0] = self.diag[0].wrapping_add(1);
        self.diag[5] = 0;
        self.flash_mode = false; // fresh link: assume XIP state unknown
        self.running = [false; 2];
        for attempt in 1u32..=4 {
            self.cur_core = None;
            self.diag[2] = match self.arm.connect_multidrop(rp2040::CORE0_TARGETSEL) {
                Ok(dpidr) => {
                    self.diag[6] = dpidr;
                    self.cur_core = Some(0);
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
                // Bring core 1 to the same state, then return to core 0.
                if self.select_core(1).is_ok() {
                    let _ = self.arm.halt();
                    let _ = self.arm.debug_units_clear();
                }
                let _ = self.select_core(0);
                return;
            }
        }
        self.diag[1] = 0xdead;
    }

    fn rom_flash_fns(&mut self) -> Result<RomFlashFns, arm_debug::ArmError> {
        if let Some(f) = self.flash_fns {
            return Ok(f);
        }
        let mut get = |code| -> Result<u32, arm_debug::ArmError> {
            self.arm
                .rom_func_lookup(code)?
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

    fn rom_call(&mut self, fn_addr: u32, args: [u32; 4], polls: u32) -> Result<u32, arm_debug::ArmError> {
        self.select_core(0)?; // bootrom calls always run on core 0
        self.arm
            .call_function(fn_addr, &args, TARGET_CALL_SP, TARGET_TRAMPOLINE, polls)
    }

    /// Put the target's flash into command mode (XIP off) for erase/program.
    /// Flash routines always run on core 0.
    fn flash_enter(&mut self) -> Result<(), arm_debug::ArmError> {
        if self.flash_mode {
            return Ok(());
        }
        self.select_core(0)?;
        let f = self.rom_flash_fns()?;
        self.rom_call(f.connect, [0; 4], POLLS_CALL)?;
        self.rom_call(f.exit_xip, [0; 4], POLLS_CALL)?;
        self.erased = Default::default();
        self.flash_mode = true;
        Ok(())
    }

    /// Leave flash-command mode: flush the XIP cache and restore XIP so the
    /// flash window is readable/executable again. Best-effort.
    fn flash_exit(&mut self) {
        if !self.flash_mode {
            return;
        }
        if self.select_core(0).is_err() {
            self.flash_mode = false;
            return;
        }
        if let Ok(f) = self.rom_flash_fns() {
            let _ = self.rom_call(f.flush, [0; 4], POLLS_CALL);
            let _ = self.rom_call(f.enter_xip, [0; 4], POLLS_CALL);
        }
        self.flash_mode = false;
    }

    /// Write `data` to flash at XIP address `addr`: erase not-yet-erased
    /// sectors, then program 256-byte pages padded with 0xFF (programming a
    /// bit to 1 leaves it unchanged, so sequential chunks within one page
    /// compose correctly).
    fn flash_write(&mut self, addr: u32, data: &[u8]) -> Result<(), arm_debug::ArmError> {
        let off = addr - FLASH_BASE;
        let end = off + data.len() as u32;
        if end > FLASH_SIZE {
            return Err(arm_debug::ArmError::Internal);
        }
        self.flash_enter()?;
        let f = self.rom_flash_fns()?;
        for sector in off / FLASH_SECTOR..=(end - 1) / FLASH_SECTOR {
            let (word, bit) = ((sector / 32) as usize, sector % 32);
            if self.erased[word] & (1 << bit) == 0 {
                // (offset, count, block_size, block_cmd) — SDK defaults.
                self.rom_call(
                    f.erase,
                    [sector * FLASH_SECTOR, FLASH_SECTOR, 1 << 16, 0xD8],
                    POLLS_ERASE,
                )?;
                self.erased[word] |= 1 << bit;
            }
        }
        // Program the 256-byte-aligned span covering the chunk, staging
        // through target RAM in buffer-sized pieces.
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
            self.arm.write_mem(TARGET_STAGE, &buf[..n])?;
            self.rom_call(f.program, [pos, TARGET_STAGE, n as u32, 0], POLLS_ERASE)?;
            pos += n as u32;
        }
        Ok(())
    }

    /// System reset via AIRCR.SYSRESETREQ (resets both cores + peripherals),
    /// then re-establish the SWD link with everything halted. With `halt`,
    /// core 0 is caught at the reset vector via DEMCR.VC_CORERESET; without
    /// it, the target runs briefly until the reconnect halts it. Returns
    /// core 0's pc after the reset.
    fn target_reset(&mut self, halt: bool) -> Result<u32, arm_debug::ArmError> {
        self.flash_exit();
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
        for core in 0..2 {
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
        for core in 0..2 {
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
                Ok(pc) => outputln!(
                    out,
                    "target reset, caught at reset vector; pc=0x{:08x}",
                    pc
                ),
                Err(e) => outputln!(out, "reset halt failed: 0x{:x}", err_code(&e)),
            },
            _ => {
                outputln!(out, "unknown command; available:");
                outputln!(out, "  monitor reset       - system reset, halt wherever it lands");
                outputln!(out, "  monitor reset halt  - system reset, halt at the reset vector");
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
        thread_is_active(core_tid(0));
        thread_is_active(core_tid(1));
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
        // Reading the XIP window requires flash to be back in XIP mode.
        if self.flash_mode && start >= FLASH_BASE && start < FLASH_BASE + FLASH_SIZE {
            self.flash_exit();
        }
        let r = self.arm.read_mem(start, data);
        self.diag_err(r).map_err(Self::map_err)?;
        Ok(data.len())
    }

    fn write_addrs(&mut self, start: u32, data: &[u8], _tid: Tid) -> TargetResult<(), Self> {
        if data.is_empty() {
            return Ok(());
        }
        // Writes into the XIP window go through the bootrom flash routines
        // (this is how GDB `load` programs the target).
        if (FLASH_BASE..FLASH_BASE + FLASH_SIZE).contains(&start) {
            let r = self.flash_write(start, data);
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
        self.flash_exit(); // code may run from the XIP window
        self.stepped = None;
        // Steps complete synchronously (arm.step waits for the re-halt), so
        // stepped cores are not marked running; the stop poll then reports
        // DoneStep.
        for core in 0..2 {
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
        for core in 0..2 {
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
        for core in 0..2 {
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
        for core in 0..2 {
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
        for core in 0..2 {
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

/// Connection that queues outgoing bytes; the main loop pumps USB and drains
/// the queue to the CDC endpoint (so Connection::write never blocks on USB).
struct UsbConn<'a> {
    usb_dev: UsbDevice<'a, hal::usb::UsbBus>,
    serial: SerialPort<'a, hal::usb::UsbBus>,
    tx: heapless::Deque<u8, 2048>,
    /// One byte un-read by drop_stray_acks, delivered by the next read_byte.
    pushback: Option<u8>,
}

impl UsbConn<'_> {
    /// Poll USB and flush queued TX to the CDC endpoint.
    fn pump(&mut self) {
        self.usb_dev.poll(&mut [&mut self.serial]);
        // 1200 bps touch → reboot into the bootloader (reflash without BOOTSEL).
        rust_dap_rp2040::util::bootsel_on_1200bps_touch(&self.serial);
        while let Some(&b) = self.tx.front() {
            match self.serial.write(&[b]) {
                Ok(1) => {
                    self.tx.pop_front();
                }
                _ => break,
            }
        }
    }
    /// Try to read one incoming byte.
    fn read_byte(&mut self) -> Option<u8> {
        if let Some(b) = self.pushback.take() {
            return Some(b);
        }
        let mut b = [0u8; 1];
        match self.serial.read(&mut b) {
            Ok(1) => Some(b[0]),
            _ => None,
        }
    }
    /// Drop leading ack bytes ('+'/'-') left over from the previous session.
    /// Anything else — e.g. the next session's opening '$' packet, which can
    /// arrive while the SWD link is being re-established — is pushed back for
    /// the state machine, NOT discarded (a blind purge here loses the new
    /// session's qSupported and hangs that GDB).
    fn drop_stray_acks(&mut self) {
        for _ in 0..50_000 {
            self.pump();
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
    fn configured(&self) -> bool {
        self.usb_dev.state() == UsbDeviceState::Configured
    }
    /// Flush pending TX (bounded, in case the host stopped reading), then
    /// drop whatever is left plus any unread RX, so a new GDB session starts
    /// with a clean channel (stale detach responses desync the next
    /// session's RSP).
    fn purge(&mut self) {
        for _ in 0..200_000 {
            if self.tx.is_empty() {
                break;
            }
            self.pump();
        }
        self.tx.clear();
        let mut sink = [0u8; 64];
        while matches!(self.serial.read(&mut sink), Ok(n) if n > 0) {}
    }
}

impl Connection for UsbConn<'_> {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        // If the queue is full, pump synchronously to make room.
        while self.tx.push_back(byte).is_err() {
            self.pump();
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

/// Borrow of the connection for one GDB session. The gdbstub state machine
/// carries per-session protocol state (notably no-ack mode negotiated via
/// QStartNoAckMode), so it cannot be reused across sessions: the next GDB
/// starts in ack mode and its leading '+' errors a stub still in no-ack mode.
/// Handing the machine a reborrow lets main rebuild it per session while
/// keeping ownership of the USB connection.
struct ConnRef<'b, 'a>(&'b mut UsbConn<'a>);

impl Connection for ConnRef<'_, '_> {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        Connection::write(self.0, byte)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Connection::flush(self.0)
    }
}

#[rp_pico::entry]
fn main() -> ! {
    let pac = pac::Peripherals::take().unwrap();
    let mut watchdog = hal::Watchdog::new(pac.WATCHDOG);
    let mut resets = pac.RESETS;
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut resets);

    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut resets,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let usb_allocator = UsbBusAllocator::new(hal::usb::UsbBus::new(
        pac.USBCTRL_REGS,
        pac.USBCTRL_DPRAM,
        clocks.usb_clock,
        true,
        &mut resets,
    ));
    let serial = SerialPort::new(&usb_allocator);
    let usb_dev = UsbDeviceBuilder::new(&usb_allocator, UsbVidPid(0x6666, 0x4444))
        .manufacturer("fugafuga.org")
        .product("rust-dap GDB server")
        .serial_number("raspberry-pi-pico-gdb")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let mut conn = UsbConn {
        usb_dev,
        serial,
        tx: heapless::Deque::new(),
        pushback: None,
    };

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

    // Wait for USB enumeration before touching the target (keeps enumeration
    // responsive), then connect + halt so GDB attaches to a stopped core.
    while !conn.configured() {
        conn.pump();
    }

    let mut target = RpTarget {
        arm,
        breakpoints: heapless::Vec::new(),
        cur_core: None,
        actions: [CoreAction::Default; 2],
        running: [false; 2],
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
        flash_fns: None,
        flash_mode: false,
        erased: Default::default(),
    };
    target.connect_and_halt();

    // One iteration per GDB session: the gdbstub state machine holds
    // per-session protocol state (e.g. negotiated no-ack mode), so it must be
    // rebuilt from scratch after every disconnect — reusing it makes the next
    // session's opening ack error the stub.
    let mut packet_buffer = [0u8; 1024];
    loop {
        let gdb = match GdbStubBuilder::new(ConnRef(&mut conn))
            .with_packet_buffer(&mut packet_buffer)
            .build()
        {
            Ok(g) => g,
            Err(_) => reset_self(1),
        };
        let mut sm = gdb
            .run_state_machine(&mut target)
            .unwrap_or_else(|_| reset_self(2));

        // Drive this session until GDB disconnects. A gdbstub error (e.g. a
        // new GDB attaching while the previous session's state machine is
        // still Running) ends the session the same way — the outer loop
        // rebuilds a fresh stub — instead of rebooting the firmware.
        loop {
            let next = match sm {
                GdbStubStateMachine::Idle(mut inner) => {
                    inner.borrow_conn().0.pump();
                    match inner.borrow_conn().0.read_byte() {
                        Some(b) => inner.incoming_data(&mut target, b).ok(),
                        None => Some(GdbStubStateMachine::Idle(inner)),
                    }
                }
                GdbStubStateMachine::Running(mut inner) => {
                    inner.borrow_conn().0.pump();
                    if let Some(b) = inner.borrow_conn().0.read_byte() {
                        // Typically a Ctrl-C (0x03) to interrupt.
                        inner.incoming_data(&mut target, b).ok()
                    } else if let Some(reason) = target.poll_stopped() {
                        // A core stopped on its own (breakpoint / watchpoint
                        // / step done); the others were halted with it.
                        inner.report_stop(&mut target, reason).ok()
                    } else {
                        Some(GdbStubStateMachine::Running(inner))
                    }
                }
                GdbStubStateMachine::CtrlCInterrupt(inner) => {
                    target.halt_running(None);
                    inner
                        .interrupt_handled(
                            &mut target,
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
        target.diag[10] = target.diag[10].wrapping_add(1); // sessions ended (any cause)
        // GDB detached: the state machine (and its borrow of conn) is dropped.
        // Flush the detach response and drop stale RX, re-establish the SWD
        // link + halt so the next `target remote` sees a clean target state,
        // then swallow the detach ack — but keep any packet bytes: a new
        // session may attach while the SWD link is still being re-established.
        conn.purge();
        target.connect_and_halt();
        conn.drop_stray_acks();
    }
}

// Watchdog scratch registers survive SYSRESETREQ; used to carry the reset
// site + count across reset_self for the diagnostic window.
const WATCHDOG_SCRATCH0: *mut u32 = 0x4005_800c as *mut u32;
const WATCHDOG_SCRATCH1: *mut u32 = 0x4005_8010 as *mut u32;

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
