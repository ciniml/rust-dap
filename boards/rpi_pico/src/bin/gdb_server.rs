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

use gdbstub::common::Signal;
use gdbstub::conn::Connection;
use gdbstub::stub::state_machine::GdbStubStateMachine;
use gdbstub::stub::{GdbStubBuilder, SingleThreadStopReason};
use gdbstub::target::ext::base::singlethread::{
    SingleThreadBase, SingleThreadResume, SingleThreadResumeOps, SingleThreadSingleStep,
    SingleThreadSingleStepOps,
};
use gdbstub::target::ext::base::BaseOps;
use gdbstub::target::ext::breakpoints::{
    Breakpoints, BreakpointsOps, HwBreakpoint, HwBreakpointOps, HwWatchpoint, HwWatchpointOps,
    SwBreakpoint, SwBreakpointOps, WatchKind,
};
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

struct SwBp {
    addr: u32,
    orig: u16,
}

struct RpTarget {
    arm: ArmDebug<Swd>,
    breakpoints: heapless::Vec<SwBp, 8>,
    /// Whether the last resume was a single-step (so the next stop is
    /// reported as DoneStep rather than SwBreak).
    stepping: bool,
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

    /// Establish the SWD link and halt the core, retrying from scratch until
    /// a DHCSR read confirms the link is actually usable. A connect issued on
    /// a live link has been observed to fail on hardware; the failed attempt
    /// leaves the target in a state from which a later attempt succeeds.
    /// Records what happened into `diag`.
    fn connect_and_halt(&mut self) {
        self.diag[0] = self.diag[0].wrapping_add(1);
        self.diag[5] = 0;
        self.flash_mode = false; // fresh link: assume XIP state unknown
        for attempt in 1u32..=4 {
            self.diag[2] = match self.arm.connect_multidrop(rp2040::CORE0_TARGETSEL) {
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
        self.arm
            .call_function(fn_addr, &args, TARGET_CALL_SP, TARGET_TRAMPOLINE, polls)
    }

    /// Put the target's flash into command mode (XIP off) for erase/program.
    fn flash_enter(&mut self) -> Result<(), arm_debug::ArmError> {
        if self.flash_mode {
            return Ok(());
        }
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

    /// Classify why the core stopped, for GDB's stop reply. DFSR (cleared by
    /// the read) distinguishes watchpoints from breakpoints; an FPB
    /// comparator covering the PC distinguishes hardware from software
    /// breakpoints.
    fn stop_reason(&mut self) -> SingleThreadStopReason<u32> {
        if self.stepping {
            return SingleThreadStopReason::DoneStep;
        }
        match self.arm.halt_reason() {
            Ok(HaltReason::Watchpoint(addr, access)) => SingleThreadStopReason::Watch {
                tid: (),
                kind: match access {
                    WatchAccess::Read => WatchKind::Read,
                    WatchAccess::Write => WatchKind::Write,
                    WatchAccess::ReadWrite => WatchKind::ReadWrite,
                },
                addr,
            },
            Ok(HaltReason::Breakpoint) => {
                let pc = self.arm.read_core_reg(cm::PC).unwrap_or(0);
                if self.arm.hw_breakpoint_at(pc).unwrap_or(false) {
                    SingleThreadStopReason::HwBreak(())
                } else {
                    SingleThreadStopReason::SwBreak(())
                }
            }
            _ => SingleThreadStopReason::SwBreak(()),
        }
    }
}

impl Target for RpTarget {
    type Arch = Armv4t;
    type Error = Infallible;

    fn base_ops(&mut self) -> BaseOps<'_, Self::Arch, Self::Error> {
        BaseOps::SingleThread(self)
    }

    fn support_breakpoints(&mut self) -> Option<BreakpointsOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadBase for RpTarget {
    fn read_registers(&mut self, regs: &mut ArmCoreRegs) -> TargetResult<(), Self> {
        for (i, r) in regs.r.iter_mut().enumerate() {
            *r = self.arm.read_core_reg(i as u8).map_err(Self::map_err)?;
        }
        regs.sp = self.arm.read_core_reg(cm::SP).map_err(Self::map_err)?;
        regs.lr = self.arm.read_core_reg(cm::LR).map_err(Self::map_err)?;
        regs.pc = self.arm.read_core_reg(cm::PC).map_err(Self::map_err)?;
        regs.cpsr = self.arm.read_core_reg(cm::XPSR).map_err(Self::map_err)?;
        Ok(())
    }

    fn write_registers(&mut self, regs: &ArmCoreRegs) -> TargetResult<(), Self> {
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

    fn read_addrs(&mut self, start: u32, data: &mut [u8]) -> TargetResult<usize, Self> {
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

    fn write_addrs(&mut self, start: u32, data: &[u8]) -> TargetResult<(), Self> {
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

    fn support_resume(&mut self) -> Option<SingleThreadResumeOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadResume for RpTarget {
    fn resume(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        self.flash_exit(); // code may run from the XIP window
        self.stepping = false;
        let _ = self.arm.resume();
        Ok(())
    }

    fn support_single_step(&mut self) -> Option<SingleThreadSingleStepOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadSingleStep for RpTarget {
    fn step(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
        self.flash_exit();
        self.stepping = true;
        let _ = self.arm.step();
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
        // FPB comparator (code region 0x0..0x2000_0000; ROM/flash included).
        self.arm.hw_breakpoint_set(addr).map_err(Self::map_err)
    }

    fn remove_hw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        self.arm.hw_breakpoint_clear(addr).map_err(Self::map_err)
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
        self.arm
            .watchpoint_set(addr, len, watch_access(kind))
            .map_err(Self::map_err)
    }

    fn remove_hw_watchpoint(
        &mut self,
        addr: u32,
        len: u32,
        kind: WatchKind,
    ) -> TargetResult<bool, Self> {
        self.arm
            .watchpoint_clear(addr, len, watch_access(kind))
            .map_err(Self::map_err)
    }
}

impl SwBreakpoint for RpTarget {
    fn add_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        // Save the original halfword and patch a Thumb BKPT (RAM only).
        let mut orig = [0u8; 2];
        self.arm.read_mem(addr, &mut orig).map_err(Self::map_err)?;
        let orig = u16::from_le_bytes(orig);
        if self.breakpoints.push(SwBp { addr, orig }).is_err() {
            return Ok(false); // out of breakpoint slots
        }
        self.arm
            .write_mem(addr, &BKPT.to_le_bytes())
            .map_err(Self::map_err)?;
        Ok(true)
    }

    fn remove_sw_breakpoint(
        &mut self,
        addr: u32,
        _kind: ArmBreakpointKind,
    ) -> TargetResult<bool, Self> {
        if let Some(pos) = self.breakpoints.iter().position(|b| b.addr == addr) {
            let bp = self.breakpoints.swap_remove(pos);
            self.arm
                .write_mem(addr, &bp.orig.to_le_bytes())
                .map_err(Self::map_err)?;
            Ok(true)
        } else {
            Ok(false)
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
        let mut b = [0u8; 1];
        match self.serial.read(&mut b) {
            Ok(1) => Some(b[0]),
            _ => None,
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
        stepping: false,
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
                    } else if target.arm.is_halted().unwrap_or(false) {
                        // The core stopped on its own (breakpoint / watchpoint
                        // / step done).
                        let reason = target.stop_reason();
                        inner.report_stop(&mut target, reason).ok()
                    } else {
                        Some(GdbStubStateMachine::Running(inner))
                    }
                }
                GdbStubStateMachine::CtrlCInterrupt(inner) => {
                    let _ = target.arm.halt();
                    inner
                        .interrupt_handled(
                            &mut target,
                            Some(SingleThreadStopReason::Signal(Signal::SIGINT)),
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
        // then purge whatever arrived meanwhile (e.g. GDB's trailing ack).
        conn.purge();
        target.connect_and_halt();
        conn.purge();
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
    cortex_m::peripheral::SCB::sys_reset();
}
