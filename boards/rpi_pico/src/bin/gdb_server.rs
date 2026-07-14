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

use arm_debug::{cortex_m as cm, rp2040, ArmDebug};
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
    Breakpoints, BreakpointsOps, SwBreakpoint, SwBreakpointOps,
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
}

impl RpTarget {
    fn map_err<E>(_e: E) -> TargetError<Infallible> {
        // arm-debug errors surface to GDB as a generic non-fatal error.
        TargetError::NonFatal
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
        self.arm.read_mem(start, data).map_err(Self::map_err)?;
        Ok(data.len())
    }

    fn write_addrs(&mut self, start: u32, data: &[u8]) -> TargetResult<(), Self> {
        self.arm.write_mem(start, data).map_err(Self::map_err)
    }

    fn support_resume(&mut self) -> Option<SingleThreadResumeOps<'_, Self>> {
        Some(self)
    }
}

impl SingleThreadResume for RpTarget {
    fn resume(&mut self, _signal: Option<Signal>) -> Result<(), Self::Error> {
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
        self.stepping = true;
        let _ = self.arm.step();
        Ok(())
    }
}

impl Breakpoints for RpTarget {
    fn support_sw_breakpoint(&mut self) -> Option<SwBreakpointOps<'_, Self>> {
        Some(self)
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
    let mut arm = ArmDebug::new(swd, config);

    // Wait for USB enumeration before touching the target (keeps enumeration
    // responsive), then connect + halt so GDB attaches to a stopped core.
    while !conn.configured() {
        conn.pump();
    }
    let _ = arm.connect_multidrop(rp2040::CORE0_TARGETSEL);
    let _ = arm.halt();

    let mut target = RpTarget {
        arm,
        breakpoints: heapless::Vec::new(),
        stepping: false,
    };

    // Run the gdbstub state machine, driven from the USB poll loop.
    let mut packet_buffer = [0u8; 1024];
    let gdb = match GdbStubBuilder::new(conn)
        .with_packet_buffer(&mut packet_buffer)
        .build()
    {
        Ok(g) => g,
        Err(_) => reset_self(),
    };
    let mut sm = gdb
        .run_state_machine(&mut target)
        .unwrap_or_else(|_| reset_self());

    loop {
        sm = match sm {
            GdbStubStateMachine::Idle(mut inner) => {
                inner.borrow_conn().pump();
                match inner.borrow_conn().read_byte() {
                    Some(b) => inner
                        .incoming_data(&mut target, b)
                        .unwrap_or_else(|_| reset_self()),
                    None => GdbStubStateMachine::Idle(inner),
                }
            }
            GdbStubStateMachine::Running(mut inner) => {
                inner.borrow_conn().pump();
                if let Some(b) = inner.borrow_conn().read_byte() {
                    // Typically a Ctrl-C (0x03) to interrupt.
                    inner
                        .incoming_data(&mut target, b)
                        .unwrap_or_else(|_| reset_self())
                } else if target.arm.is_halted().unwrap_or(false) {
                    // The core stopped on its own (breakpoint hit / step done).
                    let reason = if target.stepping {
                        SingleThreadStopReason::DoneStep
                    } else {
                        SingleThreadStopReason::SwBreak(())
                    };
                    inner
                        .report_stop(&mut target, reason)
                        .unwrap_or_else(|_| reset_self())
                } else {
                    GdbStubStateMachine::Running(inner)
                }
            }
            GdbStubStateMachine::CtrlCInterrupt(inner) => {
                let _ = target.arm.halt();
                inner
                    .interrupt_handled(
                        &mut target,
                        Some(SingleThreadStopReason::Signal(Signal::SIGINT)),
                    )
                    .unwrap_or_else(|_| reset_self())
            }
            GdbStubStateMachine::Disconnected(mut inner) => {
                // GDB detached. Flush the detach response, then drop stale
                // TX/RX so the next session's RSP stream starts clean, and
                // re-establish the SWD link + halt from scratch so the next
                // `target remote` sees a clean DP/target state (a bare halt()
                // left the following session hanging).
                inner.borrow_conn().purge();
                let _ = target.arm.connect_multidrop(rp2040::CORE0_TARGETSEL);
                let _ = target.arm.halt();
                inner.return_to_idle()
            }
        };
    }
}

/// Last-resort recovery: a gdbstub protocol/state error leaves the stub
/// unusable, so reboot the whole firmware instead of going dead (the old
/// `loop_forever` stopped USB polling, wedging the port until replug).
fn reset_self() -> ! {
    cortex_m::peripheral::SCB::sys_reset();
}
