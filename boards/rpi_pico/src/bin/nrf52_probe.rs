// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! nRF52 connection probe (milestone M-nRF1 bring-up).
//!
//! Runs on the probe RP2040. Connects to an nRF52 target over plain SW-DP
//! (not RP2040 multidrop), reads the DPIDR and the CTRL-AP APPROTECT status,
//! and — if debug access is open — reads FICR.INFO.PART to identify the chip.
//! Reports over USB-CDC, re-probing continuously so the target can be
//! connected at any time.
//!
//! Pins: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET (same as the other firmwares).
//! Open the CDC port (assert DTR) to watch lines like:
//!   `nRF52 dpidr=0x2ba01477 ctrl-ap idr OK approtect=OPEN part=0x52832`
//!
//! If it prints `approtect=CLOSED`, the part is locked (APPROTECT). The GDB
//! server's `monitor erase_all` (or arm-debug's nrf52_erase_all) can recover
//! it — this destroys all flash/UICR/RAM contents.

#![no_std]
#![no_main]

use panic_halt as _;

use arm_debug::ArmDebug;
use core::fmt::Write as _;
use rp_pico::hal;

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

/// FICR.INFO.PART — reads e.g. 0x52832 on an nRF52832.
const FICR_INFO_PART: u32 = 0x1000_0100;

/// Exercise the Cortex-M core-debug layer (halt/read PC/step/resume). These
/// APIs are the same ones used for RP2040 — this confirms they work
/// unchanged on the nRF52 (ARMv7-M) core.
fn core_check(arm: &mut ArmDebug<Swd>, line: &mut heapless::String<200>) {
    if let Err(e) = arm.halt() {
        let _ = write!(line, "halt ERR {:?}", e);
        return;
    }
    let pc0 = arm
        .read_core_reg(arm_debug::cortex_m::PC)
        .unwrap_or(0xffff_ffff);
    // Instruction at pc0: a self-branch (0xe7fe = `b .`) explains a PC that
    // doesn't move on step (idle loop) — that still proves step executed.
    let insn = arm.read_word(pc0 & !3).unwrap_or(0);
    let insn = if pc0 & 2 != 0 {
        insn >> 16
    } else {
        insn & 0xffff
    } as u16;
    let stepped = arm.step().is_ok();
    let pc1 = arm
        .read_core_reg(arm_debug::cortex_m::PC)
        .unwrap_or(0xffff_ffff);
    let halted = arm.is_halted().unwrap_or(false);
    let _ = arm.resume();
    let moved_or_selfloop = pc1 != pc0 || insn == 0xe7fe;
    let _ = write!(
        line,
        "core halt OK pc0=0x{:08x} insn=0x{:04x} pc1=0x{:08x} stepped={} {}",
        pc0,
        insn,
        pc1,
        stepped,
        if halted && stepped && moved_or_selfloop {
            "OK"
        } else {
            "CHECK"
        }
    );
}

fn probe(arm: &mut ArmDebug<Swd>) -> heapless::String<200> {
    let mut line = heapless::String::new();
    match arm.connect_swd() {
        Ok(dpidr) => {
            let _ = write!(line, "nRF52 dpidr=0x{:08x} ", dpidr);
            match arm.nrf52_approtect_status() {
                Ok((open, idr_ok)) => {
                    let _ = write!(
                        line,
                        "ctrl-ap idr {} approtect={} ",
                        if idr_ok { "OK" } else { "MISMATCH" },
                        if open { "OPEN" } else { "CLOSED" }
                    );
                    if open {
                        // Debug access is available: identify the part, then
                        // exercise the (architecture-common) core-debug layer.
                        match arm.read_word(FICR_INFO_PART) {
                            Ok(part) => {
                                let _ = write!(line, "part=0x{:05x} ", part);
                            }
                            Err(e) => {
                                let _ = write!(line, "part ERR {:?} ", e);
                            }
                        }
                        core_check(arm, &mut line);
                    } else {
                        let _ = write!(line, "(locked; erase_all to recover)");
                    }
                }
                Err(e) => {
                    let _ = write!(line, "ctrl-ap ERR {:?}", e);
                }
            }
        }
        Err(e) => {
            let _ = write!(line, "nRF52 connect ERR {:?} (check wiring/power)", e);
        }
    }
    let _ = write!(line, "\r\n");
    line
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
    let mut serial = SerialPort::new(&usb_allocator);
    let mut usb_dev = UsbDeviceBuilder::new(&usb_allocator, UsbVidPid(0x6666, 0x4444))
        .manufacturer("fugafuga.org")
        .product("rust-dap nRF52 probe")
        .serial_number("raspberry-pi-pico-nrf52")
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    let swclk = PicoBidirPin::new(pins.gpio2.into_floating_input());
    let swdio = PicoBidirPin::new(pins.gpio3.into_floating_input());
    let reset = PicoBidirPin::new(pins.gpio4.into_floating_input());
    let swd = SwdIoSet::new(swclk, swdio, reset, CortexMDelay);
    let config = DapConfig::new(
        DapIdentity {
            serial_number: "raspberry-pi-pico-nrf52",
            ..DapIdentity::default()
        },
        clocks.system_clock.freq().to_Hz(),
    );
    let mut arm = ArmDebug::new(swd, config);

    // Re-probe periodically so the target can be plugged in at any time. The
    // bit-banged probe blocks, so only run it between enumeration polls once
    // the host has configured the device.
    let mut result: Option<heapless::String<200>> = None;
    let mut offset: usize = 0;
    let mut throttle: u32 = 0;
    loop {
        usb_dev.poll(&mut [&mut serial]);
        rust_dap_rp2040::util::bootsel_on_1200bps_touch(&serial);
        if usb_dev.state() != UsbDeviceState::Configured {
            continue;
        }
        if let Some(r) = &result {
            let bytes = r.as_bytes();
            if offset < bytes.len() {
                offset += serial.write(&bytes[offset..]).unwrap_or(0);
                continue;
            }
        }
        // Idle between full transmissions, then re-probe and restart output.
        throttle = throttle.wrapping_add(1);
        if result.is_none() || throttle % 200_000 == 0 {
            result = Some(probe(&mut arm));
            offset = 0;
        }
    }
}
