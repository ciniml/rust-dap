// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! GDB debugger milestones M1+M2 hardware self-test.
//!
//! Runs on the probe RP2040. Instead of CMSIS-DAP, it drives the connected
//! target RP2040 directly through the `arm-debug` ADIv5 + Cortex-M stack over
//! a bit-banging SWD transport, then reports the result over USB-CDC. This is
//! the on-hardware counterpart to arm-debug's host tests.
//!
//! - M1: SWD connect, DP power-up, MEM-AP read (DPIDR + CHIP_ID).
//! - M2: halt the core, read PC, single-step, confirm PC advanced, resume.
//!
//! Pins: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET (same as the SWD firmware).
//! Open the CDC serial port (assert DTR) to see a line like:
//!   `M1 dpidr=0x0bc11477 chipid=0x20002927 OK | M2 halt OK pc0=... pc1=... OK`

#![no_std]
#![no_main]

use panic_halt as _;

use arm_debug::{rp2040, ArmDebug};
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

fn run_selftest(arm: &mut ArmDebug<Swd>) -> heapless::String<256> {
    let mut line = heapless::String::new();
    match arm.connect_multidrop(rp2040::CORE0_TARGETSEL) {
        Ok(dpidr) => match arm.read_word(rp2040::SYSINFO_CHIP_ID) {
            Ok(chipid) => {
                let ok = (chipid & 0x0fff_ffff) == 0x0000_2927; // RP2040 part
                let _ = write!(
                    line,
                    "M1 dpidr=0x{:08x} chipid=0x{:08x} {} | ",
                    dpidr,
                    chipid,
                    if ok { "OK" } else { "UNEXPECTED" }
                );
                m2_core_control(arm, &mut line);
                // M3 write diagnostics: memory writes go through the MEM-AP
                // and work regardless of the core's run state, but core
                // register access (DCRSR/DCRDR) requires the core halted —
                // m2_core_control resumes it, so re-halt around the register
                // check.
                let ww = arm.write_word(0x2000_1000, 0xcafe_f00d).is_ok();
                let rb = arm.read_word(0x2000_1000).unwrap_or(0);
                let halt_ok = arm.halt().is_ok();
                let wreg_ok = arm.write_core_reg(0, 0x1122_3344).is_ok();
                let r0 = arm.read_core_reg(0).unwrap_or(0xffff_ffff);
                let _ = arm.resume();
                let _ = write!(
                    line,
                    "M3 wword_ok={} readback=0x{:08x} halt_ok={} wreg_ok={} r0=0x{:08x} {}\r\n",
                    ww,
                    rb,
                    halt_ok,
                    wreg_ok,
                    r0,
                    if rb == 0xcafe_f00d && r0 == 0x1122_3344 {
                        "WRITE_OK"
                    } else {
                        "WRITE_BAD"
                    }
                );
            }
            Err(e) => {
                let _ = write!(line, "M1 dpidr=0x{:08x} read_word ERR {:?}\r\n", dpidr, e);
            }
        },
        Err(e) => {
            let _ = write!(line, "M1 connect ERR {:?}\r\n", e);
        }
    }
    line
}

/// M2 demo: halt the core, read PC, single-step, confirm PC advanced, resume.
fn m2_core_control(arm: &mut ArmDebug<Swd>, line: &mut heapless::String<256>) {
    if let Err(e) = arm.halt() {
        let _ = write!(line, "M2 halt ERR {:?}\r\n", e);
        return;
    }
    let pc0 = match arm.read_core_reg(arm_debug::cortex_m::PC) {
        Ok(v) => v,
        Err(e) => {
            let _ = write!(line, "M2 read PC ERR {:?}\r\n", e);
            let _ = arm.resume();
            return;
        }
    };
    if let Err(e) = arm.step() {
        let _ = write!(line, "M2 pc0=0x{:08x} step ERR {:?}\r\n", pc0, e);
        let _ = arm.resume();
        return;
    }
    let pc1 = arm
        .read_core_reg(arm_debug::cortex_m::PC)
        .unwrap_or(0xffff_ffff);
    let halted = arm.is_halted().unwrap_or(false);
    let _ = arm.resume();
    let moved = pc1 != pc0;
    let _ = write!(
        line,
        "M2 halt OK pc0=0x{:08x} pc1=0x{:08x} stepped={} halted={} {}\r\n",
        pc0,
        pc1,
        moved,
        halted,
        if moved && halted { "OK" } else { "CHECK" }
    );
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
        .strings(&[
            usb_device::device::StringDescriptors::new(usb_device::LangID::EN_US)
                .manufacturer("fugafuga.org")
                .product("rust-dap M1 self-test")
                .serial_number("raspberry-pi-pico-m1"),
        ])
        .unwrap()
        .device_class(usbd_serial::USB_CLASS_CDC)
        .build();

    // Bit-banging SWD transport on the SWD pins.
    let swclk = PicoBidirPin::new(pins.gpio2.into_floating_input());
    let swdio = PicoBidirPin::new(pins.gpio3.into_floating_input());
    let reset = PicoBidirPin::new(pins.gpio4.into_floating_input());
    let swd = SwdIoSet::new(swclk, swdio, reset, CortexMDelay);
    let config = DapConfig::new(
        DapIdentity {
            serial_number: "raspberry-pi-pico-m1",
            product_firmware_version: env!("GIT_REV"),
            ..DapIdentity::default()
        },
        clocks.system_clock.freq().to_Hz(),
    );
    let mut arm = ArmDebug::new(swd, config);

    // USB must be polled continuously to enumerate, so the (blocking,
    // bit-banged) self-test can only run once the host has configured the
    // device — running it before the first poll would miss enumeration.
    let mut result: Option<heapless::String<256>> = None;
    let mut offset: usize = 0;
    let mut throttle: u32 = 0;
    loop {
        usb_dev.poll(&mut [&mut serial]);
        // 1200 bps touch → reboot into the bootloader (reflash without BOOTSEL).
        rust_dap_rp2040::util::bootsel_on_1200bps_touch(&serial);
        if result.is_none() && usb_dev.state() == UsbDeviceState::Configured {
            result = Some(run_selftest(&mut arm));
        }
        // Re-emit the captured result, sending the whole string across poll
        // iterations (SerialPort::write only accepts a bounded chunk, so we
        // must advance an offset rather than resend from the start each time).
        if let Some(r) = &result {
            let bytes = r.as_bytes();
            if offset < bytes.len() {
                offset += serial.write(&bytes[offset..]).unwrap_or(0);
            } else {
                throttle = throttle.wrapping_add(1);
                if throttle % 40_000 == 0 {
                    offset = 0; // restart the transmission periodically
                }
            }
        }
    }
}
