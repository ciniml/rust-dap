// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! GDB debugger milestone M1 hardware self-test.
//!
//! Runs on the probe RP2040. Instead of CMSIS-DAP, it drives the connected
//! target RP2040 directly through the `arm-debug` ADIv5 stack over a
//! bit-banging SWD transport, then reports the result over USB-CDC. This is
//! the on-hardware counterpart to arm-debug's host tests: it proves the SWD
//! connect / DP power-up / MEM-AP read path against real silicon.
//!
//! Pins: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET (same as the SWD firmware).
//! Open the CDC serial port to see lines like:
//!   `M1 dpidr=0x0bc11477 chipid=0x20002927 OK`

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

fn run_selftest(arm: &mut ArmDebug<Swd>) -> heapless::String<96> {
    let mut line = heapless::String::new();
    match arm.connect_multidrop(rp2040::CORE0_TARGETSEL) {
        Ok(dpidr) => match arm.read_word(rp2040::SYSINFO_CHIP_ID) {
            Ok(chipid) => {
                let ok = (chipid & 0x0fff_ffff) == 0x0000_2927; // RP2040 part
                let _ = write!(
                    line,
                    "M1 dpidr=0x{:08x} chipid=0x{:08x} {}\r\n",
                    dpidr,
                    chipid,
                    if ok { "OK" } else { "UNEXPECTED" }
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
        .product("rust-dap M1 self-test")
        .serial_number("raspberry-pi-pico-m1")
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
            ..DapIdentity::default()
        },
        clocks.system_clock.freq().to_Hz(),
    );
    let mut arm = ArmDebug::new(swd, config);

    let result = run_selftest(&mut arm);
    let bytes = result.as_bytes();

    loop {
        usb_dev.poll(&mut [&mut serial]);
        // Continuously emit the captured result so it appears whenever the
        // host opens the port.
        let _ = serial.write(bytes);
        cortex_m::asm::delay(12_000_000);
    }
}
