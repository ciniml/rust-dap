// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Blink demo payload for the standalone GDB debugger (gdb_server).
//!
//! This firmware runs on the TARGET Pico (not the probe). It blinks the
//! on-board LED (GPIO25) from a busy-wait loop and exposes three globals
//! that GDB can inspect and modify live:
//!
//! - `BLINK_ENABLED` — 0 stops the blinking (LED frozen), nonzero resumes.
//! - `BLINK_DELAY`   — busy-loop iterations per half-period (speed).
//! - `BLINK_COUNT`   — increments on every toggle (watchpoint target).
//!
//! Load it through the probe's gdb_server:
//!
//!   gdb-multiarch target/thumbv6m-none-eabi/release/blink_demo \
//!     -ex 'set architecture armv4t' -ex 'target remote /dev/ttyACMx'
//!   (gdb) load
//!   (gdb) monitor reset
//!
//! See doc/blink-demo.ja.md for the full walkthrough.
//!
//! The clock setup is intentionally omitted: the core runs from the ROSC
//! (~6.5 MHz), so no crystal or PLL is involved and single-stepping the
//! whole loop stays simple.

#![no_std]
#![no_main]

use panic_halt as _;

use embedded_hal::digital::v2::OutputPin;
use rp_pico::hal;

use hal::pac;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// Nonzero = blink; zero = hold the LED in its current state.
#[no_mangle]
pub static mut BLINK_ENABLED: u32 = 1;
/// Busy-wait iterations per half period (~6.5 MHz core; 500_000 ≈ 2 Hz).
#[no_mangle]
pub static mut BLINK_DELAY: u32 = 500_000;
/// Total LED toggles since boot.
#[no_mangle]
pub static mut BLINK_COUNT: u32 = 0;

#[rp_pico::entry]
fn main() -> ! {
    let mut pac = pac::Peripherals::take().unwrap();
    let sio = hal::Sio::new(pac.SIO);
    let pins = rp_pico::Pins::new(pac.IO_BANK0, pac.PADS_BANK0, sio.gpio_bank0, &mut pac.RESETS);
    let mut led = pins.led.into_push_pull_output();
    let mut led_on = false;

    loop {
        // Volatile reads so GDB writes to the globals take effect mid-run.
        let enabled = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(BLINK_ENABLED)) } != 0;
        if enabled {
            led_on = !led_on;
            if led_on {
                let _ = led.set_high();
            } else {
                let _ = led.set_low();
            }
            unsafe {
                let count = core::ptr::read_volatile(core::ptr::addr_of!(BLINK_COUNT));
                core::ptr::write_volatile(core::ptr::addr_of_mut!(BLINK_COUNT), count.wrapping_add(1));
            }
        }
        let delay = unsafe { core::ptr::read_volatile(core::ptr::addr_of!(BLINK_DELAY)) };
        for _ in 0..delay {
            core::hint::spin_loop();
        }
    }
}
