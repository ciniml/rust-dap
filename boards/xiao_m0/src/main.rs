// Copyright 2021 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#![no_std]
#![no_main]

use panic_halt as _;

use rust_dap::bitbang::BitBangSwd;
use rust_dap::{CmsisDap, DapConfig, DapIdentity, Delay};
use rust_dap::{USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON};

use xiao_m0 as bsp;
// BSP 0.13 (atsamd-hal 0.17) flattened the `gpio::v2` module into `gpio`.
use xiao_m0::hal::gpio::{Output, Pin, PushPull, PA02, PA05, PA07, PA18};

mod swdio_pin;
use swdio_pin::*;

type SwdIoPin = PA05; // D9 = a9
type SwClkPin = PA07; // D8 = a8
type ResetPin = PA02; // D0 = a0
type MySwdIoSet =
    BitBangSwd<XiaoBidirPin<SwClkPin>, XiaoBidirPin<SwdIoPin>, XiaoBidirPin<ResetPin>, CycleDelay>;

/// SAMD21 core clock frequency configured by GenericClockController.
const CORE_CLOCK_HZ: u32 = 48_000_000;

pub struct CycleDelay {}
impl Delay for CycleDelay {
    fn delay_cycles(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

// DAP command processing runs in a low-priority software task (dispatched by
// the unused DAC interrupt) instead of inside the USB interrupt, so USB stays
// responsive during the (blocking, bit-banged) SWD transfers — the same
// restructuring already applied to the RP2040 boards.
#[rtic::app(device = xiao_m0::pac, peripherals = true, dispatchers = [DAC])]
mod app {
    use super::*;
    use usb_device::bus::UsbBusAllocator;
    use usb_device::device::StringDescriptors;
    use usb_device::prelude::*;
    use usb_device::LangID;
    use usbd_serial::SerialPort;
    use xiao_m0::hal::clock::GenericClockController;
    use xiao_m0::hal::usb::UsbBus;

    #[shared]
    struct Shared {
        usb_dev: UsbDevice<'static, UsbBus>,
        usb_serial: SerialPort<'static, UsbBus>,
        usb_dap: CmsisDap<'static, UsbBus, MySwdIoSet, 64>,
    }

    #[local]
    struct Local {
        led: Pin<PA18, Output<PushPull>>,
    }

    #[init(local = [usb_allocator: Option<UsbBusAllocator<UsbBus>> = None])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut peripherals = ctx.device;
        let mut clocks = GenericClockController::with_internal_32kosc(
            peripherals.GCLK,
            &mut peripherals.PM,
            &mut peripherals.SYSCTRL,
            &mut peripherals.NVMCTRL,
        );
        let pins = bsp::Pins::new(peripherals.PORT);

        let usb_allocator = ctx.local.usb_allocator.insert(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));

        // SWD pins: a8 = SWCLK, a9 = SWDIO, a0 = RESET (nSRST, active low).
        let swd = MySwdIoSet::new(
            XiaoBidirPin::new(pins.a8.into_floating_input()),
            XiaoBidirPin::new(pins.a9.into_floating_input()),
            XiaoBidirPin::new(pins.a0.into_floating_input()),
            CycleDelay {},
        );

        let usb_serial = SerialPort::new(usb_allocator);
        let usb_dap = CmsisDap::new(
            usb_allocator,
            swd,
            DapConfig::new(
                DapIdentity {
                    serial_number: "xiao-m0",
                    ..DapIdentity::default()
                },
                CORE_CLOCK_HZ,
            ),
        );
        // usb-device 0.3: strings go through StringDescriptors, and both the
        // strings and max_packet_size_0 builder steps now return Result.
        let usb_dev = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
            .strings(&[StringDescriptors::new(LangID::EN_US)
                .manufacturer("fugafuga.org")
                .product("CMSIS-DAP")
                .serial_number("xiao-m0")])
            .unwrap()
            .device_class(USB_CLASS_MISCELLANEOUS)
            .device_class(USB_SUBCLASS_COMMON)
            .device_protocol(USB_PROTOCOL_IAD)
            .composite_with_iads()
            .max_packet_size_0(64)
            .unwrap()
            .build();

        let led = pins.led1.into_push_pull_output();

        (
            Shared {
                usb_dev,
                usb_serial,
                usb_dap,
            },
            Local { led },
        )
    }

    /// Service USB at interrupt priority: poll the bus, echo the CDC serial,
    /// and defer DAP command processing to the low-priority software task.
    #[task(binds = USB, priority = 2, shared = [usb_dev, usb_serial, usb_dap], local = [led])]
    fn usb(ctx: usb::Context) {
        use embedded_hal::digital::StatefulOutputPin;
        let led = ctx.local.led;
        (
            ctx.shared.usb_dev,
            ctx.shared.usb_serial,
            ctx.shared.usb_dap,
        )
            .lock(|usb_dev, usb_serial, usb_dap| {
                usb_dev.poll(&mut [usb_serial, usb_dap]);
                let mut buf = [0u8; 64];
                if let Ok(count) = usb_serial.read(&mut buf) {
                    for c in &buf[..count] {
                        let _ = usb_serial.write(&[*c]);
                        led.toggle().ok();
                    }
                }
            });
        process::spawn().ok();
    }

    /// Execute a pending DAP command outside the USB interrupt. RTIC 2
    /// software tasks are async; this one has no await points.
    #[task(priority = 1, shared = [usb_dap])]
    async fn process(mut ctx: process::Context) {
        ctx.shared.usb_dap.lock(|dap| {
            let _ = dap.process();
        });
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            cortex_m::asm::wfi();
        }
    }
}
