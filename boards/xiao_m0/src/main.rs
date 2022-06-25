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
use rust_dap::bitbang::{DelayFunc, SwdIoSet};
use rust_dap::USB_CLASS_MISCELLANEOUS;
use rust_dap::USB_PROTOCOL_IAD;
use rust_dap::USB_SUBCLASS_COMMON;

use embedded_hal::digital::v2::ToggleableOutputPin;

use bsp::{entry, hal, pac};
use hal::clock::GenericClockController;
use hal::gpio::v2::{Output, Pin, PushPull};
use pac::{interrupt, CorePeripherals, Peripherals};
use xiao_m0 as bsp;

use usb_device::bus::UsbBusAllocator;
use xiao_m0::hal::usb::UsbBus;

use rust_dap::CmsisDap;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

mod swdio_pin;
use swdio_pin::*;

// Import pin types.
use hal::gpio::v2::{PA05, PA07, PA18};

type SwdIoPin = PA05;
type SwClkPin = PA07;
type SwdIoInputPin = XiaoSwdInputPin<SwdIoPin>;
type SwdIoOutputPin = XiaoSwdOutputPin<SwdIoPin>;
type SwClkInputPin = XiaoSwdInputPin<SwClkPin>;
type SwClkOutputPin = XiaoSwdOutputPin<SwClkPin>;
type MySwdIoSet =
    SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, CycleDelay>;

struct CycleDelay {}
impl DelayFunc for CycleDelay {
    fn cycle_delay(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let pins = bsp::Pins::new(peripherals.PORT);
    let mut led0 = pins.led0.into_push_pull_output();

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    let swdio = MySwdIoSet::new(
        XiaoSwdInputPin::new(pins.a8.into_floating_input()),
        XiaoSwdInputPin::new(pins.a9.into_floating_input()),
        CycleDelay {},
    );

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_DAP = Some(CmsisDap::new(&bus_allocator, swdio));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x6666, 0x4444))
                .manufacturer("fugafuga.org")
                .product("CMSIS-DAP")
                .serial_number("test")
                .device_class(USB_CLASS_MISCELLANEOUS)
                .device_class(USB_SUBCLASS_COMMON)
                .device_protocol(USB_PROTOCOL_IAD)
                .composite_with_iads()
                .max_packet_size_0(64)
                .build(),
        );
        LED = Some(pins.led1.into_push_pull_output());
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    loop {
        // unsafe {
        //     USB_DAP.as_mut().map(|dap| {
        //         let _ = dap.process();
        //     });
        // }
        cycle_delay(15 * 1024 * 1024);
        led0.toggle().ok();
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus, MySwdIoSet, 64>> = None;
static mut LED: Option<Pin<PA18, Output<PushPull>>> = None;

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                USB_DAP.as_mut().map(|dap| {
                    usb_dev.poll(&mut [serial, dap]);

                    dap.process().ok();

                    let mut buf = [0u8; 64];

                    if let Ok(count) = serial.read(&mut buf) {
                        for (i, c) in buf.iter().enumerate() {
                            if i >= count {
                                break;
                            }
                            serial.write(&[c.clone()]).unwrap();
                            LED.as_mut().map(|led| led.toggle());
                        }
                    };
                });
            });
        });
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}
