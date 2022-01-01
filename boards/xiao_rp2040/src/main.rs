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
use cortex_m_rt::entry;

use rust_dap::USB_CLASS_MISCELLANEOUS;
use rust_dap::USB_PROTOCOL_IAD;
use rust_dap::USB_SUBCLASS_COMMON;
use rust_dap::bitbang::*;

use rp_pico::hal;
use hal::pac;
use hal::gpio::{Pin, Output, PushPull};

use hal::pac::interrupt;
use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;

use rust_dap::CmsisDap;
use usb_device::prelude::*;
use usbd_serial::{SerialPort};

use embedded_hal::digital::v2::ToggleableOutputPin;

type SwdIoPin = hal::gpio::bank0::Gpio9;
type SwClkPin = hal::gpio::bank0::Gpio10;
type SwdInputPin<P> = PicoSwdInputPin<P>;
type SwdOutputPin<P> = PicoSwdOutputPin<P>;
type SwdIoInputPin = SwdInputPin<SwdIoPin>;
type SwdIoOutputPin = SwdOutputPin<SwdIoPin>;
type SwClkInputPin = SwdInputPin<SwClkPin>;
type SwClkOutputPin = SwdOutputPin<SwClkPin>;
type MySwdIoSet = SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, CycleDelay>;

mod swdio_pin;
use swdio_pin::*;

struct CycleDelay {}
impl DelayFunc for CycleDelay {
    fn cycle_delay(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}



#[entry]
fn main() -> ! {
    let mut peripherals = pac::Peripherals::take().unwrap();
    
    let sio = hal::Sio::new(peripherals.SIO);
    let pins = rp_pico::Pins::new(
        peripherals.IO_BANK0,
        peripherals.PADS_BANK0,
        sio.gpio_bank0,
        &mut peripherals.RESETS,
    );
    
    let mut watchdog = hal::Watchdog::new(peripherals.WATCHDOG);
    let clocks = hal::clocks::init_clocks_and_plls(
        rp_pico::XOSC_CRYSTAL_FREQ,
        peripherals.XOSC,
        peripherals.CLOCKS,
        peripherals.PLL_SYS,
        peripherals.PLL_USB,
        &mut peripherals.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(UsbBusAllocator::new(
            hal::usb::UsbBus::new(
                peripherals.USBCTRL_REGS,
                peripherals.USBCTRL_DPRAM,
                clocks.usb_clock,
                true,
                &mut peripherals.RESETS,
        )));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    let swdio = MySwdIoSet::new(
        PicoSwdInputPin::new(pins.gpio10.into_floating_input()),
        PicoSwdInputPin::new(pins.gpio9.into_floating_input()),
        CycleDelay{},
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
        LED = Some(pins.led.into_push_pull_output());
    }

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::USBCTRL_IRQ);
    }

    loop {
        // unsafe {
        //     USB_DAP.as_mut().map(|dap| {
        //         let _ = dap.process();
        //     });
        // }
        cortex_m::asm::delay(15 * 1024 * 1024);
        //led0.toggle();
    }
}

static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus, MySwdIoSet, 64>> = None;
static mut LED: Option<Pin<hal::gpio::bank0::Gpio25, Output<PushPull>>> = None;

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
fn USBCTRL_IRQ() {
    poll_usb();
}
