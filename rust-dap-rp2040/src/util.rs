// Copyright 2022 Kenta Ida
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

use crate::line_coding::UartConfig;
use crate::swdio_pin::{PicoSwdInputPin, PicoSwdOutputPin};
use core::result::Result;
use hal::{gpio::PinId, usb::UsbBus};
use rp2040_hal as hal;
use rust_dap::bitbang::{DelayFunc, SwdIoSet};
use rust_dap::{CmsisDap, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON};
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::{class_prelude::UsbBusAllocator, UsbError};
use usbd_serial::SerialPort;

/// DelayFunc implementation which uses cortex_m::asm::delay
pub struct CycleDelay {}
impl DelayFunc for CycleDelay {
    fn cycle_delay(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

pub fn read_usb_serial_byte_cs(usb_serial: &mut SerialPort<UsbBus>) -> Result<u8, UsbError> {
    let mut buf = [0u8; 1];
    match usb_serial.read(&mut buf) {
        Ok(1) => Ok(buf[0]),
        Ok(0) => Err(UsbError::WouldBlock),
        Ok(_) => panic!("USB Serial read extra data."),
        Err(err) => Err(err),
    }
}

pub fn write_usb_serial_byte_cs(
    usb_serial: &mut SerialPort<UsbBus>,
    data: u8,
) -> Result<(), UsbError> {
    let buf = [data; 1];
    match usb_serial.write(&buf) {
        Ok(1) => Ok(()),
        Ok(0) => Err(UsbError::WouldBlock),
        Ok(_) => panic!("USB Serial wrote extra data."),
        Err(err) => Err(err),
    }
}

/// UART configuration and UART peripheral clock frequency.
pub struct UartConfigAndClock {
    /// UART configuraion
    pub config: UartConfig,
    /// UART peripheral clock frequency
    pub clock: embedded_time::rate::Hertz,
}

type PicoUsbBusAllocator = UsbBusAllocator<UsbBus>;

type PicoSwdIoSet<SwdIoPin, SwClkPin> = SwdIoSet<
    PicoSwdInputPin<SwdIoPin>,
    PicoSwdOutputPin<SwdIoPin>,
    PicoSwdInputPin<SwClkPin>,
    PicoSwdOutputPin<SwClkPin>,
    CycleDelay,
>;

/// Initialize SWDIO, USB-UART, CMSIS-DAP and USB BUS.
pub fn initialize_usb<SwdIoPin, SwClkPin, const MAX_PACKET_SIZE: usize>(
    swdio_pin: PicoSwdInputPin<SwdIoPin>,
    swclk_pin: PicoSwdInputPin<SwClkPin>,
    usb_allocator: &'static PicoUsbBusAllocator,
) -> (
    SerialPort<UsbBus>,
    CmsisDap<'static, UsbBus, PicoSwdIoSet<SwdIoPin, SwClkPin>, MAX_PACKET_SIZE>,
    UsbDevice<UsbBus>,
)
where
    SwdIoPin: PinId,
    SwClkPin: PinId,
{
    let swdio = PicoSwdIoSet::<SwdIoPin, SwClkPin>::new(swdio_pin, swclk_pin, CycleDelay {});

    let usb_serial = SerialPort::new(usb_allocator);
    let usb_dap = CmsisDap::new(usb_allocator, swdio);
    let usb_bus = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
        .manufacturer("fugafuga.org")
        .product("CMSIS-DAP")
        .serial_number("test")
        .device_class(USB_CLASS_MISCELLANEOUS)
        .device_class(USB_SUBCLASS_COMMON)
        .device_protocol(USB_PROTOCOL_IAD)
        .composite_with_iads()
        .max_packet_size_0(64)
        .build();
    (usb_serial, usb_dap, usb_bus)
}
