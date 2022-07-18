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
use core::result::Result;
use hal::usb::UsbBus;
use rp2040_hal as hal;
use rust_dap::{
    CmsisDap, DapCapabilities, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON,
};
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::{class_prelude::UsbBusAllocator, UsbError};
use usbd_serial::SerialPort;

#[cfg(feature = "bitbang")]
use crate::swdio_pin::{PicoSwdInputPin, PicoSwdOutputPin};
#[cfg(feature = "bitbang")]
use rust_dap::bitbang::{DelayFunc, JtagIoSet as BitbangJtagIoSet, SwdIoSet as BitbangSwdIoSet};
#[cfg(feature = "bitbang")]
pub type SwdIoSet<C, D> = BitbangSwdIoSet<
    PicoSwdInputPin<C>,
    PicoSwdOutputPin<C>,
    PicoSwdInputPin<D>,
    PicoSwdOutputPin<D>,
    CycleDelay,
>;
#[cfg(feature = "bitbang")]
pub type JtagIoSet<TCK, TMS, TDI, TDO, TRST, SRST> = BitbangJtagIoSet<
    PicoSwdInputPin<TCK>,
    PicoSwdOutputPin<TCK>,
    PicoSwdInputPin<TMS>,
    PicoSwdOutputPin<TMS>,
    PicoSwdInputPin<TDI>,
    PicoSwdOutputPin<TDI>,
    PicoSwdInputPin<TDO>,
    PicoSwdOutputPin<TDO>,
    PicoSwdInputPin<TRST>,
    PicoSwdOutputPin<TRST>,
    PicoSwdInputPin<SRST>,
    PicoSwdOutputPin<SRST>,
    CycleDelay,
>;

#[cfg(not(feature = "bitbang"))]
use crate::pio::{pio0, SwdIoSet as PioSwdIoSet};
#[cfg(not(feature = "bitbang"))]
pub type SwdIoSet<C, D> = PioSwdIoSet<pio0::Pin<C>, pio0::Pin<D>>;

/// DelayFunc implementation which uses cortex_m::asm::delay
#[cfg(feature = "bitbang")]
pub struct CycleDelay {}
#[cfg(feature = "bitbang")]
impl CycleDelay {
    const CPU_FREQUENCY_HZ: u32 = 120000000;            // Default CPU Frequency.
    const AVERAGE_OPERATION_OVERHEAD_CYCLES: u32 = 60;  // Average SWD/JTAG operation overhead in cycles.
}
#[cfg(feature = "bitbang")]
impl DelayFunc for CycleDelay {
    #[cfg(feature = "set_clock")]
    fn calculate_half_clock_cycles(frequency_hz: u32) -> Option<u32> {
        let cycles = (Self::CPU_FREQUENCY_HZ / 2) / frequency_hz;
        if cycles < Self::AVERAGE_OPERATION_OVERHEAD_CYCLES {
            Some(0)
        } else {
            Some(cycles - Self::AVERAGE_OPERATION_OVERHEAD_CYCLES)
        }
    }

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

/// Initialize SWDIO, USB-UART, CMSIS-DAP and USB BUS.
pub fn initialize_usb<'a, CmsisDapCommandInner, const MAX_PACKET_SIZE: usize>(
    io: CmsisDapCommandInner,
    usb_allocator: &'a PicoUsbBusAllocator,
    serial: &'a str,
    capabilities: DapCapabilities,
) -> (
    SerialPort<'a, UsbBus>,
    CmsisDap<'a, UsbBus, CmsisDapCommandInner, MAX_PACKET_SIZE>,
    UsbDevice<'a, UsbBus>,
)
where
    CmsisDapCommandInner: rust_dap::CmsisDapCommandInner,
{
    let usb_serial = SerialPort::new(usb_allocator);
    let usb_dap = CmsisDap::new(usb_allocator, io, capabilities);
    let usb_bus = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
        .manufacturer("fugafuga.org")
        .product("CMSIS-DAP")
        .serial_number(serial)
        .device_class(USB_CLASS_MISCELLANEOUS)
        .device_class(USB_SUBCLASS_COMMON)
        .device_protocol(USB_PROTOCOL_IAD)
        .composite_with_iads()
        .max_packet_size_0(64)
        .build();
    (usb_serial, usb_dap, usb_bus)
}
