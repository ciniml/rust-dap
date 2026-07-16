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
    CmsisDap, DapConfig, DapTransport, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD,
    USB_SUBCLASS_COMMON,
};
use usb_device::class_prelude::UsbBusAllocator;
use usb_device::device::{UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::UsbError;
use usbd_serial::SerialPort;

#[cfg(not(feature = "bitbang"))]
use crate::pio::{jtag::JtagIoSet as PioJtagIoSet, pio0, swd::SwdIoSet as PioSwdIoSet};

/// SWD transport selected by the `bitbang` feature (PIO by default).
#[cfg(feature = "bitbang")]
pub use crate::bitbang::SwdIoSet;
#[cfg(not(feature = "bitbang"))]
pub type SwdIoSet<C, D, E> = PioSwdIoSet<pio0::Pin<C>, pio0::Pin<D>, pio0::Pin<E>>;
/// JTAG transport selected by the `bitbang` feature (PIO by default).
#[cfg(feature = "bitbang")]
pub use crate::bitbang::JtagIoSet;
#[cfg(not(feature = "bitbang"))]
pub type JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> = PioJtagIoSet<
    pio0::Pin<Tck>,
    pio0::Pin<Tms>,
    pio0::Pin<Tdi>,
    pio0::Pin<Tdo>,
    pio0::Pin<Trst>,
    pio0::Pin<Srst>,
>;

/// USB device identity used by [`crate::v3::initialize_usb`].
pub struct UsbIdentity<'a> {
    pub vid_pid: UsbVidPid,
    pub manufacturer: &'a str,
    pub product: &'a str,
    pub serial: &'a str,
}

impl Default for UsbIdentity<'static> {
    fn default() -> Self {
        Self {
            vid_pid: UsbVidPid(0x6666, 0x4444),
            manufacturer: "fugafuga.org",
            product: "CMSIS-DAP",
            serial: "rust-dap",
        }
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
    pub clock: fugit::HertzU32,
}

/// Initialize USB-UART, CMSIS-DAP and the USB device.
///
/// `config` carries the DAP identity and the probe core clock;
/// `usb_identity` carries the USB VID/PID and string descriptors.
pub fn initialize_usb<'a, T, const MAX_PACKET_SIZE: usize>(
    transport: T,
    usb_allocator: &'a UsbBusAllocator<UsbBus>,
    usb_identity: UsbIdentity<'a>,
    config: DapConfig,
) -> (
    SerialPort<'a, UsbBus>,
    CmsisDap<'a, UsbBus, T, MAX_PACKET_SIZE>,
    UsbDevice<'a, UsbBus>,
)
where
    T: DapTransport,
{
    let usb_serial = SerialPort::new(usb_allocator);
    let usb_dap = CmsisDap::new(usb_allocator, transport, config);
    let usb_bus = UsbDeviceBuilder::new(usb_allocator, usb_identity.vid_pid)
        .manufacturer(usb_identity.manufacturer)
        .product(usb_identity.product)
        .serial_number(usb_identity.serial)
        .device_class(USB_CLASS_MISCELLANEOUS)
        .device_class(USB_SUBCLASS_COMMON)
        .device_protocol(USB_PROTOCOL_IAD)
        .composite_with_iads()
        .max_packet_size_0(64)
        .build();
    (usb_serial, usb_dap, usb_bus)
}
