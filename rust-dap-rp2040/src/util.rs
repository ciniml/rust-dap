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
use usb_device::device::UsbVidPid;
use usb_device::UsbError;
use usbd_serial::SerialPort;

#[cfg(not(feature = "bitbang"))]
use crate::pio::{jtag::JtagIoSet as PioJtagIoSet, pio0, swd::SwdIoSet as PioSwdIoSet};
/// PIO SWD transport (see `crate::v3::SwdIoSet` for the bit-banging variant).
#[cfg(not(feature = "bitbang"))]
pub type SwdIoSet<C, D, E> = PioSwdIoSet<pio0::Pin<C>, pio0::Pin<D>, pio0::Pin<E>>;
/// PIO JTAG transport (see `crate::v3::JtagIoSet` for the bit-banging variant).
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
