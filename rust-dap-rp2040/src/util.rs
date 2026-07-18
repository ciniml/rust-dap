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
use usb_device::device::{StringDescriptors, UsbDevice, UsbDeviceBuilder, UsbVidPid};
use usb_device::LangID;
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

/// Cleanly detach from the USB bus before a self-initiated reboot: drop the
/// D+ pull-up so the host sees an orderly disconnect, then give it time to
/// tear the device down (hub debounce + OS driver removal).
///
/// Rebooting the chip while enumerated bounces D+ faster than the host can
/// process — the old device vanishes mid-transaction and the new one attaches
/// before the port state machine has settled. Observed on a Linux xHCI host
/// as the port wedging into endless `SET_ADDRESS` timeouts (-62/-110) that
/// only a physical replug cleared.
pub fn usb_detach_for_reset() {
    unsafe {
        let usb = &*hal::pac::USBCTRL_REGS::ptr();
        usb.sie_ctrl().modify(|_, w| w.pullup_en().clear_bit());
    }
    // ~0.5 s at the 125 MHz default core clock (longer on slower clocks,
    // which is harmless — we are about to reboot anyway).
    cortex_m::asm::delay(60_000_000);
}

/// Reboot the RP2040 into the USB mass-storage / PICOBOOT bootloader
/// (equivalent to holding BOOTSEL at power-on). Never returns.
///
/// Lets a host reflash the board without pressing the physical button.
pub fn reset_to_bootloader() -> ! {
    usb_detach_for_reset();
    // gpio_activity_pin_mask = 0, disable_interface_mask = 0 → expose both the
    // mass-storage and PICOBOOT interfaces.
    hal::rom_data::reset_to_usb_boot(0, 0);
    // reset_to_usb_boot does not return, but the signature is not `!`.
    loop {
        cortex_m::asm::nop();
    }
}

/// "1200 bps touch": if the host has set the USB-CDC line coding to 1200 baud,
/// reboot into the bootloader. Call this from the USB poll loop. This is the
/// same convention Arduino/pico-sdk use, so `stty -F /dev/ttyACMx 1200`
/// (or opening the port at 1200 baud) drops the board into BOOTSEL.
pub fn bootsel_on_1200bps_touch(serial: &SerialPort<UsbBus>) {
    if serial.line_coding().data_rate() == 1200 {
        reset_to_bootloader();
    }
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
        .strings(&[StringDescriptors::new(LangID::EN_US)
            .manufacturer(usb_identity.manufacturer)
            .product(usb_identity.product)
            .serial_number(usb_identity.serial)])
        .unwrap()
        .device_class(USB_CLASS_MISCELLANEOUS)
        .device_class(USB_SUBCLASS_COMMON)
        .device_protocol(USB_PROTOCOL_IAD)
        .composite_with_iads()
        .max_packet_size_0(64)
        .unwrap()
        .build();
    (usb_serial, usb_dap, usb_bus)
}
