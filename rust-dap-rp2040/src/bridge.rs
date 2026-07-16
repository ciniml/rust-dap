// Copyright 2026 Kenta Ida
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

//! USB-CDC <-> UART bridge building blocks shared by the RTIC board
//! applications. RTIC's `#[app]` macro cannot live in a library, so the
//! boards keep their task skeletons and call into these functions.

use crate::line_coding::UartConfig;
use crate::util::{read_usb_serial_byte_cs, write_usb_serial_byte_cs, UartConfigAndClock};
use embedded_hal_nb::serial::{Read, Write};
use hal::pac::{UART0, UART1};
use hal::uart::{Enabled, Reader, UartDevice, UartPeripheral, ValidUartPinout, Writer};
use hal::usb::UsbBus;
use heapless::spsc::{Consumer, Producer};
use rp2040_hal as hal;
use usbd_serial::SerialPort;

/// UART reader half, wrapped so it can be an RTIC shared resource.
pub struct UartReader<D: UartDevice, P: ValidUartPinout<D>>(pub Reader<D, P>);
/// UART writer half, wrapped so it can be an RTIC shared resource.
pub struct UartWriter<D: UartDevice, P: ValidUartPinout<D>>(pub Writer<D, P>);
// SAFETY: Reader/Writer own their halves of the UART peripheral exclusively
// and are only ever accessed through RTIC resource locks, so moving them
// between task contexts is sound.
unsafe impl<D: UartDevice, P: ValidUartPinout<D>> Send for UartReader<D, P> {}
unsafe impl<D: UartDevice, P: ValidUartPinout<D>> Send for UartWriter<D, P> {}

/// Moves data received from the USB serial into the UART TX queue.
pub fn drain_usb_to_uart_tx<const N: usize>(
    usb_serial: &mut SerialPort<UsbBus>,
    uart_tx_producer: &mut Producer<u8, N>,
) {
    while uart_tx_producer.ready() {
        if let Ok(data) = read_usb_serial_byte_cs(usb_serial) {
            uart_tx_producer.enqueue(data).unwrap();
        } else {
            break;
        }
    }
}

/// Writes queued TX data to the UART.
pub fn drain_uart_tx_queue<D: UartDevice, P: ValidUartPinout<D>, const N: usize>(
    uart_writer: &mut Option<UartWriter<D, P>>,
    uart_tx_consumer: &mut Consumer<u8, N>,
) {
    let uart = uart_writer.as_mut().unwrap();
    while let Some(data) = uart_tx_consumer.peek() {
        if uart.0.write(*data).is_ok() {
            uart_tx_consumer.dequeue().unwrap();
        } else {
            break;
        }
    }
}

/// Moves queued UART RX data to the USB serial.
/// Returns true if at least one byte was dequeued, i.e. the RX queue has
/// room again and a stopped UART RX interrupt may be re-enabled.
pub fn drain_uart_rx_queue<const N: usize>(
    usb_serial: &mut SerialPort<UsbBus>,
    uart_rx_consumer: &mut Consumer<u8, N>,
) -> bool {
    let mut dequeued = false;
    while let Some(data) = uart_rx_consumer.peek() {
        match write_usb_serial_byte_cs(usb_serial, *data) {
            Ok(_) => {
                uart_rx_consumer.dequeue().unwrap();
                dequeued = true;
            }
            _ => break,
        }
    }
    usb_serial.flush().ok();
    dequeued
}

/// UART RX interrupt body: drains the RX FIFO into the queue, calling
/// `on_byte` per received byte. When the queue is full the RX interrupt is
/// disabled to avoid an interrupt storm; the queue drain sites re-enable it
/// via [`Reader::enable_rx_interrupt`] once there is room again.
pub fn on_uart_rx_irq<D: UartDevice, P: ValidUartPinout<D>, const N: usize>(
    uart_reader: &mut Option<UartReader<D, P>>,
    uart_rx_producer: &mut Producer<u8, N>,
    mut on_byte: impl FnMut(),
) {
    let uart = uart_reader.as_mut().unwrap();
    loop {
        if !uart_rx_producer.ready() {
            uart.0.disable_rx_interrupt();
            break;
        }
        if let Ok(data) = uart.0.read() {
            on_byte();
            let _ = uart_rx_producer.enqueue(data).ok(); // Cannot fail: readiness was checked above.
        } else {
            break;
        }
    }
}

/// Abstracts `split()`, which rp2040-hal implements only on the concrete
/// UART0/UART1 peripherals.
pub trait SplitUart: UartDevice + Sized {
    fn split<P: ValidUartPinout<Self>>(
        uart: UartPeripheral<Enabled, Self, P>,
    ) -> (Reader<Self, P>, Writer<Self, P>);
}

impl SplitUart for UART0 {
    fn split<P: ValidUartPinout<Self>>(
        uart: UartPeripheral<Enabled, Self, P>,
    ) -> (Reader<Self, P>, Writer<Self, P>) {
        uart.split()
    }
}

impl SplitUart for UART1 {
    fn split<P: ValidUartPinout<Self>>(
        uart: UartPeripheral<Enabled, Self, P>,
    ) -> (Reader<Self, P>, Writer<Self, P>) {
        uart.split()
    }
}

/// Re-configures the UART for a new CDC line coding.
pub fn reconfigure_uart<D: UartDevice + SplitUart, P: ValidUartPinout<D>>(
    uart_reader: &mut Option<UartReader<D, P>>,
    uart_writer: &mut Option<UartWriter<D, P>>,
    uart_config: &mut UartConfigAndClock,
    expected_config: &UartConfig,
) {
    let reader = uart_reader.take().unwrap().0;
    let writer = uart_writer.take().unwrap().0;
    let disabled = UartPeripheral::join(reader, writer).disable();
    let enabled = disabled
        .enable(expected_config.into(), uart_config.clock)
        .unwrap();
    uart_config.config = *expected_config;
    let (mut new_reader, new_writer) = D::split(enabled);
    new_reader.enable_rx_interrupt();
    uart_reader.replace(UartReader(new_reader));
    uart_writer.replace(UartWriter(new_writer));
}
