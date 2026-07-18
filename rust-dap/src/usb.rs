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

//! USB class wrapper. Handles packet framing and delegates command
//! execution to [`Dispatcher`].

use crate::cmsis_dap::{DapCommandId, DapError};
use crate::dispatcher::Dispatcher;
use crate::interface::CmsisDapInterface;
use crate::transport::{DapConfig, DapTransport};
use num_enum::TryFromPrimitive;
use usb_device::class_prelude::*;
use usb_device::Result;

const DAP_INVALID_COMMAND_RESPONSE: u8 = 0xff;

pub struct CmsisDap<'a, B, T, const MAX_PACKET_SIZE: usize>
where
    B: UsbBus,
    T: DapTransport,
{
    inner: CmsisDapInterface<'a, B>,
    transport: T,
    dispatcher: Dispatcher,
    next_in_packet: [u8; MAX_PACKET_SIZE],
    next_in_packet_size: Option<usize>,
    pending_out_packet: [u8; MAX_PACKET_SIZE],
    pending_out_packet_size: usize,
}

impl<B, T, const MAX_PACKET_SIZE: usize> CmsisDap<'_, B, T, MAX_PACKET_SIZE>
where
    B: UsbBus,
    T: DapTransport,
{
    pub fn new(
        alloc: &UsbBusAllocator<B>,
        transport: T,
        config: DapConfig,
    ) -> CmsisDap<'_, B, T, MAX_PACKET_SIZE> {
        CmsisDap {
            inner: CmsisDapInterface::new(alloc, MAX_PACKET_SIZE as u16),
            transport,
            dispatcher: Dispatcher::new(config),
            next_in_packet: [0; MAX_PACKET_SIZE],
            next_in_packet_size: None,
            pending_out_packet: [0; MAX_PACKET_SIZE],
            pending_out_packet_size: 0,
        }
    }

    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    pub fn process(&mut self) -> core::result::Result<(), DapError> {
        self.process_out_packet().ok();
        if self.pending_out_packet_size == 0 || self.next_in_packet_size.is_some() {
            self.send_next_packet().ok();
            return Ok(());
        }

        let mut response_packet_length = 0;
        let mut bytes_processed = 0;
        while bytes_processed < self.pending_out_packet_size {
            let command_byte = self.pending_out_packet[bytes_processed];
            bytes_processed += 1;

            let mut response = [0; MAX_PACKET_SIZE];
            response[0] = command_byte;
            let response_size;

            if let Ok(command) = DapCommandId::try_from_primitive(command_byte) {
                let request =
                    &self.pending_out_packet[bytes_processed..self.pending_out_packet_size];
                match self.dispatcher.execute(
                    &mut self.transport,
                    MAX_PACKET_SIZE as u16,
                    command,
                    request,
                    &mut response[1..],
                ) {
                    Ok((request_processed, response_generated)) => {
                        bytes_processed += request_processed;
                        if MAX_PACKET_SIZE < response_generated {
                            // Cannot send more than MAX_PACKET_SIZE bytes.
                            response[1] = DAP_INVALID_COMMAND_RESPONSE;
                        }
                        response_size = 1 + response_generated;
                    }
                    Err(_) => {
                        // Command failed to parse; the remainder of the packet
                        // cannot be interpreted reliably, so report the error
                        // and stop processing this packet.
                        response[1] = DAP_INVALID_COMMAND_RESPONSE;
                        response_size = 2;
                        bytes_processed = self.pending_out_packet_size;
                    }
                }
            } else {
                // Unknown command: per CMSIS-DAP, respond with 0xFF. The
                // request length is unknown, so stop processing this packet.
                response[0] = DAP_INVALID_COMMAND_RESPONSE;
                response_size = 1;
                bytes_processed = self.pending_out_packet_size;
            }

            if MAX_PACKET_SIZE <= (response_packet_length + response_size) {
                // Flush when the accumulated responses exceed the packet size.
                self.next_in_packet_size = Some(response_packet_length);
                self.send_next_packet().ok();
                response_packet_length = 0;
            }
            self.next_in_packet[response_packet_length..response_packet_length + response_size]
                .copy_from_slice(&response[..response_size]);
            response_packet_length += response_size;
        }
        self.next_in_packet_size = Some(response_packet_length);
        self.pending_out_packet_size = 0;
        self.send_next_packet().ok();

        Ok(())
    }

    fn send_next_packet(&mut self) -> Result<()> {
        if let Some(size) = self.next_in_packet_size {
            if size > 0 {
                self.inner.write_packet(&self.next_in_packet[0..size])?;
            } else {
                self.inner.write_packet(&[])?;
            }
            self.next_in_packet_size = None;
        }
        Ok(())
    }
    fn process_out_packet(&mut self) -> Result<()> {
        if self.pending_out_packet_size == 0 {
            self.pending_out_packet_size = self
                .inner
                .read_packet(&mut self.pending_out_packet)
                .map_or(0, |size| size);
        }
        Ok(())
    }
}

impl<B, T, const MAX_PACKET_SIZE: usize> UsbClass<B> for CmsisDap<'_, B, T, MAX_PACKET_SIZE>
where
    B: UsbBus,
    T: DapTransport,
{
    fn get_configuration_descriptors(&self, writer: &mut DescriptorWriter) -> Result<()> {
        self.inner.get_configuration_descriptors(writer)
    }
    fn get_bos_descriptors(&self, writer: &mut BosWriter) -> Result<()> {
        self.inner.get_bos_descriptors(writer)
    }
    fn get_string(&self, index: StringIndex, lang_id: LangID) -> Option<&str> {
        self.inner.get_string(index, lang_id)
    }
    fn reset(&mut self) {
        self.inner.reset()
    }
    fn control_in(&mut self, xfer: ControlIn<B>) {
        self.inner.control_in(xfer)
    }
    fn control_out(&mut self, xfer: ControlOut<B>) {
        self.inner.control_out(xfer)
    }

    fn endpoint_in_complete(&mut self, addr: EndpointAddress) {
        if addr == self.inner.in_ep_address() {
            let _ = self.send_next_packet();
        }
    }

    fn endpoint_out(&mut self, addr: EndpointAddress) {
        if addr == self.inner.out_ep_address() {
            let _ = self.process_out_packet();
        }
    }
}
