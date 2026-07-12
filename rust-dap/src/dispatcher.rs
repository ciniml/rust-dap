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

//! Concrete CMSIS-DAP command parser and protocol policy.
//!
//! All request/response byte handling happens here so that transports only
//! deal with parsed values. Retry policy, posted reads and match value/mask
//! handling are also implemented here, once, for every transport.

use crate::transport::{ActivePort, ConnectPort, DapConfig, DapTransport, MAX_JTAG_DEVICES};
use crate::cmsis_dap::{
    read_swd_request, read_u16, read_u32, write_u32, DapCommandId, DapError, DapInfoId,
    JtagSequenceInfo, SwdRequest, SwjPins, DAP_ERROR, DAP_OK, DAP_TRANSFER_ERROR, DAP_TRANSFER_OK,
    DAP_TRANSFER_WAIT, SWD_SEQUENCE_CLOCK, SWD_SEQUENCE_DIN,
};
use crate::cursor::BufferCursor;
use core::convert::TryInto;
use num_enum::TryFromPrimitive;

type CommandResult = Result<(usize, usize), DapError>;

pub struct Dispatcher {
    config: DapConfig,
    active: Option<ActivePort>,
}

impl Dispatcher {
    pub fn new(config: DapConfig) -> Self {
        Self {
            config,
            active: None,
        }
    }

    pub fn config(&self) -> &DapConfig {
        &self.config
    }

    pub fn active_port(&self) -> Option<ActivePort> {
        self.active
    }

    /// Execute a single command. `request` is the packet payload after the
    /// command byte; may contain further commands, so the returned tuple is
    /// (request bytes consumed, response bytes produced).
    pub fn execute<T: DapTransport>(
        &mut self,
        transport: &mut T,
        max_packet_size: u16,
        command: DapCommandId,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        match command {
            DapCommandId::Info => self.dap_info(transport, max_packet_size, request, response),
            DapCommandId::HostStatus => self.host_status(request, response),
            DapCommandId::Connect => self.connect(transport, request, response),
            DapCommandId::Disconnect => self.disconnect(transport, request, response),
            DapCommandId::TransferConfigure => self.transfer_configure(request, response),
            DapCommandId::Transfer => self.transfer(transport, request, response),
            DapCommandId::TransferBlock => self.transfer_block(transport, request, response),
            DapCommandId::TransferAbort
            | DapCommandId::WriteAbort
            | DapCommandId::Delay
            | DapCommandId::ResetTarget => Err(DapError::InvalidCommand),
            DapCommandId::SWJPins => self.swj_pins(transport, request, response),
            DapCommandId::SWJClock => self.swj_clock(transport, request, response),
            DapCommandId::SWJSequence => self.swj_sequence(transport, request, response),
            DapCommandId::SWDConfigure => self.swd_configure(request, response),
            DapCommandId::SWDSequence => self.swd_sequence(transport, request, response),
            DapCommandId::JTAGSequence => self.jtag_sequence(transport, request, response),
            DapCommandId::JTAGConfigure => self.jtag_configure(request, response),
            DapCommandId::JTAGIdcode => self.jtag_idcode(transport, request, response),
        }
    }

    fn dap_info<T: DapTransport>(
        &mut self,
        transport: &mut T,
        max_packet_size: u16,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let id = DapInfoId::try_from_primitive(request[0])?;
        let identity = &self.config.identity;
        let length: usize = {
            let buffer = &mut response[1..];
            let write_str = |buffer: &mut [u8], s: &str| -> Result<usize, DapError> {
                let bytes = s.as_bytes();
                if buffer.len() < bytes.len() {
                    return Err(DapError::InternalError);
                }
                buffer[..bytes.len()].copy_from_slice(bytes);
                Ok(bytes.len())
            };
            match id {
                DapInfoId::Vendor => write_str(buffer, identity.vendor)?,
                DapInfoId::Product => write_str(buffer, identity.product)?,
                DapInfoId::SerialNumber => write_str(buffer, identity.serial_number)?,
                DapInfoId::CmsisDapVer => write_str(buffer, identity.firmware_version)?,
                DapInfoId::Capabilities => {
                    let capabilities = transport.capabilities();
                    let bits = capabilities.bits().to_le_bytes();
                    buffer[0] = bits[0];
                    if capabilities.bits() > 0xff {
                        buffer[1] = bits[1];
                        2
                    } else {
                        1
                    }
                }
                DapInfoId::PacketCount => {
                    buffer[0] = identity.packet_count;
                    1
                }
                DapInfoId::PacketSize => {
                    buffer[0..2].copy_from_slice(&max_packet_size.to_le_bytes());
                    2
                }
                _ => 0,
            }
        };
        response[0] = length as u8;
        Ok((1, length + 1))
    }

    fn host_status(&mut self, request: &[u8], response: &mut [u8]) -> CommandResult {
        // Request: type (1 byte), status (1 byte). LEDs are not controlled here;
        // boards may observe them via a future hook if needed.
        if request.len() < 2 {
            return Err(DapError::InvalidCommand);
        }
        response[0] = DAP_OK;
        Ok((2, 1))
    }

    fn connect<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let port = match request[0] {
            0 => ConnectPort::Default,
            1 => ConnectPort::Swd,
            2 => ConnectPort::Jtag,
            _ => return Err(DapError::InvalidCommand),
        };
        // Per spec the response is 0 when the connection failed.
        response[0] = match transport.connect(port, &self.config) {
            Ok(active) => {
                self.active = Some(active);
                match active {
                    ActivePort::Swd => 1,
                    ActivePort::Jtag => 2,
                }
            }
            Err(_) => 0,
        };
        Ok((1, 1))
    }

    fn disconnect<T: DapTransport>(
        &mut self,
        transport: &mut T,
        _request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        self.active = None;
        response[0] = match transport.disconnect(&self.config) {
            Ok(()) => DAP_OK,
            Err(_) => DAP_ERROR,
        };
        Ok((0, 1))
    }

    fn transfer_configure(&mut self, request: &[u8], response: &mut [u8]) -> CommandResult {
        if request.len() < 5 {
            return Err(DapError::InvalidCommand);
        }
        self.config.swd.idle_cycles = request[0] as u32;
        self.config.jtag.idle_cycles = request[0] as u32;
        self.config.retry_count = u16::from_le_bytes(request[1..3].try_into().unwrap()) as u32;
        self.config.match_retry_count =
            u16::from_le_bytes(request[3..5].try_into().unwrap()) as u32;
        response[0] = DAP_OK;
        Ok((5, 1))
    }

    /// Single transfer on the active port. No retry.
    fn transfer_once<T: DapTransport>(
        &mut self,
        transport: &mut T,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        match self.active {
            Some(ActivePort::Swd) => transport.swd_transfer(&self.config, request, data),
            Some(ActivePort::Jtag) => {
                transport.jtag_transfer(&self.config, dap_index, request, data)
            }
            None => Err(DapError::NotSupported),
        }
    }

    /// Transfer with the WAIT-retry policy applied.
    fn transfer_with_retry<T: DapTransport>(
        &mut self,
        transport: &mut T,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        let mut retry_count = 0;
        loop {
            match self.transfer_once(transport, dap_index, request, data) {
                Ok(value) => break Ok(value),
                Err(DapError::SwdError(err)) => {
                    if err != DAP_TRANSFER_WAIT || retry_count == self.config.retry_count {
                        break Err(DapError::SwdError(err));
                    }
                    retry_count += 1;
                }
                Err(err) => break Err(err),
            }
        }
    }

    fn transfer<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.len() < 2 {
            return Err(DapError::InvalidCommand);
        }

        let dap_index = request[0];
        let mut request_count = request[1];
        let mut request = BufferCursor::new_with_position(request, 2);
        let (response_header, response_body) = response.split_at_mut(2);
        let mut response = BufferCursor::new(response_body);
        let mut posted_read = false;
        let mut write_issued = false;
        let mut response_count = 0;
        let mut last_response = loop {
            if request_count == 0 {
                break Ok(0);
            }
            request_count -= 1;

            let swd_request = read_swd_request(&mut request);

            if swd_request.contains(SwdRequest::RnW) {
                if posted_read {
                    // Read-out the previously posted read result.
                    let result = if swd_request.contains(SwdRequest::APnDP)
                        && !swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE)
                    {
                        self.transfer_with_retry(transport, dap_index, swd_request, 0)
                    } else {
                        posted_read = false;
                        self.transfer_with_retry(
                            transport,
                            dap_index,
                            SwdRequest::RDBUFF | SwdRequest::RnW,
                            0,
                        )
                    };
                    if let Ok(value) = result {
                        write_u32(&mut response, value);
                    } else {
                        break result;
                    }
                }
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE) {
                    let match_value = read_u32(&mut request);
                    let mut match_retry_count = 0;

                    if swd_request.contains(SwdRequest::APnDP) {
                        // Issue AP read.
                        if let Err(err) =
                            self.transfer_with_retry(transport, dap_index, swd_request, 0)
                        {
                            break Err(err);
                        }
                    }
                    let result = loop {
                        match self.transfer_with_retry(transport, dap_index, swd_request, 0) {
                            Ok(value) => {
                                if value & self.config.match_mask == match_value {
                                    break Ok(value);
                                } else if match_retry_count == self.config.match_retry_count {
                                    break Err(DapError::ExceedRetryCount);
                                }
                                match_retry_count += 1;
                            }
                            Err(DapError::SwdError(err)) => {
                                if err != DAP_TRANSFER_WAIT
                                    || match_retry_count == self.config.match_retry_count
                                {
                                    break Err(DapError::SwdError(err));
                                }
                                match_retry_count += 1;
                            }
                            Err(err) => break Err(err),
                        }
                    };
                    if result.is_err() {
                        break result;
                    }
                } else if swd_request.contains(SwdRequest::APnDP) {
                    // AP read: post the read.
                    if !posted_read {
                        match self.transfer_with_retry(transport, dap_index, swd_request, 0) {
                            Ok(_) => posted_read = true,
                            Err(err) => break Err(err),
                        }
                    }
                } else {
                    // DP read.
                    match self.transfer_with_retry(transport, dap_index, swd_request, 0) {
                        Ok(value) => write_u32(&mut response, value),
                        Err(err) => break Err(err),
                    }
                }
                write_issued = false;
            } else {
                // Write register.
                if posted_read {
                    match self.transfer_with_retry(
                        transport,
                        dap_index,
                        SwdRequest::RDBUFF | SwdRequest::RnW,
                        0,
                    ) {
                        Ok(value) => {
                            write_u32(&mut response, value);
                            posted_read = false;
                        }
                        Err(err) => break Err(err),
                    }
                }

                let value = read_u32(&mut request);
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_MASK) {
                    self.config.match_mask = value;
                } else {
                    match self.transfer_with_retry(transport, dap_index, swd_request, value) {
                        Ok(_) => {}
                        Err(err) => break Err(err),
                    }
                    write_issued = true;
                }
            }

            response_count += 1;
        };

        // Skip remaining requests after an error.
        while request_count > 0 {
            request_count -= 1;
            let swd_request = read_swd_request(&mut request);
            if swd_request.contains(SwdRequest::RnW) {
                if swd_request.contains(SwdRequest::TRANSFER_MATCH_VALUE) {
                    read_u32(&mut request);
                }
            } else {
                read_u32(&mut request);
            }
        }

        if last_response.is_ok() && (posted_read || write_issued) {
            match self.transfer_with_retry(
                transport,
                dap_index,
                SwdRequest::RDBUFF | SwdRequest::RnW,
                0,
            ) {
                Ok(value) => {
                    if posted_read {
                        write_u32(&mut response, value);
                    }
                }
                Err(err) => {
                    last_response = Err(err);
                }
            }
        }

        response_header[0] = response_count as u8;
        response_header[1] = last_response.map_or_else(
            |e| {
                if let DapError::SwdError(err) = e {
                    err
                } else {
                    DAP_TRANSFER_ERROR
                }
            },
            |_| DAP_TRANSFER_OK,
        );
        Ok((request.get_position(), response.get_position() + 2))
    }

    fn transfer_block<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.len() < 3 {
            return Err(DapError::InvalidCommand);
        }
        let dap_index = request[0];
        let (response_header, response_body) = response.split_at_mut(3);
        let mut response = BufferCursor::new(response_body);
        let mut request = BufferCursor::new_with_position(request, 1);
        let request_count = read_u16(&mut request) as u32;
        if request_count == 0 {
            response_header[0] = 0;
            response_header[1] = 0;
            response_header[2] = 0;
            return Ok((3, 3));
        }

        let swd_request = read_swd_request(&mut request);
        let mut response_count = 0u32;
        let result = self.transfer_block_inner(
            transport,
            dap_index,
            swd_request,
            &mut request,
            &mut response,
            request_count,
            &mut response_count,
        );
        response_header[0] = (response_count & 0xff) as u8;
        response_header[1] = (response_count >> 8 & 0xff) as u8;
        response_header[2] = match result {
            Ok(_) => DAP_TRANSFER_OK,
            Err(err) => match err {
                DapError::ExceedRetryCount => DAP_TRANSFER_WAIT,
                DapError::SwdError(swd_error) => swd_error,
                _ => DAP_TRANSFER_ERROR,
            },
        };
        Ok((request.get_position(), response.get_position() + 3))
    }

    #[allow(clippy::too_many_arguments)]
    fn transfer_block_inner<T: DapTransport>(
        &mut self,
        transport: &mut T,
        dap_index: u8,
        swd_request: SwdRequest,
        request: &mut BufferCursor<&[u8]>,
        response: &mut BufferCursor<&mut [u8]>,
        mut request_count: u32,
        response_count: &mut u32,
    ) -> Result<(), DapError> {
        if swd_request.contains(SwdRequest::RnW) {
            // Read access.
            if swd_request.contains(SwdRequest::APnDP) {
                self.transfer_with_retry(transport, dap_index, swd_request, 0)?;
            }
            while request_count > 0 {
                request_count -= 1;
                let swd_request = if request_count == 0 && swd_request.contains(SwdRequest::APnDP) {
                    SwdRequest::RDBUFF | SwdRequest::RnW
                } else {
                    swd_request
                };
                let result = self.transfer_with_retry(transport, dap_index, swd_request, 0)?;
                write_u32(response, result);
                *response_count += 1;
            }
        } else {
            // Write access.
            while request_count > 0 {
                request_count -= 1;
                let data = read_u32(request);
                self.transfer_with_retry(transport, dap_index, swd_request, data)?;
                *response_count += 1;
            }
            // Check the last write result.
            self.transfer_with_retry(
                transport,
                dap_index,
                SwdRequest::RDBUFF | SwdRequest::RnW,
                0,
            )?;
        }
        Ok(())
    }

    fn swj_pins<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.len() < 6 {
            return Err(DapError::InvalidCommand);
        }
        let output = SwjPins::from_bits_truncate(request[0]);
        let select = SwjPins::from_bits_truncate(request[1]);
        let wait_us = u32::from_le_bytes(request[2..6].try_into().unwrap());
        let input = transport.swj_pins(&self.config, output, select, wait_us);
        response[0] = input.map_or(0, |pins| pins.bits());
        Ok((6, 1))
    }

    fn swj_clock<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.len() < 4 {
            return Err(DapError::InvalidCommand);
        }
        let clock_hz = u32::from_le_bytes(request[0..4].try_into().unwrap());
        response[0] = if transport.swj_clock(&mut self.config, clock_hz).is_ok() {
            DAP_OK
        } else {
            DAP_ERROR
        };
        Ok((4, 1))
    }

    fn swj_sequence<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let count = if request[0] == 0 {
            256
        } else {
            request[0] as usize
        };
        let count_bytes = (count + 7) >> 3;
        if request.len() > count_bytes {
            transport.swj_sequence(&self.config, count, &request[1..count_bytes + 1])?;
            response[0] = DAP_OK;
            Ok((count_bytes + 1, 1))
        } else {
            response[0] = DAP_ERROR;
            Ok((request.len(), 1))
        }
    }

    fn swd_configure(&mut self, request: &[u8], response: &mut [u8]) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        self.config.swd.always_generate_data_phase = (request[0] & 0b100) != 0;
        self.config.swd.turn_around_cycles = (request[0] & 3) as u32 + 1;
        response[0] = DAP_OK;
        Ok((1, 1))
    }

    fn swd_sequence<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let mut sequence_count = request[0];
        let mut request_index = 1;
        let mut response_index = 1;
        while sequence_count > 0 {
            sequence_count -= 1;
            if request.len() <= request_index {
                return Err(DapError::InvalidCommand);
            }
            let sequence_info = request[request_index];
            request_index += 1;

            let clock_count = if sequence_info & SWD_SEQUENCE_CLOCK == 0 {
                64
            } else {
                sequence_info & SWD_SEQUENCE_CLOCK
            } as usize;
            let bytes_count = (clock_count + 7) >> 3;
            let do_input = sequence_info & SWD_SEQUENCE_DIN != 0;

            if do_input {
                if response.len() < response_index + bytes_count {
                    return Err(DapError::InvalidCommand);
                }
                transport.swd_output_enable(false)?;
                transport.swd_read_bits(
                    &self.config,
                    clock_count,
                    &mut response[response_index..response_index + bytes_count],
                )?;
                response_index += bytes_count;
            } else {
                if request.len() < request_index + bytes_count {
                    return Err(DapError::InvalidCommand);
                }
                transport.swd_output_enable(true)?;
                transport.swd_write_bits(
                    &self.config,
                    clock_count,
                    &request[request_index..request_index + bytes_count],
                )?;
                request_index += bytes_count;
            }

            if sequence_count == 0 {
                transport.swd_output_enable(true)?;
            }
        }
        response[0] = DAP_OK;
        Ok((request_index, response_index))
    }

    fn jtag_sequence<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }

        let mut request_proceeded = 0;
        let sequence_count = request[0];
        request_proceeded += 1;
        let mut request = &request[1..];

        let mut response_body = &mut response[1..];
        let mut response_body_count = 0;

        for _ in 0..sequence_count {
            if request.is_empty() {
                return Err(DapError::InvalidCommand);
            }
            let info = JtagSequenceInfo::from(request[0]);
            request_proceeded += 1;
            request = &request[1..];

            let tdi_bytes = info.number_of_tck_cycles.div_ceil(8);
            if request.len() < tdi_bytes {
                return Err(DapError::InvalidCommand);
            }
            let mut tdi_data = [0u8; 8];
            tdi_data[..tdi_bytes].copy_from_slice(&request[..tdi_bytes]);
            let tdi_data = u64::from_le_bytes(tdi_data);
            request = &request[tdi_bytes..];
            request_proceeded += tdi_bytes;

            let tdo = transport.jtag_sequence(&self.config, &info, tdi_data)?;
            if info.tdo_capture {
                let tdo = tdo.ok_or(DapError::InternalError)?.to_le_bytes();
                if response_body.len() < tdi_bytes {
                    return Err(DapError::InvalidCommand);
                }
                response_body[..tdi_bytes].copy_from_slice(&tdo[..tdi_bytes]);
                response_body_count += tdi_bytes;
                response_body = &mut response_body[tdi_bytes..];
            }
        }

        response[0] = DAP_OK;
        Ok((request_proceeded, response_body_count + 1))
    }

    fn jtag_configure(&mut self, request: &[u8], response: &mut [u8]) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let count = request[0] as usize;
        if count > MAX_JTAG_DEVICES || request.len() < count + 1 {
            return Err(DapError::InvalidCommand);
        }
        self.config.jtag.ir_length[..count].copy_from_slice(&request[1..count + 1]);
        self.config.jtag.device_count = count as u8;
        response[0] = DAP_OK;
        Ok((count + 1, 1))
    }

    fn jtag_idcode<T: DapTransport>(
        &mut self,
        transport: &mut T,
        request: &[u8],
        response: &mut [u8],
    ) -> CommandResult {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }
        let index = request[0];
        match transport.jtag_idcode(&self.config, index) {
            Ok(idcode) => {
                response[0] = DAP_OK;
                response[1..5].copy_from_slice(&idcode.to_le_bytes());
                Ok((1, 5))
            }
            Err(err) => Err(err),
        }
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use crate::cmsis_dap::{DapCapabilities, DAP_TRANSFER_FAULT};
    use std::vec::Vec;

    /// Records every transport call and plays back scripted transfer results.
    struct MockTransport {
        capabilities: DapCapabilities,
        connect_result: Result<ActivePort, DapError>,
        transfer_log: Vec<(u8, SwdRequest, u32)>,
        transfer_results: Vec<Result<u32, DapError>>,
        swj_sequences: Vec<(usize, Vec<u8>)>,
        written_bits: Vec<(usize, Vec<u8>)>,
        read_bit_fill: u8,
        output_enables: Vec<bool>,
        jtag_sequences: Vec<(JtagSequenceInfo, u64)>,
    }

    impl MockTransport {
        fn swd() -> Self {
            Self {
                capabilities: DapCapabilities::SWD,
                connect_result: Ok(ActivePort::Swd),
                transfer_log: Vec::new(),
                transfer_results: Vec::new(),
                swj_sequences: Vec::new(),
                written_bits: Vec::new(),
                read_bit_fill: 0xa5,
                output_enables: Vec::new(),
                jtag_sequences: Vec::new(),
            }
        }
        fn with_results(results: &[Result<u32, DapError>]) -> Self {
            let mut mock = Self::swd();
            // pop() consumes from the back.
            mock.transfer_results = results.iter().rev().map(clone_result).collect();
            mock
        }
    }

    fn clone_result(r: &Result<u32, DapError>) -> Result<u32, DapError> {
        match r {
            Ok(v) => Ok(*v),
            Err(DapError::SwdError(e)) => Err(DapError::SwdError(*e)),
            Err(DapError::ExceedRetryCount) => Err(DapError::ExceedRetryCount),
            Err(_) => Err(DapError::InternalError),
        }
    }

    impl DapTransport for MockTransport {
        fn capabilities(&self) -> DapCapabilities {
            self.capabilities
        }
        fn connect(
            &mut self,
            _port: ConnectPort,
            _config: &DapConfig,
        ) -> Result<ActivePort, DapError> {
            match &self.connect_result {
                Ok(port) => Ok(*port),
                Err(DapError::NotSupported) => Err(DapError::NotSupported),
                Err(_) => Err(DapError::InternalError),
            }
        }
        fn disconnect(&mut self, _config: &DapConfig) -> Result<(), DapError> {
            Ok(())
        }
        fn swj_sequence(
            &mut self,
            _config: &DapConfig,
            count: usize,
            data: &[u8],
        ) -> Result<(), DapError> {
            self.swj_sequences.push((count, data.to_vec()));
            Ok(())
        }
        fn swj_pins(
            &mut self,
            _config: &DapConfig,
            output: SwjPins,
            select: SwjPins,
            _wait_us: u32,
        ) -> Result<SwjPins, DapError> {
            // Echo back what was driven on the selected pins.
            Ok(output & select)
        }
        fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
            config.swd.clock_wait_cycles = frequency_hz; // marker for the test
            Ok(())
        }
        fn swd_transfer(
            &mut self,
            _config: &DapConfig,
            request: SwdRequest,
            data: u32,
        ) -> Result<u32, DapError> {
            self.transfer_log.push((0, request, data));
            self.transfer_results.pop().unwrap_or(Ok(0x12345678))
        }
        fn swd_read_bits(
            &mut self,
            _config: &DapConfig,
            count: usize,
            data: &mut [u8],
        ) -> Result<(), DapError> {
            for byte in data.iter_mut() {
                *byte = self.read_bit_fill;
            }
            let _ = count;
            Ok(())
        }
        fn swd_write_bits(
            &mut self,
            _config: &DapConfig,
            count: usize,
            data: &[u8],
        ) -> Result<(), DapError> {
            self.written_bits.push((count, data.to_vec()));
            Ok(())
        }
        fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> {
            self.output_enables.push(enable);
            Ok(())
        }
        fn jtag_sequence(
            &mut self,
            _config: &DapConfig,
            info: &JtagSequenceInfo,
            tdi_data: u64,
        ) -> Result<Option<u64>, DapError> {
            let capture = info.tdo_capture;
            self.jtag_sequences.push(
                (JtagSequenceInfo {
                    number_of_tck_cycles: info.number_of_tck_cycles,
                    tms_value: info.tms_value,
                    tdo_capture: info.tdo_capture,
                }, tdi_data),
            );
            Ok(if capture { Some(0xdead_beef_cafe_f00d) } else { None })
        }
    }

    fn dispatcher() -> Dispatcher {
        Dispatcher::new(DapConfig::default())
    }

    fn connected_dispatcher(t: &mut MockTransport) -> Dispatcher {
        let mut d = dispatcher();
        let mut resp = [0u8; 8];
        d.execute(t, 64, DapCommandId::Connect, &[1], &mut resp)
            .unwrap();
        assert_eq!(resp[0], 1);
        d
    }

    #[test]
    fn info_reports_identity_and_packet_size() {
        let mut t = MockTransport::swd();
        let mut d = Dispatcher::new(DapConfig {
            identity: crate::transport::DapIdentity {
                vendor: "TestVendor",
                ..Default::default()
            },
            ..Default::default()
        });
        let mut resp = [0u8; 64];
        // Vendor string
        let (req, len) = d
            .execute(&mut t, 64, DapCommandId::Info, &[0x01], &mut resp)
            .unwrap();
        assert_eq!((req, len), (1, 1 + 10));
        assert_eq!(&resp[1..11], b"TestVendor");
        // PacketSize reflects MAX_PACKET_SIZE
        let (_, len) = d
            .execute(&mut t, 512, DapCommandId::Info, &[0xff], &mut resp)
            .unwrap();
        assert_eq!(len, 3);
        assert_eq!(u16::from_le_bytes([resp[1], resp[2]]), 512);
        // Capabilities from the transport
        d.execute(&mut t, 64, DapCommandId::Info, &[0xf0], &mut resp)
            .unwrap();
        assert_eq!(resp[1], DapCapabilities::SWD.bits() as u8);
    }

    #[test]
    fn connect_selects_port_and_reports_failure() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 8];
        // Default port -> SWD
        d.execute(&mut t, 64, DapCommandId::Connect, &[0], &mut resp)
            .unwrap();
        assert_eq!(resp[0], 1);
        assert_eq!(d.active_port(), Some(ActivePort::Swd));
        // JTAG not supported -> response 0, port unchanged... (stays on previous)
        let mut t2 = MockTransport::swd();
        t2.connect_result = Err(DapError::NotSupported);
        let mut d2 = dispatcher();
        d2.execute(&mut t2, 64, DapCommandId::Connect, &[2], &mut resp)
            .unwrap();
        assert_eq!(resp[0], 0);
        assert_eq!(d2.active_port(), None);
    }

    #[test]
    fn transfer_without_connect_reports_error() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 64];
        // One DP read without DAP_Connect.
        let (_, len) = d
            .execute(&mut t, 64, DapCommandId::Transfer, &[0, 1, 0x02], &mut resp)
            .unwrap();
        assert_eq!(len, 2);
        assert_eq!(resp[0], 0); // no transfers executed
        assert_eq!(resp[1], DAP_TRANSFER_ERROR);
    }

    #[test]
    fn transfer_dp_read_and_write() {
        let mut t = MockTransport::with_results(&[Ok(0xcafebabe), Ok(0)]);
        let mut d = connected_dispatcher(&mut t);
        // DP read (RnW), then DP write with value 0x11223344.
        let request = [
            0u8, 2, // dap_index, count
            0x02, // DP read
            0x00, 0x44, 0x33, 0x22, 0x11, // DP write
        ];
        let mut resp = [0u8; 64];
        let (consumed, produced) = d
            .execute(&mut t, 64, DapCommandId::Transfer, &request, &mut resp)
            .unwrap();
        assert_eq!(consumed, request.len());
        assert_eq!(resp[0], 2); // 2 transfers
        assert_eq!(resp[1], DAP_TRANSFER_OK);
        assert_eq!(u32::from_le_bytes(resp[2..6].try_into().unwrap()), 0xcafebabe);
        // write + trailing RDBUFF check
        assert_eq!(produced, 2 + 4);
        assert_eq!(t.transfer_log.len(), 3);
        assert_eq!(t.transfer_log[1].2, 0x11223344);
        assert!(t.transfer_log[2].1.contains(SwdRequest::RDBUFF));
    }

    #[test]
    fn transfer_posted_ap_read() {
        // AP read is posted: first read issues, second returns previous value,
        // final RDBUFF picks up the last one.
        let mut t = MockTransport::with_results(&[Ok(0), Ok(0xaaaa0001), Ok(0xaaaa0002)]);
        let mut d = connected_dispatcher(&mut t);
        let request = [0u8, 2, 0x03, 0x03]; // two AP reads
        let mut resp = [0u8; 64];
        let (_, produced) = d
            .execute(&mut t, 64, DapCommandId::Transfer, &request, &mut resp)
            .unwrap();
        assert_eq!(resp[0], 2);
        assert_eq!(resp[1], DAP_TRANSFER_OK);
        assert_eq!(produced, 2 + 8);
        assert_eq!(u32::from_le_bytes(resp[2..6].try_into().unwrap()), 0xaaaa0001);
        assert_eq!(u32::from_le_bytes(resp[6..10].try_into().unwrap()), 0xaaaa0002);
        assert_eq!(t.transfer_log.len(), 3);
    }

    #[test]
    fn transfer_retries_on_wait() {
        let mut t = MockTransport::with_results(&[
            Err(DapError::SwdError(DAP_TRANSFER_WAIT)),
            Err(DapError::SwdError(DAP_TRANSFER_WAIT)),
            Ok(0x5555aaaa),
        ]);
        let mut d = connected_dispatcher(&mut t);
        let request = [0u8, 1, 0x02];
        let mut resp = [0u8; 64];
        d.execute(&mut t, 64, DapCommandId::Transfer, &request, &mut resp)
            .unwrap();
        assert_eq!(resp[1], DAP_TRANSFER_OK);
        assert_eq!(t.transfer_log.len(), 3); // 2 retries + success
    }

    #[test]
    fn transfer_stops_on_fault() {
        let mut t = MockTransport::with_results(&[Err(DapError::SwdError(DAP_TRANSFER_FAULT))]);
        let mut d = connected_dispatcher(&mut t);
        // Two DP reads; first faults, second must be skipped.
        let request = [0u8, 2, 0x02, 0x02];
        let mut resp = [0u8; 64];
        let (consumed, _) = d
            .execute(&mut t, 64, DapCommandId::Transfer, &request, &mut resp)
            .unwrap();
        assert_eq!(consumed, request.len()); // remaining requests skipped but consumed
        assert_eq!(resp[0], 0);
        assert_eq!(resp[1], DAP_TRANSFER_FAULT);
        assert_eq!(t.transfer_log.len(), 1);
    }

    #[test]
    fn transfer_block_write_and_read() {
        let mut t = MockTransport::swd();
        let mut d = connected_dispatcher(&mut t);
        // Write 2 words to DP.
        let request = [
            0u8, 2, 0, // dap_index, count=2 (LE)
            0x00, // DP write
            0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55,
        ];
        let mut resp = [0u8; 64];
        let (consumed, produced) = d
            .execute(&mut t, 64, DapCommandId::TransferBlock, &request, &mut resp)
            .unwrap();
        assert_eq!(consumed, request.len());
        assert_eq!(produced, 3);
        assert_eq!(u16::from_le_bytes([resp[0], resp[1]]), 2);
        assert_eq!(resp[2], DAP_TRANSFER_OK);
        assert_eq!(t.transfer_log.len(), 3); // 2 writes + RDBUFF

        // Read 2 words from DP.
        let request = [0u8, 2, 0, 0x02];
        let (_, produced) = d
            .execute(&mut t, 64, DapCommandId::TransferBlock, &request, &mut resp)
            .unwrap();
        assert_eq!(produced, 3 + 8);
    }

    #[test]
    fn swj_commands() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 8];
        // SWJ_Sequence: 16 bits
        let (consumed, _) = d
            .execute(
                &mut t,
                64,
                DapCommandId::SWJSequence,
                &[16, 0xaa, 0x55, 0x99],
                &mut resp,
            )
            .unwrap();
        assert_eq!(consumed, 3);
        assert_eq!(resp[0], DAP_OK);
        assert_eq!(t.swj_sequences[0], (16, vec![0xaa, 0x55]));
        // SWJ_Clock with trailing bytes must not panic and consume 4.
        let (consumed, _) = d
            .execute(
                &mut t,
                64,
                DapCommandId::SWJClock,
                &[0x40, 0x42, 0x0f, 0x00, 0xde],
                &mut resp,
            )
            .unwrap();
        assert_eq!(consumed, 4);
        assert_eq!(d.config().swd.clock_wait_cycles, 1_000_000); // marker
        // SWJ_Pins echo.
        let (consumed, _) = d
            .execute(
                &mut t,
                64,
                DapCommandId::SWJPins,
                &[0x80, 0x80, 0, 0, 0, 0],
                &mut resp,
            )
            .unwrap();
        assert_eq!(consumed, 6);
        assert_eq!(resp[0], 0x80);
    }

    #[test]
    fn swd_sequence_mixed_input_output() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 64];
        // 2 sequences: output 8 bits (0x3c), input 8 bits.
        let request = [2u8, 0x08, 0x3c, 0x88];
        let (consumed, produced) = d
            .execute(&mut t, 64, DapCommandId::SWDSequence, &request, &mut resp)
            .unwrap();
        assert_eq!(consumed, 4);
        assert_eq!(produced, 2);
        assert_eq!(resp[0], DAP_OK);
        assert_eq!(resp[1], 0xa5); // read_bit_fill
        assert_eq!(t.written_bits[0], (8, vec![0x3c]));
        // enable(write), disable(read), final enable
        assert_eq!(t.output_enables, vec![true, false, true]);
    }

    #[test]
    fn jtag_configure_and_sequence() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 64];
        // Configure 2 devices with IR lengths 4 and 5; consumed must be 3.
        let (consumed, _) = d
            .execute(&mut t, 64, DapCommandId::JTAGConfigure, &[2, 4, 5], &mut resp)
            .unwrap();
        assert_eq!(consumed, 3);
        assert_eq!(d.config().jtag.device_count, 2);
        assert_eq!(&d.config().jtag.ir_length[..2], &[4, 5]);
        // Too many devices is rejected.
        let too_many = [MAX_JTAG_DEVICES as u8 + 1; 10];
        assert!(d
            .execute(&mut t, 64, DapCommandId::JTAGConfigure, &too_many, &mut resp)
            .is_err());
        // JTAG sequence with TDO capture: 8 cycles.
        let request = [1u8, 0x88, 0x5a];
        let (consumed, produced) = d
            .execute(&mut t, 64, DapCommandId::JTAGSequence, &request, &mut resp)
            .unwrap();
        assert_eq!(consumed, 3);
        assert_eq!(produced, 2);
        assert_eq!(resp[0], DAP_OK);
        assert_eq!(resp[1], 0x0d); // LSB of 0xdead_beef_cafe_f00d
        assert_eq!(t.jtag_sequences[0].1, 0x5a);
    }

    #[test]
    fn short_requests_are_rejected() {
        let mut t = MockTransport::swd();
        let mut d = dispatcher();
        let mut resp = [0u8; 8];
        for (cmd, req) in [
            (DapCommandId::Info, &[][..]),
            (DapCommandId::Connect, &[][..]),
            (DapCommandId::Transfer, &[0][..]),
            (DapCommandId::TransferBlock, &[0, 1][..]),
            (DapCommandId::SWJPins, &[0, 1][..]),
            (DapCommandId::SWJClock, &[0, 1][..]),
            (DapCommandId::SWDSequence, &[2, 0x08][..]),
            (DapCommandId::JTAGSequence, &[1, 0x88][..]),
            (DapCommandId::JTAGConfigure, &[2, 4][..]),
        ] {
            assert!(
                d.execute(&mut t, 64, cmd, req, &mut resp).is_err(),
                "command {:?} should reject short request",
                cmd
            );
        }
    }
}
