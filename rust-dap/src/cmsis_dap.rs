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

#![allow(non_upper_case_globals)]

use crate::cursor::{CursorError, CursorRead, CursorWrite};
use bitflags::bitflags;
use bitvec::{
    self,
    prelude::{BitArray, Lsb0},
    BitArr,
};
use num_enum::{IntoPrimitive, TryFromPrimitive, TryFromPrimitiveError};
use usb_device::UsbError;

#[derive(IntoPrimitive, TryFromPrimitive, Clone, Copy, Debug)]
#[repr(u8)]
pub enum DapCommandId {
    Info = 0x00,
    HostStatus = 0x01,
    Connect = 0x02,
    Disconnect = 0x03,
    TransferConfigure = 0x04,
    Transfer = 0x05,
    TransferBlock = 0x06,
    TransferAbort = 0x07,
    WriteAbort = 0x08,
    Delay = 0x09,
    ResetTarget = 0x0a,
    SWJPins = 0x10,
    SWJClock = 0x11,
    SWJSequence = 0x12,
    SWDConfigure = 0x13,
    SWDSequence = 0x1d,
    JTAGSequence = 0x14,
    JTAGConfigure = 0x15,
    JTAGIdcode = 0x16,
}

#[derive(IntoPrimitive, TryFromPrimitive)]
#[repr(u8)]
pub(crate) enum DapInfoId {
    Vendor = 1,
    Product = 2,
    SerialNumber = 3,
    CmsisDapVer = 4,
    DeviceVendor = 5,
    DeviceName = 6,
    Capabilities = 0xf0,
    TimeStampClock = 0xf1,
    SwoBufferSize = 0xf2,
    PacketCount = 0xfe,
    PacketSize = 0xff,
}

bitflags! {
    pub struct DapCapabilities:u16 {
        const SWD = 0b0000_0001;
        const JTAG = 0b0000_0010;
        const SWO_UART = 0b0000_0100;
        const SWO_MANCHESTER = 0b0000_1000;
        const ATOMIC_COMMANDS = 0b0001_0000;
        const TEST_DOMAIN_TIMER = 0b0010_0000;
        const SWO_STREAMING_TRACE = 0b0100_0000;
        const UART_COMMUNICATION_PORT = 0b1000_0000;
        const USB_COM_PORT = 0b1_0000_0000;
    }
}

bitflags! {
    pub struct SwdRequest: u8 {
        const APnDP = 0b00000001;
        const RnW = 0b00000010;
        const A2 = 0b00000100;
        const A3 = 0b00001000;
        const RESEND = Self::A3.bits;
        const RDBUFF = Self::A2.bits | Self::A3.bits;
        const TRANSFER_MATCH_VALUE = 0b00010000;
        const TRANSFER_MATCH_MASK = 0b00100000;
        const TRANSFER_TIMESTAMP = 0b10000000;
    }
}

// Bit 0: SWCLK/TCK
// Bit 1: SWDIO/TMS
// Bit 2: TDI
// Bit 3: TDO
// Bit 5: nTRST
// Bit 7: nRESET
// https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__SWJ__Pins.html
bitflags! {
    pub struct SwjPins: u8 {
        const TCK_SWDCLK = 1 << 0;
        const TMS_SWDIO  = 1 << 1;
        const TDI        = 1 << 2;
        const TDO        = 1 << 3;
        const UNKNOWN4   = 1 << 4;
        const N_TRST     = 1 << 5;
        const UNKNOWN6   = 1 << 6;
        const N_RESET    = 1 << 7;
    }
}

pub const DAP_TRANSFER_OK: u8 = 0x01;
pub const DAP_TRANSFER_WAIT: u8 = 0x02;
pub const DAP_TRANSFER_FAULT: u8 = 0x04;
pub const DAP_TRANSFER_ERROR: u8 = 0x08;
pub const DAP_TRANSFER_MISMATCH: u8 = 0x10;

#[allow(dead_code)]
pub enum JtagInstruction {
    ABORT = 0b1000,
    DPACC = 0b1010,
    APACC = 0b1011,
    IDCODE = 0b1110,
    BYPASS = 0b1111,
}

pub fn build_acc(data: u32, a3: bool, a2: bool, read: bool) -> BitArray<[u32; 2], Lsb0> {
    let mut dr: BitArr!(for 35, in u32, Lsb0) = BitArray::new([data, 0]);
    dr.shift_left(3);
    *dr.get_mut(0).unwrap() = read;
    *dr.get_mut(1).unwrap() = a2;
    *dr.get_mut(2).unwrap() = a3;
    dr
}

pub const DAP_OK: u8 = 0x00;
pub const DAP_ERROR: u8 = 0xff;
pub const SWD_SEQUENCE_CLOCK: u8 = 0x3f;
pub const SWD_SEQUENCE_DIN: u8 = 0x80;

#[derive(Debug, PartialEq)]
pub enum DapError {
    InvalidCommand,
    InvalidDapInfoId,
    SwdError(u8),
    InternalError,
    ExceedRetryCount,
    /// The requested operation is not supported by this transport.
    NotSupported,
}

impl From<TryFromPrimitiveError<DapCommandId>> for DapError {
    fn from(_: TryFromPrimitiveError<DapCommandId>) -> Self {
        Self::InvalidCommand
    }
}
impl From<TryFromPrimitiveError<DapInfoId>> for DapError {
    fn from(_: TryFromPrimitiveError<DapInfoId>) -> Self {
        Self::InvalidDapInfoId
    }
}
impl From<CursorError> for DapError {
    fn from(_: CursorError) -> Self {
        Self::InternalError
    }
}
impl From<UsbError> for DapError {
    fn from(_: UsbError) -> Self {
        Self::InternalError
    }
}

#[derive(Debug)]
pub struct JtagSequenceInfo {
    pub number_of_tck_cycles: usize,
    pub tms_value: bool,
    pub tdo_capture: bool,
}

impl From<u8> for JtagSequenceInfo {
    fn from(sequence_info: u8) -> Self {
        JtagSequenceInfo {
            tdo_capture: (sequence_info & (1 << 7)) != 0,
            tms_value: (sequence_info & (1 << 6)) != 0,
            number_of_tck_cycles: if sequence_info & 0b0011_1111 == 0 {
                64
            } else {
                (sequence_info & 0x3f) as usize
            },
        }
    }
}

pub(crate) fn read_swd_request<C: CursorRead>(cursor: &mut C) -> SwdRequest {
    let mut buffer = [0u8; 1];
    cursor.read(&mut buffer).ok();
    unsafe { SwdRequest::from_bits_unchecked(buffer[0]) }
}
pub(crate) fn read_u16<C: CursorRead>(cursor: &mut C) -> u16 {
    let mut value = [0u8; 2];
    cursor.read(&mut value).ok();
    u16::from_le_bytes(value)
}

pub(crate) fn read_u32<C: CursorRead>(cursor: &mut C) -> u32 {
    let mut value = [0u8; 4];
    cursor.read(&mut value).ok();
    u32::from_le_bytes(value)
}
pub(crate) fn write_u32<C: CursorWrite>(cursor: &mut C, value: u32) {
    let bytes = u32::to_le_bytes(value);
    cursor.write(&bytes).ok();
}
