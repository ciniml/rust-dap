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

use hal::uart;
use rp2040_hal as hal;
use usbd_serial;

use core::convert::{From, TryFrom};

/// UART configuration conversion error
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartConvertError {
    /// Incompatible UART parameter
    Incompatible,
}

/// UART Stop bits
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartStopBits {
    /// 1 stop bit
    One,
    /// 2 stop bits
    Two,
}

/// UART parity type
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartParityType {
    /// No parity
    None,
    /// Odd parity
    Odd,
    /// Even parity
    Even,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UartDataBits(u8);

/// UART configuration which can be converted into both usbd_serial UART configuration and RP2040 UART configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UartConfig {
    pub data_bits: UartDataBits,
    pub stop_bits: UartStopBits,
    pub parity_type: UartParityType,
    pub data_rate: u32,
}

impl From<UartStopBits> for usbd_serial::StopBits {
    fn from(value: UartStopBits) -> Self {
        match value {
            UartStopBits::One => Self::One,
            UartStopBits::Two => Self::Two,
        }
    }
}
impl TryFrom<usbd_serial::StopBits> for UartStopBits {
    type Error = UartConvertError;
    fn try_from(value: usbd_serial::StopBits) -> Result<Self, Self::Error> {
        match value {
            usbd_serial::StopBits::One => Ok(Self::One),
            usbd_serial::StopBits::Two => Ok(Self::Two),
            _ => Err(Self::Error::Incompatible),
        }
    }
}

impl From<UartStopBits> for uart::StopBits {
    fn from(value: UartStopBits) -> Self {
        match value {
            UartStopBits::One => Self::One,
            UartStopBits::Two => Self::Two,
        }
    }
}

impl From<uart::StopBits> for UartStopBits {
    fn from(value: uart::StopBits) -> Self {
        match value {
            uart::StopBits::One => Self::One,
            uart::StopBits::Two => Self::Two,
        }
    }
}

impl From<UartParityType> for usbd_serial::ParityType {
    fn from(value: UartParityType) -> Self {
        match value {
            UartParityType::None => Self::None,
            UartParityType::Even => Self::Even,
            UartParityType::Odd => Self::Odd,
        }
    }
}
impl TryFrom<usbd_serial::ParityType> for UartParityType {
    type Error = UartConvertError;
    fn try_from(value: usbd_serial::ParityType) -> Result<Self, Self::Error> {
        match value {
            usbd_serial::ParityType::None => Ok(Self::None),
            usbd_serial::ParityType::Even => Ok(Self::Even),
            usbd_serial::ParityType::Odd => Ok(Self::Odd),
            _ => Err(Self::Error::Incompatible),
        }
    }
}

impl From<UartParityType> for Option<uart::Parity> {
    fn from(value: UartParityType) -> Self {
        match value {
            UartParityType::None => None,
            UartParityType::Even => Some(uart::Parity::Even),
            UartParityType::Odd => Some(uart::Parity::Odd),
        }
    }
}
impl From<Option<uart::Parity>> for UartParityType {
    fn from(value: Option<uart::Parity>) -> Self {
        match value {
            None => Self::None,
            Some(uart::Parity::Even) => Self::Even,
            Some(uart::Parity::Odd) => Self::Odd,
        }
    }
}

impl From<uart::DataBits> for UartDataBits {
    fn from(value: uart::DataBits) -> Self {
        match value {
            uart::DataBits::Five => Self(5),
            uart::DataBits::Six => Self(6),
            uart::DataBits::Seven => Self(7),
            uart::DataBits::Eight => Self(8),
        }
    }
}
impl From<UartDataBits> for uart::DataBits {
    fn from(value: UartDataBits) -> Self {
        match value.0 {
            5 => Self::Five,
            6 => Self::Six,
            7 => Self::Seven,
            8 => Self::Eight,
            _ => panic!("Incompative UART data bits."),
        }
    }
}
impl TryFrom<u8> for UartDataBits {
    type Error = UartConvertError;
    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            5 => Ok(Self(5)),
            6 => Ok(Self(6)),
            7 => Ok(Self(7)),
            8 => Ok(Self(8)),
            _ => Err(Self::Error::Incompatible),
        }
    }
}

impl From<uart::UartConfig> for UartConfig {
    fn from(value: uart::UartConfig) -> Self {
        Self {
            data_bits: value.data_bits.into(),
            stop_bits: value.stop_bits.into(),
            parity_type: value.parity.into(),
            data_rate: value.baudrate.raw(),
        }
    }
}
impl From<&UartConfig> for uart::UartConfig {
    fn from(value: &UartConfig) -> Self {
        let mut config = uart::UartConfig::default();
        config.data_bits = value.data_bits.into();
        config.stop_bits = value.stop_bits.into();
        config.parity = value.parity_type.into();
        config.baudrate = fugit::HertzU32::from_raw(value.data_rate);
        config
    }
}
impl TryFrom<&usbd_serial::LineCoding> for UartConfig {
    type Error = UartConvertError;
    fn try_from(value: &usbd_serial::LineCoding) -> Result<Self, Self::Error> {
        let data_bits = value.data_bits().try_into()?;
        let stop_bits = value.stop_bits().try_into()?;
        let parity_type = value.parity_type().try_into()?;
        let data_rate = value.data_rate();
        Ok(Self {
            data_bits,
            stop_bits,
            parity_type,
            data_rate,
        })
    }
}
