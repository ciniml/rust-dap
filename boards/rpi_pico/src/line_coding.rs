use rp_pico::hal::uart;
use usbd_serial;

use core::convert::{From, TryFrom};

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartConvertError {
    Incompatible,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartStopBits {
    One,
    Two,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UartParityType {
    None,
    Odd,
    Even,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct UartDataBits(u8);

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
            UartParityType::Even => Self::Event,
            UartParityType::Odd  => Self::Odd,
        }
    }
}
impl TryFrom<usbd_serial::ParityType> for UartParityType {
    type Error = UartConvertError;
    fn try_from(value: usbd_serial::ParityType) -> Result<Self, Self::Error> {
        match value {
            usbd_serial::ParityType::None => Ok(Self::None),
            usbd_serial::ParityType::Event => Ok(Self::Even),
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
            data_rate: value.baudrate.0,
        }
    }
}
impl From<&UartConfig> for uart::UartConfig {
    fn from(value: &UartConfig) -> Self {
        let mut config = uart::common_configs::_115200_8_N_1;
        config.data_bits = value.data_bits.into();
        config.stop_bits = value.stop_bits.into();
        config.parity = value.parity_type.into();
        config.baudrate = embedded_time::rate::Baud(value.data_rate);
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