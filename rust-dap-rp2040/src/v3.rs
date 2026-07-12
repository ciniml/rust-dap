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

//! RP2040 glue for the rust-dap v3 architecture.

use crate::util::UsbIdentity;
use hal::gpio::{Floating, Input, Output, Pin, PinId, PushPull};
use hal::usb::UsbBus;
use rp2040_hal as hal;
use rust_dap::v3::bitbang::{BidirPin, BitBangSwd};
use rust_dap::v3::{CmsisDap, DapConfig, Delay};
use rust_dap::{USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON};
use usb_device::device::{UsbDevice, UsbDeviceBuilder};
use usb_device::class_prelude::UsbBusAllocator;
use usbd_serial::SerialPort;

/// Bidirectional pin backed by the rp2040-hal type-state GPIO API.
/// Holds the pin as an enum of its two mode states, so one type covers
/// both directions without the IoPin input/output type pair.
pub enum PicoBidirPin<I: PinId> {
    Input(Pin<I, Input<Floating>>),
    Output(Pin<I, Output<PushPull>>),
    /// Transient state during a mode switch; never observable.
    Invalid,
}

impl<I: PinId> PicoBidirPin<I> {
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self::Input(pin)
    }
}

impl<I: PinId> BidirPin for PicoBidirPin<I> {
    fn set_mode_output(&mut self, high: bool) {
        let state = if high {
            embedded_hal::digital::v2::PinState::High
        } else {
            embedded_hal::digital::v2::PinState::Low
        };
        *self = match core::mem::replace(self, Self::Invalid) {
            Self::Input(pin) => Self::Output(pin.into_push_pull_output_in_state(state)),
            Self::Output(mut pin) => {
                use embedded_hal::digital::v2::OutputPin;
                pin.set_state(state).ok();
                Self::Output(pin)
            }
            Self::Invalid => unreachable!(),
        };
    }

    fn set_mode_input(&mut self) {
        *self = match core::mem::replace(self, Self::Invalid) {
            Self::Input(pin) => Self::Input(pin),
            Self::Output(pin) => Self::Input(pin.into_floating_input()),
            Self::Invalid => unreachable!(),
        };
    }

    fn write(&mut self, high: bool) {
        if let Self::Output(pin) = self {
            use embedded_hal::digital::v2::OutputPin;
            if high {
                pin.set_high().ok();
            } else {
                pin.set_low().ok();
            }
        }
    }

    fn read(&mut self) -> bool {
        use embedded_hal::digital::v2::InputPin;
        match self {
            Self::Input(pin) => pin.is_high().unwrap_or(false),
            _ => false,
        }
    }
}

/// Cycle delay based on cortex_m::asm::delay.
pub struct CortexMDelay;

impl Delay for CortexMDelay {
    fn delay_cycles(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

/// Bit-banging SWD transport over three RP2040 GPIO pins.
pub type SwdIoSet<Clk, Dio, Rst> =
    BitBangSwd<PicoBidirPin<Clk>, PicoBidirPin<Dio>, PicoBidirPin<Rst>, CortexMDelay>;

/// Bit-banging JTAG transport over six RP2040 GPIO pins.
pub type JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> = rust_dap::v3::bitbang::BitBangJtag<
    PicoBidirPin<Tck>,
    PicoBidirPin<Tms>,
    PicoBidirPin<Tdi>,
    PicoBidirPin<Tdo>,
    PicoBidirPin<Trst>,
    PicoBidirPin<Srst>,
    CortexMDelay,
>;

/// Initialize USB-UART, v3 CMSIS-DAP and the USB device.
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
    T: rust_dap::v3::DapTransport,
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

/// v3 adapters for the existing PIO transports.
///
/// These delegate to the legacy `SwdIo`/`JtagIo` implementations so that PIO
/// builds run on the v3 dispatcher today; the PIO internals will move to a
/// native `DapTransport` implementation (with runtime SWD/JTAG switching on
/// shared pins) when the legacy trait stack is removed.
#[cfg(not(feature = "bitbang"))]
mod pio_adapter {
    use crate::pio::{jtag::JtagIoSet as PioJtagIoSet, swd::SwdIoSet as PioSwdIoSet};
    use rust_dap::v3::{
        ActivePort, ConnectPort, DapCapabilities, DapConfig, DapError, DapTransport,
        JtagSequenceInfo, SwdRequest, SwjPins,
    };
    use rust_dap::{CmsisDapCommandInner, CmsisDapConfig, JtagIo, JtagIoConfig, SwdIo, SwdIoConfig};

    fn legacy_swd_config(config: &DapConfig) -> SwdIoConfig {
        SwdIoConfig {
            clock_wait_cycles: config.swd.clock_wait_cycles,
            idle_cycles: config.swd.idle_cycles,
            turn_around_cycles: config.swd.turn_around_cycles,
            always_generate_data_phase: config.swd.always_generate_data_phase,
        }
    }

    fn legacy_jtag_config(config: &DapConfig) -> JtagIoConfig {
        let mut ir_length = [0u8; 256];
        ir_length[..config.jtag.ir_length.len()].copy_from_slice(&config.jtag.ir_length);
        JtagIoConfig {
            clock_wait_cycles: config.jtag.clock_wait_cycles,
            device_count: config.jtag.device_count,
            ir_length,
            idle_cycles: config.jtag.idle_cycles,
        }
    }

    fn legacy_config(config: &DapConfig) -> CmsisDapConfig {
        CmsisDapConfig {
            swdio: legacy_swd_config(config),
            jtag: legacy_jtag_config(config),
            retry_count: config.retry_count,
            match_mask: config.match_mask,
            match_retry_count: config.match_retry_count,
            capabilities: rust_dap::DapCapabilities::empty(),
        }
    }

    impl<C, D, E> DapTransport for PioSwdIoSet<C, D, E> {
        fn capabilities(&self) -> DapCapabilities {
            DapCapabilities::SWD
        }

        fn connect(
            &mut self,
            port: ConnectPort,
            _config: &DapConfig,
        ) -> Result<ActivePort, DapError> {
            match port {
                ConnectPort::Default | ConnectPort::Swd => {
                    SwdIo::connect(self);
                    Ok(ActivePort::Swd)
                }
                ConnectPort::Jtag => Err(DapError::NotSupported),
            }
        }

        fn disconnect(&mut self, _config: &DapConfig) -> Result<(), DapError> {
            SwdIo::disconnect(self);
            Ok(())
        }

        fn swj_sequence(
            &mut self,
            config: &DapConfig,
            count: usize,
            data: &[u8],
        ) -> Result<(), DapError> {
            SwdIo::swj_sequence(self, &legacy_swd_config(config), count, data);
            Ok(())
        }

        fn swj_pins(
            &mut self,
            config: &DapConfig,
            output: SwjPins,
            select: SwjPins,
            wait_us: u32,
        ) -> Result<SwjPins, DapError> {
            CmsisDapCommandInner::swj_pins(
                self,
                &legacy_config(config),
                output.bits(),
                select.bits(),
                wait_us,
            )
            .map(SwjPins::from_bits_truncate)
        }

        fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
            let mut legacy = legacy_swd_config(config);
            SwdIo::swj_clock(self, &mut legacy, frequency_hz)
        }

        fn swd_transfer(
            &mut self,
            config: &DapConfig,
            request: SwdRequest,
            data: u32,
        ) -> Result<u32, DapError> {
            SwdIo::swd_transfer(self, &legacy_swd_config(config), request, data)
        }

        fn swd_read_bits(
            &mut self,
            config: &DapConfig,
            count: usize,
            data: &mut [u8],
        ) -> Result<(), DapError> {
            SwdIo::swd_read_sequence(self, &legacy_swd_config(config), count, data);
            Ok(())
        }

        fn swd_write_bits(
            &mut self,
            config: &DapConfig,
            count: usize,
            data: &[u8],
        ) -> Result<(), DapError> {
            SwdIo::swd_write_sequence(self, &legacy_swd_config(config), count, data);
            Ok(())
        }

        fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> {
            if enable {
                SwdIo::enable_output(self);
            } else {
                SwdIo::disable_output(self);
            }
            Ok(())
        }
    }

    impl<Tck, Tms, Tdi, Tdo, Trst, Srst> DapTransport
        for PioJtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst>
    {
        fn capabilities(&self) -> DapCapabilities {
            DapCapabilities::JTAG
        }

        fn connect(
            &mut self,
            port: ConnectPort,
            config: &DapConfig,
        ) -> Result<ActivePort, DapError> {
            match port {
                ConnectPort::Default | ConnectPort::Jtag => {
                    JtagIo::connect(self, &legacy_jtag_config(config));
                    Ok(ActivePort::Jtag)
                }
                ConnectPort::Swd => Err(DapError::NotSupported),
            }
        }

        fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError> {
            JtagIo::disconnect(self, &legacy_jtag_config(config));
            Ok(())
        }

        fn swj_sequence(
            &mut self,
            config: &DapConfig,
            count: usize,
            data: &[u8],
        ) -> Result<(), DapError> {
            JtagIo::swj_sequence(self, &legacy_jtag_config(config), count, data);
            Ok(())
        }

        fn swj_pins(
            &mut self,
            config: &DapConfig,
            output: SwjPins,
            select: SwjPins,
            wait_us: u32,
        ) -> Result<SwjPins, DapError> {
            CmsisDapCommandInner::swj_pins(
                self,
                &legacy_config(config),
                output.bits(),
                select.bits(),
                wait_us,
            )
            .map(SwjPins::from_bits_truncate)
        }

        fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
            let mut legacy = legacy_jtag_config(config);
            JtagIo::swj_clock(self, &mut legacy, frequency_hz)
        }

        fn jtag_transfer(
            &mut self,
            config: &DapConfig,
            dap_index: u8,
            request: SwdRequest,
            data: u32,
        ) -> Result<u32, DapError> {
            JtagIo::jtag_transfer(self, &legacy_jtag_config(config), dap_index, request, data)
        }

        fn jtag_sequence(
            &mut self,
            config: &DapConfig,
            info: &JtagSequenceInfo,
            tdi_data: u64,
        ) -> Result<Option<u64>, DapError> {
            CmsisDapCommandInner::jtag_sequence(self, &legacy_config(config), info, tdi_data)
        }

        fn jtag_idcode(&mut self, config: &DapConfig, index: u8) -> Result<u32, DapError> {
            JtagIo::jtag_idcode(self, &legacy_jtag_config(config), index)
        }
    }
}
