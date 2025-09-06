// Copyright 2021-2022 Kenta Ida
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

use crate::cmsis_dap::*;
use bitvec::slice::BitSlice;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, PinState};

pub trait DelayFunc {
    fn calculate_half_clock_cycles(_frequency_hz: u32) -> Option<u32> {
        None
    }
    fn cycle_delay(&self, cycles: u32);
}

// A copy of digital::v2::IoPin taken from embedded-hal = "0.2" .
pub trait IoPin<TInput, TOutput>: ErrorType
where
    TInput: InputPin + IoPin<TInput, TOutput>,
    TOutput: OutputPin + IoPin<TInput, TOutput>,
{
    /// Tries to convert this pin to input mode.
    ///
    /// If the pin is already in input mode, this method should succeed.
    fn into_input_pin(self) -> Result<TInput, Self::Error>;

    /// Tries to convert this pin to output mode with the given initial state.
    ///
    /// If the pin is already in the requested state, this method should
    /// succeed.
    fn into_output_pin(self, state: PinState) -> Result<TOutput, Self::Error>;
}

fn turn_to_in<I: InputPin + IoPin<I, O>, O: OutputPin + IoPin<I, O>>(
    pin_in: &mut Option<I>,
    pin_out: &mut Option<O>,
) {
    let mut pin = None;
    core::mem::swap(&mut pin, pin_out);
    if let Some(pin_out) = pin {
        *pin_in = Some(
            pin_out
                .into_input_pin()
                .unwrap_or_else(|_| panic!("Failed to turn pin to input.")),
        );
    }
}
fn turn_to_out<I: InputPin + IoPin<I, O>, O: OutputPin + IoPin<I, O>>(
    pin_in: &mut Option<I>,
    pin_out: &mut Option<O>,
    output: bool,
) {
    let mut pin = None;
    core::mem::swap(&mut pin, pin_in);
    if let Some(pin_in) = pin {
        let state = if output {
            PinState::High
        } else {
            PinState::Low
        };
        *pin_out = Some(
            pin_in
                .into_output_pin(state)
                .unwrap_or_else(|_| panic!("Failed to turn pin to output.")),
        );
    }
}

fn set_output<I: InputPin + IoPin<I, O>, O: OutputPin + IoPin<I, O>>(
    pin_out: &mut Option<O>,
    output: bool,
) {
    pin_out.as_mut().and_then(|p| {
        if output {
            p.set_high().ok()
        } else {
            p.set_low().ok()
        }
    });
}
fn get_input<I: InputPin + IoPin<I, O>, O: OutputPin + IoPin<I, O>>(
    pin_in: &mut Option<I>,
) -> bool {
    if let Some(pin_in) = pin_in {
        pin_in
            .is_high()
            .unwrap_or_else(|_| panic!("Failed to get input pin is high"))
    } else {
        false
    }
}

///////////////////
/////// SWD ///////
///////////////////
pub struct SwdIoSet<
    SwClkInputPin,
    SwClkOutputPin,
    SwdIoInputPin,
    SwdIoOutputPin,
    ResetInputPin,
    ResetOutputPin,
    DelayFn,
> where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    ResetInputPin: InputPin + IoPin<ResetInputPin, ResetOutputPin>,
    ResetOutputPin: OutputPin + IoPin<ResetInputPin, ResetOutputPin>,
    DelayFn: DelayFunc,
{
    swdio_in: Option<SwdIoInputPin>,
    swdio_out: Option<SwdIoOutputPin>,
    swclk_in: Option<SwClkInputPin>,
    swclk_out: Option<SwClkOutputPin>,
    reset_in: Option<ResetInputPin>,
    reset_out: Option<ResetOutputPin>,
    cycle_delay: DelayFn,
}

impl<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    >
    SwdIoSet<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    >
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    ResetInputPin: InputPin + IoPin<ResetInputPin, ResetOutputPin>,
    ResetOutputPin: OutputPin + IoPin<ResetInputPin, ResetOutputPin>,
    DelayFn: DelayFunc,
{
    pub fn new(
        swclk: SwClkInputPin,
        swdio: SwdIoInputPin,
        reset: ResetInputPin,
        cycle_delay: DelayFn,
    ) -> Self {
        Self {
            swdio_in: Some(swdio),
            swdio_out: None,
            swclk_in: Some(swclk),
            swclk_out: None,
            reset_in: Some(reset),
            reset_out: None,
            cycle_delay,
        }
    }
}

impl<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    > BitBangSwdIo
    for SwdIoSet<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    >
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    ResetInputPin: InputPin + IoPin<ResetInputPin, ResetOutputPin>,
    ResetOutputPin: OutputPin + IoPin<ResetInputPin, ResetOutputPin>,
    DelayFn: DelayFunc,
{
    fn calculate_half_clock_cycles(frequency_hz: u32) -> Option<u32> {
        DelayFn::calculate_half_clock_cycles(frequency_hz)
    }

    fn to_swclk_in(&mut self) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swclk_out);
        if let Some(swclk_out) = pin {
            self.swclk_in = Some(
                swclk_out
                    .into_input_pin()
                    .unwrap_or_else(|_| panic!("Failed to turn SWCLK pin to input.")),
            );
        }
    }
    fn to_swclk_out(&mut self, output: bool) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swclk_in);
        if let Some(swclk_in) = pin {
            let state = if output {
                PinState::High
            } else {
                PinState::Low
            };
            self.swclk_out = Some(
                swclk_in
                    .into_output_pin(state)
                    .unwrap_or_else(|_| panic!("Failed to turn SWCLK pin to output.")),
            );
        }
    }
    fn to_swdio_in(&mut self) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swdio_out);
        if let Some(swdio_out) = pin {
            self.swdio_in = Some(
                swdio_out
                    .into_input_pin()
                    .unwrap_or_else(|_| panic!("Failed to turn SWDIO pin to input.")),
            );
        }
    }
    fn to_swdio_out(&mut self, output: bool) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.swdio_in);
        if let Some(swdio_in) = pin {
            let state = if output {
                PinState::High
            } else {
                PinState::Low
            };
            self.swdio_out = Some(
                swdio_in
                    .into_output_pin(state)
                    .unwrap_or_else(|_| panic!("Failed to turn SWDIO pin to output.")),
            );
        }
    }
    fn to_reset_in(&mut self) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.reset_out);
        if let Some(reset_out) = pin {
            self.reset_in = Some(
                reset_out
                    .into_input_pin()
                    .unwrap_or_else(|_| panic!("Failed to turn RESET pin to input.")),
            );
        }
    }
    fn to_reset_out(&mut self, output: bool) {
        let mut pin = None;
        core::mem::swap(&mut pin, &mut self.reset_in);
        if let Some(reset_in) = pin {
            let state = if output {
                PinState::High
            } else {
                PinState::Low
            };
            self.reset_out = Some(
                reset_in
                    .into_output_pin(state)
                    .unwrap_or_else(|_| panic!("Failed to turn RESET pin to output.")),
            );
        }
    }
    fn set_swclk_output(&mut self, output: bool) {
        self.swclk_out.as_mut().and_then(|p| {
            if output {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
    }
    fn set_swdio_output(&mut self, output: bool) {
        self.swdio_out.as_mut().and_then(|p| {
            if output {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
    }
    fn set_reset_output(&mut self, output: bool) {
        self.reset_out.as_mut().and_then(|p| {
            if output {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
    }
    fn get_swdio_input(&mut self) -> bool {
        self.swdio_in
            .as_mut()
            .map(|p| p.is_high().unwrap_or(false))
            .unwrap()
    }
    fn get_swclk_input(&mut self) -> bool {
        self.swclk_in
            .as_mut()
            .map(|p| p.is_high().unwrap_or(false))
            .unwrap()
    }
    fn get_reset_input(&mut self) -> bool {
        self.reset_in
            .as_mut()
            .map(|p| p.is_high().unwrap_or(false))
            .unwrap()
    }
    fn clock_wait(&self, config: &SwdIoConfig) {
        self.cycle_delay.cycle_delay(config.clock_wait_cycles);
    }
}

pub trait BitBangSwdIo {
    fn calculate_half_clock_cycles(_frequency_hz: u32) -> Option<u32> {
        None
    }
    fn to_swclk_in(&mut self);
    fn to_swclk_out(&mut self, output: bool);
    fn to_swdio_in(&mut self);
    fn to_swdio_out(&mut self, output: bool);
    fn to_reset_in(&mut self);
    fn to_reset_out(&mut self, output: bool);
    fn set_swclk_output(&mut self, output: bool);
    fn set_swdio_output(&mut self, output: bool);
    fn set_reset_output(&mut self, output: bool);
    fn get_swclk_input(&mut self) -> bool;
    fn get_swdio_input(&mut self) -> bool;
    fn get_reset_input(&mut self) -> bool;
    fn clock_wait(&self, config: &SwdIoConfig);
}

pub trait PrimitiveSwdIo {
    /// Calculates number of cycles to delay to generate the target clock frequency.
    fn calculate_half_clock_cycles(_frequency_hz: u32) -> Option<u32> {
        None
    }

    fn connect(&mut self);
    fn disconnect(&mut self);
    fn enable_output(&mut self);
    fn disable_output(&mut self);

    fn cycle_clock(&mut self, config: &SwdIoConfig);
    fn turn_around(&mut self, config: &SwdIoConfig);
    fn idle_cycle(&mut self, config: &SwdIoConfig);
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool);
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool;
    fn set_swdio(&mut self, value: bool);
    fn get_timestamp(&mut self) -> u32;
}

impl<Io: BitBangSwdIo> PrimitiveSwdIo for Io {
    fn calculate_half_clock_cycles(frequency_hz: u32) -> Option<u32> {
        Io::calculate_half_clock_cycles(frequency_hz)
    }

    fn connect(&mut self) {
        self.to_swclk_out(false);
        self.to_swdio_out(false);
        self.to_reset_out(true);
    }
    fn disconnect(&mut self) {
        self.to_swclk_in();
        self.to_swdio_in();
        self.to_reset_in();
    }
    fn enable_output(&mut self) {
        self.to_swdio_out(false);
    }
    fn disable_output(&mut self) {
        self.to_swdio_in();
    }

    fn cycle_clock(&mut self, config: &SwdIoConfig) {
        self.set_swclk_output(false);
        self.clock_wait(config);
        self.set_swclk_output(true);
        self.clock_wait(config);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.turn_around_cycles {
            self.cycle_clock(config);
        }
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.idle_cycles {
            self.write_bit(config, false);
        }
    }
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool) {
        self.set_swdio_output(value);
        self.set_swclk_output(false);
        self.clock_wait(config);
        self.set_swclk_output(true);
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool {
        self.set_swclk_output(false);
        self.clock_wait(config);
        let value = self.get_swdio_input();
        self.set_swclk_output(true);
        self.clock_wait(config);
        value
    }
    fn set_swdio(&mut self, value: bool) {
        self.set_swdio_output(value);
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}

impl<Io: PrimitiveSwdIo> SwdIo for Io {
    fn connect(&mut self) {
        PrimitiveSwdIo::connect(self)
    }
    fn disconnect(&mut self) {
        PrimitiveSwdIo::disconnect(self)
    }
    fn swj_clock(
        &mut self,
        config: &mut SwdIoConfig,
        frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        match Self::calculate_half_clock_cycles(frequency_hz) {
            Some(cycles) => config.clock_wait_cycles = cycles, // Update clock_wait_cycles.
            _ => {}
        }
        Ok(())
    }
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;

        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
    }
    fn swd_read_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &mut [u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = 0;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                let bit_value = self.read_bit(config);
                value = if bit_value {
                    (value >> 1) | 0x80
                } else {
                    value >> 1
                };
            }
            value >>= bits;
            data[index] = value;
            index += 1;
        }
    }

    fn swd_write_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = data[index];
            index += 1;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                self.write_bit(config, value & 1 != 0);
                value >>= 1;
            }
        }
    }

    fn swd_transfer(
        &mut self,
        config: &SwdIoConfig,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        // write request
        self.enable_output();
        {
            let mut parity = false;
            self.write_bit(config, true); // Start
            let bit = request.contains(SwdRequest::APnDP);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::RnW);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A2);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A3);
            self.write_bit(config, bit);
            parity ^= bit;
            self.write_bit(config, parity); // Parity
            self.write_bit(config, false); // Stop
            self.write_bit(config, true); // Park
        }

        // turnaround + read ack.
        self.disable_output();
        self.turn_around(config);
        let ack = {
            let mut ack = 0u8;
            ack |= if self.read_bit(config) { 0b001 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b010 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b100 } else { 0b000 };
            ack
        };
        if ack == DAP_TRANSFER_OK {
            let ack = if request.contains(SwdRequest::RnW) {
                // READ request
                let mut value = 0u32;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = self.read_bit(config);
                    parity ^= bit;
                    value = (value >> 1) | if bit { 0x80000000 } else { 0x00000000 };
                }
                let parity_expected = self.read_bit(config);
                self.turn_around(config);
                self.enable_output();
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // WRITE request
                self.turn_around(config);
                self.enable_output();
                let mut value = data;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = value & 1 != 0;
                    self.write_bit(config, bit);
                    parity ^= bit;
                    value >>= 1;
                }
                self.write_bit(config, parity);
                Ok(0)
            };
            // TODO: capture timestamp
            self.idle_cycle(config);
            self.set_swdio(true);
            return ack;
        }

        // An error occured.
        if ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT {
            self.disable_output();
            if config.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.enable_output();
            if config.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.set_swdio(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.enable_output();
        self.set_swdio(true);
        Err(DapError::SwdError(ack))
    }

    fn enable_output(&mut self) {
        PrimitiveSwdIo::enable_output(self);
    }

    fn disable_output(&mut self) {
        PrimitiveSwdIo::disable_output(self);
    }
}

impl<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    > CmsisDapCommandInner
    for SwdIoSet<
        SwClkInputPin,
        SwClkOutputPin,
        SwdIoInputPin,
        SwdIoOutputPin,
        ResetInputPin,
        ResetOutputPin,
        DelayFn,
    >
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    ResetInputPin: InputPin + IoPin<ResetInputPin, ResetOutputPin>,
    ResetOutputPin: OutputPin + IoPin<ResetInputPin, ResetOutputPin>,
    DelayFn: DelayFunc,
{
    fn connect(&mut self, _config: &CmsisDapConfig) {
        SwdIo::connect(self);
    }
    fn disconnect(&mut self, _config: &CmsisDapConfig) {
        SwdIo::disconnect(self);
    }

    fn swj_sequence(&mut self, config: &CmsisDapConfig, count: usize, data: &[u8]) {
        SwdIo::swj_sequence(self, &config.swdio, count, data);
    }

    fn swd_sequence(
        &mut self,
        config: &CmsisDapConfig,
        request: &[u8],
        response: &mut [u8],
    ) -> core::result::Result<(usize, usize), DapError> {
        if request.is_empty() {
            return Err(DapError::InvalidCommand);
        }

        let mut sequence_count = request[0];
        let mut request_index = 1;
        let mut response_index = 1;
        while sequence_count > 0 {
            sequence_count -= 1;
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
                SwdIo::disable_output(self);
                SwdIo::swd_read_sequence(
                    self,
                    &config.swdio,
                    clock_count,
                    &mut response[response_index..],
                );
                response_index += bytes_count;
            } else {
                SwdIo::enable_output(self);
                SwdIo::swd_write_sequence(
                    self,
                    &config.swdio,
                    clock_count,
                    &request[request_index..],
                );
                request_index += bytes_count;
            }

            if sequence_count == 0 {
                SwdIo::enable_output(self)
            }
        }
        response[0] = DAP_OK;
        Ok((request_index, response_index))
    }

    fn transfer_inner_with_retry(
        &mut self,
        config: &CmsisDapConfig,
        _dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        let mut retry_count = 0;
        loop {
            match SwdIo::swd_transfer(self, &config.swdio, request, data) {
                Ok(value) => break Ok(value),
                Err(DapError::SwdError(err)) => {
                    if err != DAP_TRANSFER_WAIT || retry_count == config.retry_count {
                        break Err(DapError::SwdError(err));
                    }
                    retry_count += 1;
                }
                Err(err) => break Err(err),
            }
        }
    }
    fn swj_pins(
        &mut self,
        _config: &CmsisDapConfig,
        pin_output: u8,
        pin_select: u8,
        wait_us: u32,
    ) -> core::result::Result<u8, DapError> {
        let pin_output = SwjPins::from_bits(pin_output).unwrap();
        let pin_select = SwjPins::from_bits(pin_select).unwrap();

        let flags = [
            SwjPins::TCK_SWDCLK,
            SwjPins::TMS_SWDIO,
            SwjPins::TDI,
            SwjPins::TDO,
            SwjPins::N_TRST,
            SwjPins::N_RESET,
        ];

        // output
        for f in flags {
            if pin_select.contains(f) {
                let output = pin_output.contains(f);
                match f {
                    SwjPins::TCK_SWDCLK => {
                        // TODO: inputならoutputにする
                        // TODO: outputする値を覚えておいて、最後にもとに戻す
                        self.set_swclk_output(output);
                    }
                    SwjPins::TMS_SWDIO => {
                        self.set_swdio_output(output);
                    }
                    SwjPins::N_RESET => {
                        self.set_reset_output(output);
                    }
                    _ => (),
                }
            }
        }

        // FIXIT: get core clock from system
        // CORE_CLOCK / 1000000 * wait_us
        const CORE_CLOCK: u64 = 125000000;
        self.cycle_delay
            .cycle_delay((CORE_CLOCK * wait_us as u64 / 1000000u64) as u32);

        // backup
        let swclk_is_output = self.swclk_out.is_some();
        let swdio_is_output = self.swdio_out.is_some();
        let n_reset_is_output = self.reset_out.is_some();

        // change io
        if swclk_is_output {
            self.to_swclk_in();
        }
        if swdio_is_output {
            self.to_swdio_in();
        }
        if n_reset_is_output {
            self.to_reset_in();
        }

        // input
        let mut pin_input = if self.get_swclk_input() {
            SwjPins::TCK_SWDCLK
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_swdio_input() {
            SwjPins::TMS_SWDIO
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_reset_input() {
            SwjPins::N_RESET
        } else {
            SwjPins::empty()
        };

        // restore io
        if swclk_is_output {
            self.to_swclk_out(false);
        }
        if swdio_is_output {
            self.to_swdio_out(false);
        }
        if n_reset_is_output {
            self.to_reset_out(true);
        }

        Ok(pin_input.bits())
    }

    fn swj_clock(
        &mut self,
        config: &mut CmsisDapConfig,
        frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        SwdIo::swj_clock(self, &mut config.swdio, frequency_hz)
    }

    fn jtag_idcode(
        &self,
        _config: &mut CmsisDapConfig,
        _request: &[u8],
        _response: &mut [u8],
    ) -> core::result::Result<(usize, usize), DapError> {
        Err(DapError::InvalidCommand)
    }

    fn jtag_sequence(
        &mut self,
        _config: &CmsisDapConfig,
        _sequence_info: &JtagSequenceInfo,
        _tdi_data: u64,
    ) -> core::result::Result<Option<u64>, DapError> {
        Err(DapError::InvalidCommand)
    }
}

///////////////////
/////// JTAG ///////
///////////////////
pub struct JtagIoSet<
    TckInputPin,
    TckOutputPin,
    TmsInputPin,
    TmsOutputPin,
    TdiInputPin,
    TdiOutputPin,
    TdoInputPin,
    TdoOutputPin,
    TrstInputPin,
    TrstOutputPin,
    SrstInputPin,
    SrstOutputPin,
    DelayFn,
> where
    TckInputPin: InputPin + IoPin<TckInputPin, TckOutputPin>,
    TckOutputPin: OutputPin + IoPin<TckInputPin, TckOutputPin>,
    TmsInputPin: InputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TmsOutputPin: OutputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TdiInputPin: InputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdiOutputPin: OutputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdoInputPin: InputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TdoOutputPin: OutputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TrstInputPin: InputPin + IoPin<TrstInputPin, TrstOutputPin>,
    TrstOutputPin: OutputPin + IoPin<TrstInputPin, TrstOutputPin>,
    SrstInputPin: InputPin + IoPin<SrstInputPin, SrstOutputPin>,
    SrstOutputPin: OutputPin + IoPin<SrstInputPin, SrstOutputPin>,
    DelayFn: DelayFunc,
{
    tms_in: Option<TmsInputPin>,
    tms_out: Option<TmsOutputPin>,
    tck_in: Option<TckInputPin>,
    tck_out: Option<TckOutputPin>,
    tdi_in: Option<TdiInputPin>,
    tdi_out: Option<TdiOutputPin>,
    tdo_in: Option<TdoInputPin>,
    tdo_out: Option<TdoOutputPin>,
    ntrst_in: Option<TrstInputPin>,
    ntrst_out: Option<TrstOutputPin>,
    nsrst_in: Option<SrstInputPin>,
    nsrst_out: Option<SrstOutputPin>,
    cycle_delay: DelayFn,
}

impl<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    >
    JtagIoSet<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    >
where
    TckInputPin: InputPin + IoPin<TckInputPin, TckOutputPin>,
    TckOutputPin: OutputPin + IoPin<TckInputPin, TckOutputPin>,
    TmsInputPin: InputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TmsOutputPin: OutputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TdiInputPin: InputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdiOutputPin: OutputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdoInputPin: InputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TdoOutputPin: OutputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TrstInputPin: InputPin + IoPin<TrstInputPin, TrstOutputPin>,
    TrstOutputPin: OutputPin + IoPin<TrstInputPin, TrstOutputPin>,
    SrstInputPin: InputPin + IoPin<SrstInputPin, SrstOutputPin>,
    SrstOutputPin: OutputPin + IoPin<SrstInputPin, SrstOutputPin>,
    DelayFn: DelayFunc,
{
    pub fn new(
        tck: TckInputPin,
        tms: TmsInputPin,
        tdi: TdiInputPin,
        tdo: TdoInputPin,
        ntrst: TrstInputPin,
        nsrst: SrstInputPin,
        cycle_delay: DelayFn,
    ) -> Self {
        Self {
            tms_in: Some(tms),
            tms_out: None,
            tck_in: Some(tck),
            tck_out: None,
            tdi_in: Some(tdi),
            tdi_out: None,
            tdo_in: Some(tdo),
            tdo_out: None,
            ntrst_in: Some(ntrst),
            ntrst_out: None,
            nsrst_in: Some(nsrst),
            nsrst_out: None,
            cycle_delay: cycle_delay,
        }
    }
}

impl<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    > BitBangJtagIo
    for JtagIoSet<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    >
where
    TckInputPin: InputPin + IoPin<TckInputPin, TckOutputPin>,
    TckOutputPin: OutputPin + IoPin<TckInputPin, TckOutputPin>,
    TmsInputPin: InputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TmsOutputPin: OutputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TdiInputPin: InputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdiOutputPin: OutputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdoInputPin: InputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TdoOutputPin: OutputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TrstInputPin: InputPin + IoPin<TrstInputPin, TrstOutputPin>,
    TrstOutputPin: OutputPin + IoPin<TrstInputPin, TrstOutputPin>,
    SrstInputPin: InputPin + IoPin<SrstInputPin, SrstOutputPin>,
    SrstOutputPin: OutputPin + IoPin<SrstInputPin, SrstOutputPin>,
    DelayFn: DelayFunc,
{
    fn calculate_half_clock_cycles(frequency_hz: u32) -> Option<u32> {
        DelayFn::calculate_half_clock_cycles(frequency_hz)
    }

    // TCK
    fn to_tck_in(&mut self) {
        turn_to_in(&mut self.tck_in, &mut self.tck_out);
    }
    fn to_tck_out(&mut self, output: bool) {
        turn_to_out(&mut self.tck_in, &mut self.tck_out, output);
    }
    fn set_tck_output(&mut self, output: bool) {
        set_output(&mut self.tck_out, output);
    }
    fn get_tck_input(&mut self) -> bool {
        get_input(&mut self.tck_in)
    }
    // TMS
    fn to_tms_in(&mut self) {
        turn_to_in(&mut self.tms_in, &mut self.tms_out);
    }
    fn to_tms_out(&mut self, output: bool) {
        turn_to_out(&mut self.tms_in, &mut self.tms_out, output);
    }
    fn set_tms_output(&mut self, output: bool) {
        set_output(&mut self.tms_out, output);
    }
    fn get_tms_input(&mut self) -> bool {
        get_input(&mut self.tms_in)
    }
    // TDI
    fn to_tdi_in(&mut self) {
        turn_to_in(&mut self.tdi_in, &mut self.tdi_out);
    }
    fn to_tdi_out(&mut self, output: bool) {
        turn_to_out(&mut self.tdi_in, &mut self.tdi_out, output);
    }
    fn set_tdi_output(&mut self, output: bool) {
        set_output(&mut self.tdi_out, output);
    }
    fn get_tdi_input(&mut self) -> bool {
        get_input(&mut self.tdi_in)
    }
    // TDO
    fn to_tdo_in(&mut self) {
        turn_to_in(&mut self.tdo_in, &mut self.tdo_out);
    }
    fn to_tdo_out(&mut self, output: bool) {
        turn_to_out(&mut self.tdo_in, &mut self.tdo_out, output);
    }
    fn set_tdo_output(&mut self, output: bool) {
        set_output(&mut self.tdo_out, output);
    }
    fn get_tdo_input(&mut self) -> bool {
        get_input(&mut self.tdo_in)
    }
    // nTRST
    fn to_ntrst_in(&mut self) {
        turn_to_in(&mut self.ntrst_in, &mut self.ntrst_out);
    }
    fn to_ntrst_out(&mut self, output: bool) {
        turn_to_out(&mut self.ntrst_in, &mut self.ntrst_out, output);
    }
    fn set_ntrst_output(&mut self, output: bool) {
        set_output(&mut self.ntrst_out, output);
    }
    fn get_ntrst_input(&mut self) -> bool {
        get_input(&mut self.ntrst_in)
    }
    // nSRST
    fn to_nsrst_in(&mut self) {
        turn_to_in(&mut self.nsrst_in, &mut self.nsrst_out);
    }
    fn to_nsrst_out(&mut self, output: bool) {
        turn_to_out(&mut self.nsrst_in, &mut self.nsrst_out, output);
    }
    fn set_nsrst_output(&mut self, output: bool) {
        set_output(&mut self.nsrst_out, output);
    }
    fn get_nsrst_input(&mut self) -> bool {
        get_input(&mut self.nsrst_in)
    }
    // delay
    fn clock_wait(&self, config: &JtagIoConfig) {
        self.cycle_delay.cycle_delay(config.clock_wait_cycles);
    }
}

pub trait BitBangJtagIo {
    fn calculate_half_clock_cycles(_frequency_hz: u32) -> Option<u32> {
        None
    }

    // TCK
    fn to_tck_in(&mut self);
    fn to_tck_out(&mut self, output: bool);
    fn set_tck_output(&mut self, output: bool);
    fn get_tck_input(&mut self) -> bool;
    // TMS
    fn to_tms_in(&mut self);
    fn to_tms_out(&mut self, output: bool);
    fn set_tms_output(&mut self, output: bool);
    fn get_tms_input(&mut self) -> bool;
    // TDI
    fn to_tdi_in(&mut self);
    fn to_tdi_out(&mut self, output: bool);
    fn set_tdi_output(&mut self, output: bool);
    fn get_tdi_input(&mut self) -> bool;
    // TDO
    fn to_tdo_in(&mut self);
    fn to_tdo_out(&mut self, output: bool);
    fn set_tdo_output(&mut self, output: bool);
    fn get_tdo_input(&mut self) -> bool;
    // nTRST
    fn to_ntrst_in(&mut self);
    fn to_ntrst_out(&mut self, output: bool);
    fn set_ntrst_output(&mut self, output: bool);
    fn get_ntrst_input(&mut self) -> bool;
    // nSRST
    fn to_nsrst_in(&mut self);
    fn to_nsrst_out(&mut self, output: bool);
    fn set_nsrst_output(&mut self, output: bool);
    fn get_nsrst_input(&mut self) -> bool;
    // delay
    fn clock_wait(&self, config: &JtagIoConfig);
}

pub trait PrimitiveJtagIo {
    fn calculate_half_clock_cycles(_frequency_hz: u32) -> Option<u32> {
        None
    }

    fn connect(&mut self, config: &JtagIoConfig);
    fn disconnect(&mut self, config: &JtagIoConfig);
    fn write_bit(&mut self, config: &JtagIoConfig, tms: bool, tdi: bool);
    fn read_bit(&mut self, config: &JtagIoConfig, tms: bool, tdi: bool) -> bool;
}

impl<Io: BitBangJtagIo> PrimitiveJtagIo for Io {
    fn calculate_half_clock_cycles(frequency_hz: u32) -> Option<u32> {
        Io::calculate_half_clock_cycles(frequency_hz)
    }

    fn connect(&mut self, config: &JtagIoConfig) {
        // initial value
        self.to_tck_out(false);
        self.to_tms_out(false);
        self.to_tdi_out(false);
        self.to_tdo_in();
        self.to_ntrst_out(true);
        self.to_nsrst_out(true);

        // reset with nTRST
        // TODO: wait 1ms
        self.set_ntrst_output(false);
        self.clock_wait(config);
        self.set_ntrst_output(true);
        self.clock_wait(config);
        // reset JTAG state machine
        for _i in 0..10 {
            self.write_bit(config, true, false);
        }
    }
    fn disconnect(&mut self, config: &JtagIoConfig) {
        // reset JTAG state machine
        for _i in 0..10 {
            self.write_bit(config, true, false);
        }

        self.to_tck_in();
        self.to_tms_in();
        self.to_tdi_in();
        self.to_tdo_in();
        self.to_ntrst_in();
        self.to_nsrst_in();
    }

    fn write_bit(&mut self, config: &JtagIoConfig, tms: bool, tdi: bool) {
        self.set_tms_output(tms);
        self.set_tdi_output(tdi);
        self.set_tck_output(false);
        self.clock_wait(config);
        self.set_tck_output(true);
        self.clock_wait(config);
    }

    fn read_bit(&mut self, config: &JtagIoConfig, tms: bool, tdi: bool) -> bool {
        self.set_tms_output(tms);
        self.set_tdi_output(tdi);
        self.set_tck_output(false);
        self.clock_wait(config);
        let value = self.get_tdo_input();
        self.set_tck_output(true);
        self.clock_wait(config);
        value
    }
}

impl<Io: PrimitiveJtagIo> JtagIo for Io {
    fn connect(&mut self, config: &JtagIoConfig) {
        PrimitiveJtagIo::connect(self, config)
    }
    fn disconnect(&mut self, config: &JtagIoConfig) {
        PrimitiveJtagIo::disconnect(self, config)
    }

    fn swj_clock(
        &mut self,
        config: &mut JtagIoConfig,
        frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        match Self::calculate_half_clock_cycles(frequency_hz) {
            Some(cycles) => config.clock_wait_cycles = cycles,
            _ => {}
        }
        Ok(())
    }

    fn swj_sequence(&mut self, config: &JtagIoConfig, count: usize, data: &[u8]) {
        // https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__SWJ__Sequence.html
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;
        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0, false);
            value >>= 1;
            bits -= 1;
        }
    }
    fn jtag_read_sequence(
        &mut self,
        config: &JtagIoConfig,
        clock_count: usize,
        tms_value: bool,
        tdi_data: u64,
    ) -> u64 {
        let tdi_data = tdi_data;
        let mut tdo_data = 0;
        for i in 0..clock_count {
            assert!(clock_count <= 64);
            let tdo = self.read_bit(config, tms_value, (tdi_data & (1 << i)) != 0);
            tdo_data |= if tdo { 1 } else { 0 } << i;
        }
        tdo_data
    }

    fn jtag_write_sequence(
        &mut self,
        config: &JtagIoConfig,
        clock_count: usize,
        tms_value: bool,
        tdi_data: u64,
    ) {
        let tdi_data = tdi_data;
        for i in 0..clock_count {
            self.write_bit(config, tms_value, (tdi_data & (1 << i)) != 0);
        }
    }

    fn jtag_idcode(
        &mut self,
        _config: &JtagIoConfig,
        _index: u8,
    ) -> core::result::Result<u32, DapError> {
        // https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__jtag__idcode.html#details
        unimplemented!();
    }

    fn write_ir(&mut self, config: &JtagIoConfig, dap_index: u8, ir: u32) {
        // to ShiftIR from Run-Test-Idle
        self.write_bit(config, true, false); // SelectDR
        self.write_bit(config, true, false); // SelectIR
        self.write_bit(config, false, false); // CaptureIR
        self.write_bit(config, false, false); // ShiftIR

        // dap_indexの示すIRレジスタ以外はBYPASS(all 1)にする
        let device_count = config.device_count;
        let ir_length = &config.ir_length[0..(device_count as usize)];
        let bit_count_max = ir_length.iter().map(|x| *x as usize).sum();
        let mut bit_count: usize = 0;
        let mut ir = ir;
        for (i, length) in ir_length.iter().enumerate().rev() {
            for _j in 0..*length {
                let tdi = if i == (dap_index as usize) {
                    let bit = (ir & 1) != 0;
                    ir >>= 1;
                    bit
                } else {
                    true
                };
                bit_count += 1;
                // last 1bit は TMS を true にしてExit-IRにする
                let tms = bit_count == bit_count_max;
                self.write_bit(config, tms, tdi);
            }
        }

        // to Run from Exit-IR
        self.write_bit(config, true, false); // Update-IR
        self.write_bit(config, false, false); // Run
    }

    fn read_write_dr(&mut self, config: &JtagIoConfig, dap_index: u8, dr: &mut BitSlice<u32>) {
        // to ShiftIR from Run-Test-Idle
        self.write_bit(config, true, false); // SelectDR
        self.write_bit(config, false, false); // CaptureDR
        self.write_bit(config, false, false); // ShiftDR

        // DRの内容を取得する
        let head_bits = config.ir_length[0..(config.device_count as usize)]
            .iter()
            .take(dap_index.into())
            .map(|x| *x as usize)
            .sum();
        let tail_bits = config.ir_length[0..(config.device_count as usize)]
            .iter()
            .skip(dap_index as usize + 1)
            .map(|x| *x as usize)
            .sum();
        let total_bits = config.ir_length[0..(config.device_count as usize)]
            .iter()
            .map(|x| *x as usize)
            .sum();
        let mut bit_count: usize = 0;
        for _i in 0..tail_bits {
            // dap_index以降のDR用
            bit_count += 1;
            self.write_bit(config, false, false);
        }
        for mut dr_bit in dr {
            bit_count += 1;
            let tms = bit_count == total_bits;
            let tdo = self.read_bit(config, tms, *dr_bit);
            dr_bit.set(tdo);
        }
        for _i in 0..head_bits {
            bit_count += 1;
            let tms = bit_count == total_bits;
            self.write_bit(config, tms, false);
        }

        // to Run from Exit-DR
        self.write_bit(config, true, false); // Update-DR
        self.write_bit(config, false, false); // Run
    }

    fn jtag_transfer(
        &mut self,
        config: &JtagIoConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        // https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__Transfer.html
        // JTAGを使ってDAPとの送受信をやる

        let instruction = if request.contains(SwdRequest::APnDP) {
            // APACC
            JtagInstruction::APACC as u32
        } else {
            // DPACC
            JtagInstruction::DPACC as u32
        };
        let read = request.contains(SwdRequest::RnW);
        let a2 = request.contains(SwdRequest::A2);
        let a3 = request.contains(SwdRequest::A3);

        // DPACCかAPACCを発行する
        self.write_ir(config, dap_index, instruction);
        // DRを書き込む
        if read {
            let mut dr = build_acc(data, a3, a2, true);
            self.read_write_dr(config, dap_index, dr.as_mut_bitslice());
            // TODO: OK_FALSE等の情報を返す必要がないか調べる
            let dr_data = &dr[3..35];
            let mut result: u32 = 0;
            for (i, dr) in dr_data.iter().enumerate().take(32) {
                result |= if *dr { 1 << i } else { 0 };
            }
            Ok(result)
        } else {
            let mut dr = build_acc(data, a3, a2, false);
            self.read_write_dr(config, dap_index, dr.as_mut_bitslice());
            Ok(0)
        }
    }
}

impl<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    > CmsisDapCommandInner
    for JtagIoSet<
        TckInputPin,
        TckOutputPin,
        TmsInputPin,
        TmsOutputPin,
        TdiInputPin,
        TdiOutputPin,
        TdoInputPin,
        TdoOutputPin,
        TrstInputPin,
        TrstOutputPin,
        SrstInputPin,
        SrstOutputPin,
        DelayFn,
    >
where
    TckInputPin: InputPin + IoPin<TckInputPin, TckOutputPin>,
    TckOutputPin: OutputPin + IoPin<TckInputPin, TckOutputPin>,
    TmsInputPin: InputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TmsOutputPin: OutputPin + IoPin<TmsInputPin, TmsOutputPin>,
    TdiInputPin: InputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdiOutputPin: OutputPin + IoPin<TdiInputPin, TdiOutputPin>,
    TdoInputPin: InputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TdoOutputPin: OutputPin + IoPin<TdoInputPin, TdoOutputPin>,
    TrstInputPin: InputPin + IoPin<TrstInputPin, TrstOutputPin>,
    TrstOutputPin: OutputPin + IoPin<TrstInputPin, TrstOutputPin>,
    SrstInputPin: InputPin + IoPin<SrstInputPin, SrstOutputPin>,
    SrstOutputPin: OutputPin + IoPin<SrstInputPin, SrstOutputPin>,
    DelayFn: DelayFunc,
{
    fn connect(&mut self, config: &CmsisDapConfig) {
        JtagIo::connect(self, &config.jtag);
    }
    fn disconnect(&mut self, config: &CmsisDapConfig) {
        JtagIo::disconnect(self, &config.jtag);
    }

    fn swj_sequence(&mut self, config: &CmsisDapConfig, count: usize, data: &[u8]) {
        JtagIo::swj_sequence(self, &config.jtag, count, data);
    }

    fn swj_clock(
        &mut self,
        config: &mut CmsisDapConfig,
        frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        JtagIo::swj_clock(self, &mut config.jtag, frequency_hz)
    }

    fn swd_sequence(
        &mut self,
        _config: &CmsisDapConfig,
        _request: &[u8],
        _response: &mut [u8],
    ) -> core::result::Result<(usize, usize), DapError> {
        Err(DapError::InvalidCommand)
    }

    fn jtag_sequence(
        &mut self,
        config: &CmsisDapConfig,
        sequence_info: &JtagSequenceInfo,
        tdi_data: u64,
    ) -> core::result::Result<Option<u64>, DapError> {
        Ok(if sequence_info.tdo_capture {
            Some(JtagIo::jtag_read_sequence(
                self,
                &config.jtag,
                sequence_info.number_of_tck_cycles,
                sequence_info.tms_value,
                tdi_data,
            ))
        } else {
            JtagIo::jtag_write_sequence(
                self,
                &config.jtag,
                sequence_info.number_of_tck_cycles,
                sequence_info.tms_value,
                tdi_data,
            );
            None
        })
    }

    fn transfer_inner_with_retry(
        &mut self,
        config: &CmsisDapConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, DapError> {
        let mut retry_count = 0;
        loop {
            match JtagIo::jtag_transfer(self, &config.jtag, dap_index, request, data) {
                Ok(value) => break Ok(value),
                Err(DapError::SwdError(err)) => {
                    // TODO: 動作確認
                    if err != DAP_TRANSFER_WAIT || retry_count == config.retry_count {
                        break Err(DapError::SwdError(err));
                    }
                    retry_count += 1;
                }
                Err(err) => break Err(err),
            }
        }
    }
    fn swj_pins(
        &mut self,
        _config: &CmsisDapConfig,
        pin_output: u8,
        pin_select: u8,
        _wait_us: u32,
    ) -> core::result::Result<u8, DapError> {
        let pin_output = SwjPins::from_bits(pin_output).unwrap();
        let pin_select = SwjPins::from_bits(pin_select).unwrap();

        let flags = [
            SwjPins::TCK_SWDCLK,
            SwjPins::TMS_SWDIO,
            SwjPins::TDI,
            SwjPins::TDO,
            SwjPins::N_TRST,
            SwjPins::N_RESET,
        ];

        // output
        for f in flags {
            if pin_select.contains(f) {
                let output = pin_output.contains(f);
                match f {
                    SwjPins::TCK_SWDCLK => self.set_tck_output(output),
                    SwjPins::TMS_SWDIO => self.set_tms_output(output),
                    SwjPins::TDI => self.set_tdi_output(output),
                    SwjPins::TDO => self.set_tdo_output(output),
                    SwjPins::N_TRST => self.set_ntrst_output(output),
                    SwjPins::N_RESET => self.set_nsrst_output(output),
                    _ => (),
                }
            }
        }

        // TODO: support wait_us

        // input
        let mut pin_input = if self.get_tck_input() {
            SwjPins::TCK_SWDCLK
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_tms_input() {
            SwjPins::TMS_SWDIO
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_tdi_input() {
            SwjPins::TDI
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_ntrst_input() {
            SwjPins::N_TRST
        } else {
            SwjPins::empty()
        };
        pin_input |= if self.get_nsrst_input() {
            SwjPins::N_RESET
        } else {
            SwjPins::empty()
        };

        Ok(pin_input.bits())
    }

    fn jtag_idcode(
        &self,
        _config: &mut CmsisDapConfig,
        _request: &[u8],
        _response: &mut [u8],
    ) -> core::result::Result<(usize, usize), DapError> {
        unimplemented!();
    }
}
