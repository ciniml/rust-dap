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

//! GPIO bit-banging SWD transport for the v3 architecture.
//!
//! Pins are abstracted by [`BidirPin`]: one type per pin, with direction
//! switching as a method call instead of the embedded-hal 0.2 `IoPin`
//! type-conversion pattern. This keeps one type parameter per pin and has
//! no embedded-hal dependency, so HAL crates are free to implement it on
//! whatever pin representation they like (type-state wrapper, DynPin, ...).

use super::{
    ActivePort, ConnectPort, DapConfig, DapTransport, Delay,
};
use crate::cmsis_dap::{
    DapCapabilities, DapError, SwdRequest, SwjPins, DAP_TRANSFER_FAULT, DAP_TRANSFER_MISMATCH,
    DAP_TRANSFER_OK, DAP_TRANSFER_WAIT,
};

/// A GPIO pin that can be switched between input and output at runtime.
///
/// Implementations are expected to be infallible; HALs whose pin operations
/// can fail should handle or defer those errors internally.
pub trait BidirPin {
    /// Switch the pin to push-pull output, driving `high`.
    fn set_mode_output(&mut self, high: bool);
    /// Switch the pin to high impedance input.
    fn set_mode_input(&mut self);
    /// Drive the output level. Only meaningful in output mode.
    fn write(&mut self, high: bool);
    /// Read the input level. Only guaranteed valid in input mode.
    fn read(&mut self) -> bool;
}

/// Average overhead cycles of one bit-banged half clock, subtracted from the
/// delay calculated for a requested SWJ clock frequency.
const HALF_CLOCK_OVERHEAD_CYCLES: u32 = 60;

#[derive(Clone, Copy, PartialEq, Eq)]
enum PinDir {
    Input,
    Output,
}

pub struct BitBangSwd<Clk, Dio, Rst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Rst: BidirPin,
    D: Delay,
{
    swclk: Clk,
    swdio: Dio,
    reset: Rst,
    delay: D,
    swclk_dir: PinDir,
    swdio_dir: PinDir,
    reset_dir: PinDir,
}

impl<Clk, Dio, Rst, D> BitBangSwd<Clk, Dio, Rst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Rst: BidirPin,
    D: Delay,
{
    /// Creates the transport. All pins must be in input mode.
    pub fn new(swclk: Clk, swdio: Dio, reset: Rst, delay: D) -> Self {
        Self {
            swclk,
            swdio,
            reset,
            delay,
            swclk_dir: PinDir::Input,
            swdio_dir: PinDir::Input,
            reset_dir: PinDir::Input,
        }
    }

    fn clock_wait(&self, config: &DapConfig) {
        self.delay.delay_cycles(config.swd.clock_wait_cycles);
    }

    fn cycle_clock(&mut self, config: &DapConfig) {
        self.swclk.write(false);
        self.clock_wait(config);
        self.swclk.write(true);
        self.clock_wait(config);
    }

    fn write_bit(&mut self, config: &DapConfig, value: bool) {
        self.swdio.write(value);
        self.swclk.write(false);
        self.clock_wait(config);
        self.swclk.write(true);
        self.clock_wait(config);
    }

    fn read_bit(&mut self, config: &DapConfig) -> bool {
        self.swclk.write(false);
        self.clock_wait(config);
        let value = self.swdio.read();
        self.swclk.write(true);
        self.clock_wait(config);
        value
    }

    fn turn_around(&mut self, config: &DapConfig) {
        for _ in 0..config.swd.turn_around_cycles {
            self.cycle_clock(config);
        }
    }

    fn idle_cycle(&mut self, config: &DapConfig) {
        for _ in 0..config.swd.idle_cycles {
            self.write_bit(config, false);
        }
    }

    fn swdio_to_output(&mut self, high: bool) {
        self.swdio.set_mode_output(high);
        self.swdio_dir = PinDir::Output;
    }

    fn swdio_to_input(&mut self) {
        self.swdio.set_mode_input();
        self.swdio_dir = PinDir::Input;
    }
}

impl<Clk, Dio, Rst, D> DapTransport for BitBangSwd<Clk, Dio, Rst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Rst: BidirPin,
    D: Delay,
{
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
                self.swclk.set_mode_output(false);
                self.swclk_dir = PinDir::Output;
                self.swdio_to_output(false);
                self.reset.set_mode_output(true);
                self.reset_dir = PinDir::Output;
                Ok(ActivePort::Swd)
            }
            ConnectPort::Jtag => Err(DapError::NotSupported),
        }
    }

    fn disconnect(&mut self, _config: &DapConfig) -> Result<(), DapError> {
        self.swclk.set_mode_input();
        self.swclk_dir = PinDir::Input;
        self.swdio_to_input();
        self.reset.set_mode_input();
        self.reset_dir = PinDir::Input;
        Ok(())
    }

    fn swj_sequence(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;
        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = *data.get(index).ok_or(DapError::InvalidCommand)?;
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
        Ok(())
    }

    fn swj_pins(
        &mut self,
        config: &DapConfig,
        output: SwjPins,
        select: SwjPins,
        wait_us: u32,
    ) -> Result<SwjPins, DapError> {
        // Drive the selected pins.
        if select.contains(SwjPins::TCK_SWDCLK) {
            self.swclk.write(output.contains(SwjPins::TCK_SWDCLK));
        }
        if select.contains(SwjPins::TMS_SWDIO) {
            self.swdio.write(output.contains(SwjPins::TMS_SWDIO));
        }
        if select.contains(SwjPins::N_RESET) {
            if self.reset_dir == PinDir::Input {
                self.reset
                    .set_mode_output(output.contains(SwjPins::N_RESET));
                self.reset_dir = PinDir::Output;
            } else {
                self.reset.write(output.contains(SwjPins::N_RESET));
            }
        }

        // Wait. wait_us is limited to 3 seconds by the spec.
        let wait_us = wait_us.min(3_000_000);
        let cycles = (config.core_clock_hz as u64 * wait_us as u64 / 1_000_000) as u32;
        self.delay.delay_cycles(cycles);

        // Read all pins back through input mode, then restore directions.
        let swclk_was_output = self.swclk_dir == PinDir::Output;
        let swdio_was_output = self.swdio_dir == PinDir::Output;
        let reset_was_output = self.reset_dir == PinDir::Output;
        if swclk_was_output {
            self.swclk.set_mode_input();
        }
        if swdio_was_output {
            self.swdio.set_mode_input();
        }
        if reset_was_output {
            self.reset.set_mode_input();
        }

        let mut input = SwjPins::empty();
        if self.swclk.read() {
            input |= SwjPins::TCK_SWDCLK;
        }
        if self.swdio.read() {
            input |= SwjPins::TMS_SWDIO;
        }
        if self.reset.read() {
            input |= SwjPins::N_RESET;
        }

        if swclk_was_output {
            self.swclk.set_mode_output(false);
        }
        if swdio_was_output {
            self.swdio.set_mode_output(false);
        }
        if reset_was_output {
            self.reset.set_mode_output(true);
        }

        Ok(input)
    }

    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
        if frequency_hz == 0 {
            return Err(DapError::InvalidCommand);
        }
        let cycles = (config.core_clock_hz / 2) / frequency_hz;
        config.swd.clock_wait_cycles = cycles.saturating_sub(HALF_CLOCK_OVERHEAD_CYCLES);
        Ok(())
    }

    fn swd_read_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &mut [u8],
    ) -> Result<(), DapError> {
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
            *data.get_mut(index).ok_or(DapError::InvalidCommand)? = value;
            index += 1;
        }
        Ok(())
    }

    fn swd_write_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = *data.get(index).ok_or(DapError::InvalidCommand)?;
            index += 1;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;
                self.write_bit(config, value & 1 != 0);
                value >>= 1;
            }
        }
        Ok(())
    }

    fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> {
        if enable {
            self.swdio_to_output(false);
        } else {
            self.swdio_to_input();
        }
        Ok(())
    }

    fn swd_transfer(
        &mut self,
        config: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        // Request phase.
        self.swdio_to_output(false);
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

        // Turnaround and ACK.
        self.swdio_to_input();
        self.turn_around(config);
        let ack = {
            let mut ack = 0u8;
            ack |= if self.read_bit(config) { 0b001 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b010 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b100 } else { 0b000 };
            ack
        };
        if ack == DAP_TRANSFER_OK {
            let result = if request.contains(SwdRequest::RnW) {
                // Read data phase.
                let mut value = 0u32;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = self.read_bit(config);
                    parity ^= bit;
                    value = (value >> 1) | if bit { 0x8000_0000 } else { 0 };
                }
                let parity_expected = self.read_bit(config);
                self.turn_around(config);
                self.swdio_to_output(false);
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // Write data phase.
                self.turn_around(config);
                self.swdio_to_output(false);
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
            self.idle_cycle(config);
            self.swdio.write(true);
            return result;
        }

        if ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT {
            self.swdio_to_input();
            if config.swd.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.swdio_to_output(false);
            if config.swd.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.swdio.write(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error.
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.swdio_to_output(false);
        self.swdio.write(true);
        Err(DapError::SwdError(ack))
    }
}
