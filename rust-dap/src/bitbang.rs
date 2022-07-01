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
use embedded_hal::digital::v2::{InputPin, IoPin, OutputPin, PinState};

pub trait DelayFunc {
    fn cycle_delay(&self, cycles: u32);
}

pub struct SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn>
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    DelayFn: DelayFunc,
{
    swdio_in: Option<SwdIoInputPin>,
    swdio_out: Option<SwdIoOutputPin>,
    swclk_in: Option<SwClkInputPin>,
    swclk_out: Option<SwClkOutputPin>,
    cycle_delay: DelayFn,
}

impl<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn>
    SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn>
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    DelayFn: DelayFunc,
{
    pub fn new(swclk: SwClkInputPin, swdio: SwdIoInputPin, cycle_delay: DelayFn) -> Self {
        Self {
            swdio_in: Some(swdio),
            swdio_out: None,
            swclk_in: Some(swclk),
            swclk_out: None,
            cycle_delay: cycle_delay,
        }
    }
}

impl<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn> BitBangSwdIo
    for SwdIoSet<SwClkInputPin, SwClkOutputPin, SwdIoInputPin, SwdIoOutputPin, DelayFn>
where
    SwClkInputPin: InputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwClkOutputPin: OutputPin + IoPin<SwClkInputPin, SwClkOutputPin>,
    SwdIoInputPin: InputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    SwdIoOutputPin: OutputPin + IoPin<SwdIoInputPin, SwdIoOutputPin>,
    DelayFn: DelayFunc,
{
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
    fn get_swdio_input(&mut self) -> bool {
        self.swdio_in
            .as_mut()
            .and_then(|p| Some(p.is_high().unwrap_or(false)))
            .unwrap()
    }
    fn clock_wait(&self, config: &SwdIoConfig) {
        self.cycle_delay.cycle_delay(config.clock_wait_cycles);
    }
}

pub trait BitBangSwdIo {
    fn to_swclk_in(&mut self);
    fn to_swclk_out(&mut self, output: bool);
    fn to_swdio_in(&mut self);
    fn to_swdio_out(&mut self, output: bool);
    fn set_swclk_output(&mut self, output: bool);
    fn set_swdio_output(&mut self, output: bool);
    fn get_swdio_input(&mut self) -> bool;
    fn clock_wait(&self, config: &SwdIoConfig);
}

pub trait PrimitiveSwdIo {
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
    fn connect(&mut self) {
        self.to_swclk_out(false);
        self.to_swdio_out(false);
    }
    fn disconnect(&mut self) {
        self.to_swclk_in();
        self.to_swdio_in();
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
        return Err(DapError::SwdError(ack));
    }

    fn enable_output(&mut self) {
        PrimitiveSwdIo::enable_output(self);
    }

    fn disable_output(&mut self) {
        PrimitiveSwdIo::disable_output(self);
    }
}
