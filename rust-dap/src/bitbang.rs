// Copyright 2021-2026 Kenta Ida
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

//! GPIO bit-banging SWD/JTAG transports.
//!
//! Pins are abstracted by [`BidirPin`]: one type per pin, with direction
//! switching as a method call instead of the embedded-hal 0.2 `IoPin`
//! type-conversion pattern. This keeps one type parameter per pin and has
//! no embedded-hal dependency, so HALs are free to implement it on whatever
//! pin representation they like (type-state wrapper, DynPin, ...).
//!
//! The bit-level SWD and JTAG algorithms live in the [`SwdBits`]/[`JtagBits`]
//! borrow helpers so that the SWD-only ([`BitBangSwd`]), JTAG-only
//! ([`BitBangJtag`]) and combined ([`BitBangSwj`]) transports all share one
//! implementation. The combined transport owns a single set of pins and
//! reconfigures them in `connect(port)`, giving runtime SWD/JTAG switching
//! on shared hardware (SWCLK=TCK, SWDIO=TMS, RESET=nSRST).

use crate::cmsis_dap::{build_acc, JtagInstruction, JtagSequenceInfo};
use crate::transport::{ActivePort, ConnectPort, DapConfig, DapTransport, Delay};
use crate::{
    DapCapabilities, DapError, SwdRequest, SwjPins, DAP_TRANSFER_FAULT, DAP_TRANSFER_MISMATCH,
    DAP_TRANSFER_OK, DAP_TRANSFER_WAIT,
};
use bitvec::slice::BitSlice;

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

fn swj_clock_cycles(config: &mut DapConfig, frequency_hz: u32, jtag: bool) -> Result<(), DapError> {
    if frequency_hz == 0 {
        return Err(DapError::InvalidCommand);
    }
    let cycles = (config.core_clock_hz / 2) / frequency_hz;
    let cycles = cycles.saturating_sub(HALF_CLOCK_OVERHEAD_CYCLES);
    if jtag {
        config.jtag.clock_wait_cycles = cycles;
    } else {
        config.swd.clock_wait_cycles = cycles;
    }
    Ok(())
}

/// Packs bits into `data` LSB-first (SWD sequence read helper).
fn pack_read_bits(
    count: usize,
    data: &mut [u8],
    mut read_bit: impl FnMut() -> bool,
) -> Result<(), DapError> {
    let mut count = count;
    let mut index = 0;
    while count > 0 {
        let mut value = 0;
        let mut bits = 8;
        while bits > 0 && count > 0 {
            bits -= 1;
            count -= 1;
            value = if read_bit() {
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

////////////////////////////////////////////////////////////////////////////
// SWD bit-level algorithm (shared by BitBangSwd and BitBangSwj)
////////////////////////////////////////////////////////////////////////////

/// Borrows the three SWD pins and their direction state so the SWD algorithm
/// can be shared between the SWD-only and combined transports.
struct SwdBits<'a, Clk: BidirPin, Dio: BidirPin, Rst: BidirPin, D: Delay> {
    clk: &'a mut Clk,
    dio: &'a mut Dio,
    reset: &'a mut Rst,
    delay: &'a D,
    clk_dir: &'a mut PinDir,
    dio_dir: &'a mut PinDir,
    reset_dir: &'a mut PinDir,
}

impl<Clk: BidirPin, Dio: BidirPin, Rst: BidirPin, D: Delay> SwdBits<'_, Clk, Dio, Rst, D> {
    fn clock_wait(&self, config: &DapConfig) {
        self.delay.delay_cycles(config.swd.clock_wait_cycles);
    }
    fn cycle_clock(&mut self, config: &DapConfig) {
        self.clk.write(false);
        self.clock_wait(config);
        self.clk.write(true);
        self.clock_wait(config);
    }
    fn write_bit(&mut self, config: &DapConfig, value: bool) {
        self.dio.write(value);
        self.clk.write(false);
        self.clock_wait(config);
        self.clk.write(true);
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &DapConfig) -> bool {
        self.clk.write(false);
        self.clock_wait(config);
        let value = self.dio.read();
        self.clk.write(true);
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
    fn dio_to_output(&mut self, high: bool) {
        self.dio.set_mode_output(high);
        *self.dio_dir = PinDir::Output;
    }
    fn dio_to_input(&mut self) {
        self.dio.set_mode_input();
        *self.dio_dir = PinDir::Input;
    }

    fn connect(&mut self) {
        self.clk.set_mode_output(false);
        *self.clk_dir = PinDir::Output;
        self.dio_to_output(false);
        self.reset.set_mode_output(true);
        *self.reset_dir = PinDir::Output;
    }
    fn disconnect(&mut self) {
        self.clk.set_mode_input();
        *self.clk_dir = PinDir::Input;
        self.dio_to_input();
        self.reset.set_mode_input();
        *self.reset_dir = PinDir::Input;
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
        if select.contains(SwjPins::TCK_SWDCLK) {
            self.clk.write(output.contains(SwjPins::TCK_SWDCLK));
        }
        if select.contains(SwjPins::TMS_SWDIO) {
            self.dio.write(output.contains(SwjPins::TMS_SWDIO));
        }
        if select.contains(SwjPins::N_RESET) {
            if *self.reset_dir == PinDir::Input {
                self.reset
                    .set_mode_output(output.contains(SwjPins::N_RESET));
                *self.reset_dir = PinDir::Output;
            } else {
                self.reset.write(output.contains(SwjPins::N_RESET));
            }
        }

        let wait_us = wait_us.min(3_000_000);
        let cycles = (config.core_clock_hz as u64 * wait_us as u64 / 1_000_000) as u32;
        self.delay.delay_cycles(cycles);

        let clk_was_output = *self.clk_dir == PinDir::Output;
        let dio_was_output = *self.dio_dir == PinDir::Output;
        let reset_was_output = *self.reset_dir == PinDir::Output;
        if clk_was_output {
            self.clk.set_mode_input();
        }
        if dio_was_output {
            self.dio.set_mode_input();
        }
        if reset_was_output {
            self.reset.set_mode_input();
        }

        let mut input = SwjPins::empty();
        if self.clk.read() {
            input |= SwjPins::TCK_SWDCLK;
        }
        if self.dio.read() {
            input |= SwjPins::TMS_SWDIO;
        }
        if self.reset.read() {
            input |= SwjPins::N_RESET;
        }

        if clk_was_output {
            self.clk.set_mode_output(false);
        }
        if dio_was_output {
            self.dio.set_mode_output(false);
        }
        if reset_was_output {
            self.reset.set_mode_output(true);
        }
        Ok(input)
    }

    fn read_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &mut [u8],
    ) -> Result<(), DapError> {
        pack_read_bits(count, data, || self.read_bit(config))
    }

    fn write_bits(
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

    fn output_enable(&mut self, enable: bool) {
        if enable {
            self.dio_to_output(false);
        } else {
            self.dio_to_input();
        }
    }

    fn transfer(
        &mut self,
        config: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        // Request phase.
        self.dio_to_output(false);
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
        self.dio_to_input();
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
                self.dio_to_output(false);
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // Write data phase.
                self.turn_around(config);
                self.dio_to_output(false);
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
            self.dio.write(true);
            return result;
        }

        if ack == DAP_TRANSFER_WAIT || ack == DAP_TRANSFER_FAULT {
            self.dio_to_input();
            if config.swd.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.dio_to_output(false);
            if config.swd.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.dio.write(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error.
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.dio_to_output(false);
        self.dio.write(true);
        Err(DapError::SwdError(ack))
    }
}

////////////////////////////////////////////////////////////////////////////
// JTAG bit-level algorithm (shared by BitBangJtag and BitBangSwj)
////////////////////////////////////////////////////////////////////////////

/// Borrows the six JTAG pins so the JTAG algorithm can be shared between the
/// JTAG-only and combined transports.
struct JtagBits<'a, Tck, Tms, Tdi, Tdo, Trst, Srst, D>
where
    Tck: BidirPin,
    Tms: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    tck: &'a mut Tck,
    tms: &'a mut Tms,
    tdi: &'a mut Tdi,
    tdo: &'a mut Tdo,
    ntrst: &'a mut Trst,
    nsrst: &'a mut Srst,
    delay: &'a D,
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst, D> JtagBits<'_, Tck, Tms, Tdi, Tdo, Trst, Srst, D>
where
    Tck: BidirPin,
    Tms: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    fn clock_wait(&self, config: &DapConfig) {
        self.delay.delay_cycles(config.jtag.clock_wait_cycles);
    }
    fn write_bit(&mut self, config: &DapConfig, tms: bool, tdi: bool) {
        self.tms.write(tms);
        self.tdi.write(tdi);
        self.tck.write(false);
        self.clock_wait(config);
        self.tck.write(true);
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &DapConfig, tms: bool, tdi: bool) -> bool {
        self.tms.write(tms);
        self.tdi.write(tdi);
        self.tck.write(false);
        self.clock_wait(config);
        let value = self.tdo.read();
        self.tck.write(true);
        self.clock_wait(config);
        value
    }

    fn connect(&mut self, config: &DapConfig) {
        self.tck.set_mode_output(false);
        self.tms.set_mode_output(false);
        self.tdi.set_mode_output(false);
        self.tdo.set_mode_input();
        self.ntrst.set_mode_output(true);
        self.nsrst.set_mode_output(true);

        // Reset the TAPs with nTRST.
        self.ntrst.write(false);
        self.clock_wait(config);
        self.ntrst.write(true);
        self.clock_wait(config);
        // Reset the JTAG state machine.
        for _ in 0..10 {
            self.write_bit(config, true, false);
        }
    }

    fn reset_state_machine(&mut self, config: &DapConfig) {
        for _ in 0..10 {
            self.write_bit(config, true, false);
        }
    }

    fn release(&mut self) {
        self.tck.set_mode_input();
        self.tms.set_mode_input();
        self.tdi.set_mode_input();
        self.tdo.set_mode_input();
        self.ntrst.set_mode_input();
        self.nsrst.set_mode_input();
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
            self.write_bit(config, value & 1 != 0, false);
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
        if select.contains(SwjPins::TCK_SWDCLK) {
            self.tck.write(output.contains(SwjPins::TCK_SWDCLK));
        }
        if select.contains(SwjPins::TMS_SWDIO) {
            self.tms.write(output.contains(SwjPins::TMS_SWDIO));
        }
        if select.contains(SwjPins::TDI) {
            self.tdi.write(output.contains(SwjPins::TDI));
        }
        if select.contains(SwjPins::N_TRST) {
            self.ntrst.write(output.contains(SwjPins::N_TRST));
        }
        if select.contains(SwjPins::N_RESET) {
            self.nsrst.write(output.contains(SwjPins::N_RESET));
        }

        let wait_us = wait_us.min(3_000_000);
        let cycles = (config.core_clock_hz as u64 * wait_us as u64 / 1_000_000) as u32;
        self.delay.delay_cycles(cycles);

        let mut input = SwjPins::empty();
        if self.tdo.read() {
            input |= SwjPins::TDO;
        }
        Ok(input)
    }

    fn write_ir(&mut self, config: &DapConfig, dap_index: u8, ir: u32) {
        // to ShiftIR from Run-Test-Idle
        self.write_bit(config, true, false); // SelectDR
        self.write_bit(config, true, false); // SelectIR
        self.write_bit(config, false, false); // CaptureIR
        self.write_bit(config, false, false); // ShiftIR

        let device_count = config.jtag.device_count as usize;
        let ir_length = &config.jtag.ir_length[0..device_count];
        let bit_count_max: usize = ir_length.iter().map(|x| *x as usize).sum();
        let mut bit_count: usize = 0;
        let mut ir = ir;
        for (i, length) in ir_length.iter().enumerate().rev() {
            for _ in 0..*length {
                let tdi = if i == (dap_index as usize) {
                    let bit = (ir & 1) != 0;
                    ir >>= 1;
                    bit
                } else {
                    true
                };
                bit_count += 1;
                let tms = bit_count == bit_count_max;
                self.write_bit(config, tms, tdi);
            }
        }

        self.write_bit(config, true, false); // Update-IR
        self.write_bit(config, false, false); // Run
    }

    fn read_write_dr(&mut self, config: &DapConfig, dap_index: u8, dr: &mut BitSlice<u32>) {
        // to ShiftDR from Run-Test-Idle
        self.write_bit(config, true, false); // SelectDR
        self.write_bit(config, false, false); // CaptureDR
        self.write_bit(config, false, false); // ShiftDR

        let device_count = config.jtag.device_count as usize;
        let ir_length = &config.jtag.ir_length[0..device_count];
        let head_bits: usize = ir_length
            .iter()
            .take(dap_index.into())
            .map(|x| *x as usize)
            .sum();
        let tail_bits: usize = ir_length
            .iter()
            .skip(dap_index as usize + 1)
            .map(|x| *x as usize)
            .sum();
        let total_bits: usize = tail_bits + dr.len() + head_bits;
        let mut bit_count: usize = 0;
        for _ in 0..tail_bits {
            bit_count += 1;
            self.write_bit(config, false, false);
        }
        for mut dr_bit in dr {
            bit_count += 1;
            let tms = bit_count == total_bits;
            let tdo = self.read_bit(config, tms, *dr_bit);
            dr_bit.set(tdo);
        }
        for _ in 0..head_bits {
            bit_count += 1;
            let tms = bit_count == total_bits;
            self.write_bit(config, tms, false);
        }

        self.write_bit(config, true, false); // Update-DR
        self.write_bit(config, false, false); // Run
    }

    fn transfer(
        &mut self,
        config: &DapConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        let instruction = if request.contains(SwdRequest::APnDP) {
            JtagInstruction::APACC as u32
        } else {
            JtagInstruction::DPACC as u32
        };
        let read = request.contains(SwdRequest::RnW);
        let a2 = request.contains(SwdRequest::A2);
        let a3 = request.contains(SwdRequest::A3);

        self.write_ir(config, dap_index, instruction);
        if read {
            let mut dr = build_acc(data, a3, a2, true);
            self.read_write_dr(config, dap_index, dr.as_mut_bitslice());
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

    fn sequence(
        &mut self,
        config: &DapConfig,
        info: &JtagSequenceInfo,
        tdi_data: u64,
    ) -> Result<Option<u64>, DapError> {
        if info.number_of_tck_cycles > 64 {
            return Err(DapError::InvalidCommand);
        }
        Ok(if info.tdo_capture {
            let mut tdo_data = 0u64;
            for i in 0..info.number_of_tck_cycles {
                let tdo = self.read_bit(config, info.tms_value, (tdi_data & (1 << i)) != 0);
                tdo_data |= if tdo { 1 } else { 0 } << i;
            }
            Some(tdo_data)
        } else {
            for i in 0..info.number_of_tck_cycles {
                self.write_bit(config, info.tms_value, (tdi_data & (1 << i)) != 0);
            }
            None
        })
    }
}

////////////////////////////////////////////////////////////////////////////
// SWD-only transport
////////////////////////////////////////////////////////////////////////////

pub struct BitBangSwd<Clk: BidirPin, Dio: BidirPin, Rst: BidirPin, D: Delay> {
    swclk: Clk,
    swdio: Dio,
    reset: Rst,
    delay: D,
    swclk_dir: PinDir,
    swdio_dir: PinDir,
    reset_dir: PinDir,
}

impl<Clk: BidirPin, Dio: BidirPin, Rst: BidirPin, D: Delay> BitBangSwd<Clk, Dio, Rst, D> {
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

    fn swd(&mut self) -> SwdBits<'_, Clk, Dio, Rst, D> {
        SwdBits {
            clk: &mut self.swclk,
            dio: &mut self.swdio,
            reset: &mut self.reset,
            delay: &self.delay,
            clk_dir: &mut self.swclk_dir,
            dio_dir: &mut self.swdio_dir,
            reset_dir: &mut self.reset_dir,
        }
    }
}

impl<Clk: BidirPin, Dio: BidirPin, Rst: BidirPin, D: Delay> DapTransport
    for BitBangSwd<Clk, Dio, Rst, D>
{
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::SWD
    }
    fn connect(&mut self, port: ConnectPort, _config: &DapConfig) -> Result<ActivePort, DapError> {
        match port {
            ConnectPort::Default | ConnectPort::Swd => {
                self.swd().connect();
                Ok(ActivePort::Swd)
            }
            ConnectPort::Jtag => Err(DapError::NotSupported),
        }
    }
    fn disconnect(&mut self, _config: &DapConfig) -> Result<(), DapError> {
        self.swd().disconnect();
        Ok(())
    }
    fn swj_sequence(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        self.swd().swj_sequence(config, count, data)
    }
    fn swj_pins(
        &mut self,
        config: &DapConfig,
        output: SwjPins,
        select: SwjPins,
        wait_us: u32,
    ) -> Result<SwjPins, DapError> {
        self.swd().swj_pins(config, output, select, wait_us)
    }
    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
        swj_clock_cycles(config, frequency_hz, false)
    }
    fn swd_read_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &mut [u8],
    ) -> Result<(), DapError> {
        self.swd().read_bits(config, count, data)
    }
    fn swd_write_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        self.swd().write_bits(config, count, data)
    }
    fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> {
        self.swd().output_enable(enable);
        Ok(())
    }
    fn swd_transfer(
        &mut self,
        config: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        self.swd().transfer(config, request, data)
    }
}

////////////////////////////////////////////////////////////////////////////
// JTAG-only transport
////////////////////////////////////////////////////////////////////////////

pub struct BitBangJtag<Tck, Tms, Tdi, Tdo, Trst, Srst, D>
where
    Tck: BidirPin,
    Tms: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    tck: Tck,
    tms: Tms,
    tdi: Tdi,
    tdo: Tdo,
    ntrst: Trst,
    nsrst: Srst,
    delay: D,
    pins_connected: bool,
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst, D> BitBangJtag<Tck, Tms, Tdi, Tdo, Trst, Srst, D>
where
    Tck: BidirPin,
    Tms: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    /// Creates the transport. All pins must be in input mode.
    pub fn new(tck: Tck, tms: Tms, tdi: Tdi, tdo: Tdo, ntrst: Trst, nsrst: Srst, delay: D) -> Self {
        Self {
            tck,
            tms,
            tdi,
            tdo,
            ntrst,
            nsrst,
            delay,
            pins_connected: false,
        }
    }

    fn jtag(&mut self) -> JtagBits<'_, Tck, Tms, Tdi, Tdo, Trst, Srst, D> {
        JtagBits {
            tck: &mut self.tck,
            tms: &mut self.tms,
            tdi: &mut self.tdi,
            tdo: &mut self.tdo,
            ntrst: &mut self.ntrst,
            nsrst: &mut self.nsrst,
            delay: &self.delay,
        }
    }
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst, D> DapTransport
    for BitBangJtag<Tck, Tms, Tdi, Tdo, Trst, Srst, D>
where
    Tck: BidirPin,
    Tms: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::JTAG
    }
    fn connect(&mut self, port: ConnectPort, config: &DapConfig) -> Result<ActivePort, DapError> {
        match port {
            ConnectPort::Default | ConnectPort::Jtag => {
                self.jtag().connect(config);
                self.pins_connected = true;
                Ok(ActivePort::Jtag)
            }
            ConnectPort::Swd => Err(DapError::NotSupported),
        }
    }
    fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError> {
        let connected = self.pins_connected;
        let mut jtag = self.jtag();
        if connected {
            jtag.reset_state_machine(config);
        }
        jtag.release();
        self.pins_connected = false;
        Ok(())
    }
    fn swj_sequence(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        self.jtag().swj_sequence(config, count, data)
    }
    fn swj_pins(
        &mut self,
        config: &DapConfig,
        output: SwjPins,
        select: SwjPins,
        wait_us: u32,
    ) -> Result<SwjPins, DapError> {
        self.jtag().swj_pins(config, output, select, wait_us)
    }
    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
        swj_clock_cycles(config, frequency_hz, true)
    }
    fn jtag_transfer(
        &mut self,
        config: &DapConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        self.jtag().transfer(config, dap_index, request, data)
    }
    fn jtag_sequence(
        &mut self,
        config: &DapConfig,
        info: &JtagSequenceInfo,
        tdi_data: u64,
    ) -> Result<Option<u64>, DapError> {
        self.jtag().sequence(config, info, tdi_data)
    }
}

////////////////////////////////////////////////////////////////////////////
// Combined SWD+JTAG transport with runtime port switching
////////////////////////////////////////////////////////////////////////////

/// Bit-banging transport that supports both SWD and JTAG over one shared set
/// of pins, selected at runtime by `DAP_Connect(port)`.
///
/// SWCLK and TCK share `clk`, SWDIO and TMS share `dio`, and RESET and nSRST
/// share `nsrst`; `tdi`, `tdo` and `ntrst` are JTAG-only. This is what a
/// standard 10-pin Cortex debug connector needs, and is only expressible now
/// that a single type can implement both protocols.
pub struct BitBangSwj<Clk, Dio, Tdi, Tdo, Trst, Srst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    clk: Clk,
    dio: Dio,
    tdi: Tdi,
    tdo: Tdo,
    ntrst: Trst,
    nsrst: Srst,
    delay: D,
    clk_dir: PinDir,
    dio_dir: PinDir,
    nsrst_dir: PinDir,
    active: Option<ActivePort>,
}

impl<Clk, Dio, Tdi, Tdo, Trst, Srst, D> BitBangSwj<Clk, Dio, Tdi, Tdo, Trst, Srst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    /// Creates the transport. All pins must be in input mode.
    /// `clk`=SWCLK/TCK, `dio`=SWDIO/TMS, `nsrst`=RESET/nSRST are shared;
    /// `tdi`/`tdo`/`ntrst` are used only in JTAG mode.
    #[allow(clippy::too_many_arguments)]
    pub fn new(clk: Clk, dio: Dio, tdi: Tdi, tdo: Tdo, ntrst: Trst, nsrst: Srst, delay: D) -> Self {
        Self {
            clk,
            dio,
            tdi,
            tdo,
            ntrst,
            nsrst,
            delay,
            clk_dir: PinDir::Input,
            dio_dir: PinDir::Input,
            nsrst_dir: PinDir::Input,
            active: None,
        }
    }

    fn swd(&mut self) -> SwdBits<'_, Clk, Dio, Srst, D> {
        SwdBits {
            clk: &mut self.clk,
            dio: &mut self.dio,
            reset: &mut self.nsrst,
            delay: &self.delay,
            clk_dir: &mut self.clk_dir,
            dio_dir: &mut self.dio_dir,
            reset_dir: &mut self.nsrst_dir,
        }
    }

    fn jtag(&mut self) -> JtagBits<'_, Clk, Dio, Tdi, Tdo, Trst, Srst, D> {
        JtagBits {
            tck: &mut self.clk,
            tms: &mut self.dio,
            tdi: &mut self.tdi,
            tdo: &mut self.tdo,
            ntrst: &mut self.ntrst,
            nsrst: &mut self.nsrst,
            delay: &self.delay,
        }
    }
}

impl<Clk, Dio, Tdi, Tdo, Trst, Srst, D> DapTransport
    for BitBangSwj<Clk, Dio, Tdi, Tdo, Trst, Srst, D>
where
    Clk: BidirPin,
    Dio: BidirPin,
    Tdi: BidirPin,
    Tdo: BidirPin,
    Trst: BidirPin,
    Srst: BidirPin,
    D: Delay,
{
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::SWD | DapCapabilities::JTAG
    }

    fn connect(&mut self, port: ConnectPort, config: &DapConfig) -> Result<ActivePort, DapError> {
        // Release whatever was previously configured so a switch between
        // protocols starts from a clean, all-input state.
        if self.active.is_some() {
            self.disconnect(config).ok();
        }
        let active = match port {
            ConnectPort::Default | ConnectPort::Swd => {
                self.swd().connect();
                ActivePort::Swd
            }
            ConnectPort::Jtag => {
                self.jtag().connect(config);
                ActivePort::Jtag
            }
        };
        self.active = Some(active);
        Ok(active)
    }

    fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError> {
        match self.active {
            Some(ActivePort::Swd) => self.swd().disconnect(),
            Some(ActivePort::Jtag) => {
                let mut jtag = self.jtag();
                jtag.reset_state_machine(config);
                jtag.release();
            }
            None => {}
        }
        // Ensure every pin is released, including the JTAG-only ones.
        self.tdi.set_mode_input();
        self.tdo.set_mode_input();
        self.ntrst.set_mode_input();
        self.active = None;
        Ok(())
    }

    fn swj_sequence(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        match self.active {
            Some(ActivePort::Jtag) => self.jtag().swj_sequence(config, count, data),
            _ => self.swd().swj_sequence(config, count, data),
        }
    }

    fn swj_pins(
        &mut self,
        config: &DapConfig,
        output: SwjPins,
        select: SwjPins,
        wait_us: u32,
    ) -> Result<SwjPins, DapError> {
        match self.active {
            Some(ActivePort::Jtag) => self.jtag().swj_pins(config, output, select, wait_us),
            _ => self.swd().swj_pins(config, output, select, wait_us),
        }
    }

    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
        swj_clock_cycles(config, frequency_hz, self.active == Some(ActivePort::Jtag))
    }

    fn swd_read_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &mut [u8],
    ) -> Result<(), DapError> {
        self.swd().read_bits(config, count, data)
    }
    fn swd_write_bits(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        self.swd().write_bits(config, count, data)
    }
    fn swd_output_enable(&mut self, enable: bool) -> Result<(), DapError> {
        self.swd().output_enable(enable);
        Ok(())
    }
    fn swd_transfer(
        &mut self,
        config: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        self.swd().transfer(config, request, data)
    }

    fn jtag_transfer(
        &mut self,
        config: &DapConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        self.jtag().transfer(config, dap_index, request, data)
    }
    fn jtag_sequence(
        &mut self,
        config: &DapConfig,
        info: &JtagSequenceInfo,
        tdi_data: u64,
    ) -> Result<Option<u64>, DapError> {
        self.jtag().sequence(config, info, tdi_data)
    }
}
