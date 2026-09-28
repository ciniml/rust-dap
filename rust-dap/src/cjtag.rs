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

//! Bit-banged 2-wire cJTAG (IEEE 1149.7, OScan1 scan format) transport.
//!
//! cJTAG carries the four JTAG signals over two pins, TCKC and TMSC. After an
//! activation handshake (escape sequences + a 12-bit online activation code)
//! the link runs in the OScan1 format, where every JTAG TCK cycle becomes
//! three TCKC periods: the probe drives nTDI, then TMS, then releases TMSC so
//! the target can drive TDO. Everything above that cycle — IR/DR scans, the
//! DPACC/APACC transfers, raw sequences — is the shared [`JtagScan`] code, so
//! a cJTAG target looks like any JTAG-DP to `arm-debug`.
//!
//! Written for the TI SimpleLink CC13x2/CC26x2, whose debug port comes up in
//! 2-pin cJTAG and offers no SWD. The activation recipe follows OpenOCD's
//! `cjtag_reset_online_activate` (ftdi.c) and TI TRM SWCU185 §6.2.

use crate::bitbang::{swj_clock_cycles, BidirPin, JtagBitDriver, JtagScan};
use crate::cmsis_dap::{DapCapabilities, DapError, JtagSequenceInfo, SwdRequest, SwjPins};
use crate::transport::{ActivePort, ConnectPort, DapConfig, DapTransport, Delay};

/// TMSC edges (while TCKC is high) that reset the cJTAG adapter to offline.
const ESCAPE_RESET_EDGES: u32 = 8;
/// TMSC edges that select the adapter (make it listen for an activation code).
const ESCAPE_SELECT_EDGES: u32 = 6;
/// Online Activation Code for a 2-wire (OScan1) link, sent LSB first.
const OAC_OSCAN1: u8 = 0b1100;
/// Extension Code: no extensions, sent LSB first.
const EC_NONE: u8 = 0b1000;
/// Check Packet. OpenOCD sends all zeros and TI parts accept it; the strict
/// IEEE 1149.7 value would be `OAC ^ EC` (0b0100). Change here if a target
/// refuses activation.
const CP: u8 = 0b0000;

#[derive(Clone, Copy, PartialEq, Eq)]
enum Dir {
    Input,
    Output,
}

/// Bit-banging OScan1 transport over TCKC, TMSC and an nRESET pin.
pub struct BitBangOscan1<Tckc: BidirPin, Tmsc: BidirPin, Srst: BidirPin, D: Delay> {
    tckc: Tckc,
    tmsc: Tmsc,
    nsrst: Srst,
    delay: D,
    tmsc_dir: Dir,
    pins_connected: bool,
}

impl<Tckc: BidirPin, Tmsc: BidirPin, Srst: BidirPin, D: Delay> BitBangOscan1<Tckc, Tmsc, Srst, D> {
    /// Creates the transport. All pins must be in input mode.
    pub fn new(tckc: Tckc, tmsc: Tmsc, nsrst: Srst, delay: D) -> Self {
        Self {
            tckc,
            tmsc,
            nsrst,
            delay,
            tmsc_dir: Dir::Input,
            pins_connected: false,
        }
    }

    fn wait(&self, config: &DapConfig) {
        self.delay.delay_cycles(config.jtag.clock_wait_cycles);
    }

    /// Drive TMSC, switching it to output first if the target had it.
    fn drive(&mut self, high: bool) {
        if self.tmsc_dir == Dir::Input {
            self.tmsc.set_mode_output(high);
            self.tmsc_dir = Dir::Output;
        } else {
            self.tmsc.write(high);
        }
    }

    /// Release TMSC to the target.
    fn release_tmsc(&mut self) {
        if self.tmsc_dir == Dir::Output {
            self.tmsc.set_mode_input();
            self.tmsc_dir = Dir::Input;
        }
    }

    /// One TCKC period with TMSC driven to `level` (level applied after the
    /// falling edge, sampled by the target on the rising edge).
    fn drive_cycle(&mut self, config: &DapConfig, level: bool) {
        self.tckc.write(false);
        self.drive(level);
        self.wait(config);
        self.tckc.write(true);
        self.wait(config);
    }

    /// An escape sequence: `edges` TMSC transitions while TCKC is held high,
    /// starting and ending with TMSC low.
    fn escape(&mut self, config: &DapConfig, edges: u32) {
        self.tckc.write(false);
        self.drive(false);
        self.wait(config);
        self.tckc.write(true);
        self.wait(config);
        let mut level = false;
        for _ in 0..edges {
            level = !level;
            self.drive(level);
            self.wait(config);
        }
        if level {
            // odd edge count: bring TMSC back low without counting as an edge
            // that matters (the adapter classifies by count, 8+ = reset)
            self.drive(false);
            self.wait(config);
        }
        self.tckc.write(false);
        self.wait(config);
    }

    /// Reset the cJTAG adapter and activate the 2-wire OScan1 link:
    /// reset escape, select escape, then OAC/EC/CP on successive TCKC
    /// rising edges. Leaves the target's TAP in Test-Logic-Reset.
    pub fn activate(&mut self, config: &DapConfig) {
        self.tckc.set_mode_output(false);
        self.drive(false);
        self.wait(config);
        // Some parts (TI ICEMelter) only wake their JTAG domain after seeing
        // TCKC toggle; a few idle periods first cost nothing.
        for _ in 0..8 {
            self.drive_cycle(config, false);
        }
        self.escape(config, ESCAPE_RESET_EDGES);
        for _ in 0..3 {
            self.drive_cycle(config, false);
        }
        self.escape(config, ESCAPE_SELECT_EDGES);
        for code in [OAC_OSCAN1, EC_NONE, CP] {
            for bit in 0..4 {
                self.drive_cycle(config, (code >> bit) & 1 != 0);
            }
        }
        self.tckc.write(false);
        self.drive(false);
        self.wait(config);
        // Now in OScan1: put the TAP into Test-Logic-Reset.
        for _ in 0..10 {
            self.write_bit(config, true, false);
        }
    }

    fn release_pins(&mut self) {
        self.tckc.set_mode_input();
        self.tmsc.set_mode_input();
        self.tmsc_dir = Dir::Input;
        self.nsrst.set_mode_input();
    }
}

impl<Tckc: BidirPin, Tmsc: BidirPin, Srst: BidirPin, D: Delay> JtagBitDriver
    for BitBangOscan1<Tckc, Tmsc, Srst, D>
{
    /// One JTAG cycle = three TCKC periods: nTDI, TMS (probe drives TMSC),
    /// then TDO (target drives TMSC; released by the probe beforehand).
    fn write_bit(&mut self, config: &DapConfig, tms: bool, tdi: bool) {
        self.drive_cycle(config, !tdi);
        self.drive_cycle(config, tms);
        self.release_tmsc();
        self.tckc.write(false);
        self.wait(config);
        self.tckc.write(true);
        self.wait(config);
    }

    fn read_bit(&mut self, config: &DapConfig, tms: bool, tdi: bool) -> bool {
        self.drive_cycle(config, !tdi);
        self.drive_cycle(config, tms);
        self.release_tmsc();
        self.tckc.write(false);
        self.wait(config);
        let value = self.tmsc.read();
        self.tckc.write(true);
        self.wait(config);
        value
    }
}

impl<Tckc: BidirPin, Tmsc: BidirPin, Srst: BidirPin, D: Delay> DapTransport
    for BitBangOscan1<Tckc, Tmsc, Srst, D>
{
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::JTAG
    }

    fn connect(&mut self, port: ConnectPort, config: &DapConfig) -> Result<ActivePort, DapError> {
        match port {
            ConnectPort::Default | ConnectPort::Jtag => {
                self.nsrst.set_mode_output(true);
                self.activate(config);
                self.pins_connected = true;
                Ok(ActivePort::Jtag)
            }
            ConnectPort::Swd => Err(DapError::NotSupported),
        }
    }

    fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError> {
        if self.pins_connected {
            self.reset_state_machine(config);
        }
        self.release_pins();
        self.pins_connected = false;
        Ok(())
    }

    fn swj_sequence(
        &mut self,
        config: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        JtagScan::swj_sequence(self, config, count, data)
    }

    fn swj_pins(
        &mut self,
        config: &DapConfig,
        output: SwjPins,
        select: SwjPins,
        wait_us: u32,
    ) -> Result<SwjPins, DapError> {
        if select.contains(SwjPins::TCK_SWDCLK) {
            self.tckc.write(output.contains(SwjPins::TCK_SWDCLK));
        }
        if select.contains(SwjPins::TMS_SWDIO) {
            self.drive(output.contains(SwjPins::TMS_SWDIO));
        }
        if select.contains(SwjPins::N_RESET) {
            self.nsrst.write(output.contains(SwjPins::N_RESET));
        }
        let wait_us = wait_us.min(3_000_000);
        let cycles = (config.core_clock_hz as u64 * wait_us as u64 / 1_000_000) as u32;
        self.delay.delay_cycles(cycles);
        let mut input = SwjPins::empty();
        if self.nsrst.read() {
            input |= SwjPins::N_RESET;
        }
        Ok(input)
    }

    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32) -> Result<(), DapError> {
        // frequency_hz is the TCKC rate; one JTAG bit takes three periods.
        swj_clock_cycles(config, frequency_hz, true)
    }

    fn jtag_transfer(
        &mut self,
        config: &DapConfig,
        dap_index: u8,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        self.transfer(config, dap_index, request, data)
    }

    fn jtag_sequence(
        &mut self,
        config: &DapConfig,
        info: &JtagSequenceInfo,
        tdi_data: u64,
    ) -> Result<Option<u64>, DapError> {
        self.sequence(config, info, tdi_data)
    }
}
