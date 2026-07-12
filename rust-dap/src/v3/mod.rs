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

//! Next generation CMSIS-DAP architecture (see doc/redesign-proposal.ja.md).
//!
//! Design points compared to the legacy trait stack:
//! - Command parsing and protocol policy (retries, posted reads, match
//!   value/mask) live entirely in [`Dispatcher`], which is concrete core
//!   code, not a trait.
//! - The only implementation point is [`DapTransport`]. SWD and JTAG
//!   operations are fine grained methods with `Err(NotSupported)` defaults,
//!   so one type can implement SWD only, JTAG only, or both, and
//!   `DAP_Connect(port)` switches between them at runtime.
//! - No blanket impls, so downstream crates can override any subset.

pub mod bitbang;
mod dispatcher;
mod usb;

pub use dispatcher::Dispatcher;
pub use usb::CmsisDap;

pub use crate::cmsis_dap::{
    DapCapabilities, DapError, JtagSequenceInfo, SwdRequest, SwjPins,
};

/// Maximum number of devices on a JTAG scan chain supported by [`JtagConfig`].
pub const MAX_JTAG_DEVICES: usize = 8;

/// Port requested by the host in DAP_Connect.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ConnectPort {
    Default,
    Swd,
    Jtag,
}

/// Port actually selected by the transport.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ActivePort {
    Swd,
    Jtag,
}

/// Identification strings and constants reported by DAP_Info.
#[derive(Clone, Copy)]
pub struct DapIdentity {
    pub vendor: &'static str,
    pub product: &'static str,
    pub serial_number: &'static str,
    pub firmware_version: &'static str,
    pub packet_count: u8,
}

impl Default for DapIdentity {
    fn default() -> Self {
        Self {
            vendor: "rust-dap",
            product: "CMSIS-DAP",
            serial_number: "rust-dap",
            firmware_version: "2.00",
            packet_count: 1,
        }
    }
}

#[derive(Clone, Copy)]
pub struct SwdConfig {
    pub clock_wait_cycles: u32,
    pub idle_cycles: u32,
    pub turn_around_cycles: u32,
    pub always_generate_data_phase: bool,
}

impl Default for SwdConfig {
    fn default() -> Self {
        Self {
            clock_wait_cycles: 1,
            idle_cycles: 0,
            turn_around_cycles: 1,
            always_generate_data_phase: false,
        }
    }
}

#[derive(Clone, Copy)]
pub struct JtagConfig {
    pub clock_wait_cycles: u32,
    pub device_count: u8,
    pub ir_length: [u8; MAX_JTAG_DEVICES],
    pub idle_cycles: u32,
}

impl Default for JtagConfig {
    fn default() -> Self {
        Self {
            clock_wait_cycles: 1000,
            device_count: 0,
            ir_length: [0; MAX_JTAG_DEVICES],
            idle_cycles: 0,
        }
    }
}

/// All runtime configuration of the DAP, injected via [`CmsisDap::new`].
pub struct DapConfig {
    pub identity: DapIdentity,
    pub swd: SwdConfig,
    pub jtag: JtagConfig,
    pub retry_count: u32,
    pub match_mask: u32,
    pub match_retry_count: u32,
    /// CPU core clock of the probe. Used to convert time based waits
    /// (e.g. DAP_SWJ_Pins wait_us) and target clock frequencies into
    /// delay cycles.
    pub core_clock_hz: u32,
}

impl Default for DapConfig {
    fn default() -> Self {
        Self {
            identity: DapIdentity::default(),
            swd: SwdConfig::default(),
            jtag: JtagConfig::default(),
            retry_count: 5,
            match_mask: 0xffff_ffff,
            match_retry_count: 5,
            core_clock_hz: 125_000_000,
        }
    }
}

impl DapConfig {
    pub fn new(identity: DapIdentity, core_clock_hz: u32) -> Self {
        Self {
            identity,
            core_clock_hz,
            ..Self::default()
        }
    }
}

/// Busy-wait delay used by bit-banging transports.
pub trait Delay {
    fn delay_cycles(&self, cycles: u32);
}

/// Physical layer abstraction and single implementation point of the v3
/// architecture.
///
/// SWD and JTAG specific methods have `Err(NotSupported)` default
/// implementations; a transport implements the subsets it supports and
/// reports them via [`DapTransport::capabilities`]. All byte level command
/// parsing is done by [`Dispatcher`]; transports only receive parsed values.
pub trait DapTransport {
    /// Features actually supported by this transport, reported via DAP_Info.
    fn capabilities(&self) -> DapCapabilities;

    /// DAP_Connect: reconfigure the pins for `port` and return the port
    /// actually selected.
    fn connect(&mut self, port: ConnectPort, config: &DapConfig)
        -> Result<ActivePort, DapError>;

    /// DAP_Disconnect: release the pins (high impedance).
    fn disconnect(&mut self, config: &DapConfig) -> Result<(), DapError>;

    // ---- SWJ (common, required) ----

    /// Output `count` bits of `data` on SWDIO/TMS with SWCLK/TCK clocking.
    fn swj_sequence(&mut self, config: &DapConfig, count: usize, data: &[u8])
        -> Result<(), DapError>;

    /// Drive/read the SWJ pins. `select` chooses which pins to drive with
    /// `output`; afterwards waits `wait_us` and returns the pin input state.
    fn swj_pins(&mut self, config: &DapConfig, output: SwjPins, select: SwjPins, wait_us: u32)
        -> Result<SwjPins, DapError>;

    /// DAP_SWJ_Clock: update the transport clock. Implementations update
    /// `config.swd`/`config.jtag` timing fields as appropriate.
    fn swj_clock(&mut self, config: &mut DapConfig, frequency_hz: u32)
        -> Result<(), DapError>;

    // ---- SWD ----

    /// Single SWD transfer (request phase, ack, data phase). No retries;
    /// retry policy is applied by the dispatcher.
    fn swd_transfer(
        &mut self,
        _config: &DapConfig,
        _request: SwdRequest,
        _data: u32,
    ) -> Result<u32, DapError> {
        Err(DapError::NotSupported)
    }

    /// Read `count` bits from SWDIO into `data` (LSB first, packed).
    fn swd_read_bits(
        &mut self,
        _config: &DapConfig,
        _count: usize,
        _data: &mut [u8],
    ) -> Result<(), DapError> {
        Err(DapError::NotSupported)
    }

    /// Write `count` bits from `data` to SWDIO (LSB first, packed).
    fn swd_write_bits(
        &mut self,
        _config: &DapConfig,
        _count: usize,
        _data: &[u8],
    ) -> Result<(), DapError> {
        Err(DapError::NotSupported)
    }

    /// Enable or disable the SWDIO output driver.
    fn swd_output_enable(&mut self, _enable: bool) -> Result<(), DapError> {
        Err(DapError::NotSupported)
    }

    // ---- JTAG ----

    /// Single DPACC/APACC transfer over JTAG. No retries.
    fn jtag_transfer(
        &mut self,
        _config: &DapConfig,
        _dap_index: u8,
        _request: SwdRequest,
        _data: u32,
    ) -> Result<u32, DapError> {
        Err(DapError::NotSupported)
    }

    /// Execute one parsed JTAG sequence. Returns captured TDO data when
    /// `info.tdo_capture` is set.
    fn jtag_sequence(
        &mut self,
        _config: &DapConfig,
        _info: &JtagSequenceInfo,
        _tdi_data: u64,
    ) -> Result<Option<u64>, DapError> {
        Err(DapError::NotSupported)
    }

    /// Read the IDCODE of the device at `index` on the scan chain.
    fn jtag_idcode(&mut self, _config: &DapConfig, _index: u8) -> Result<u32, DapError> {
        Err(DapError::NotSupported)
    }
}
