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

//! ARM Debug Interface v5 (ADIv5) access layer built on a rust-dap
//! [`DapTransport`]. This is milestone M1 of the standalone GDB debugger
//! (see `doc/gdb-debugger-proposal.ja.md`): SWD connect, DP power-up and
//! MEM-AP memory read/write. The Cortex-M core-debug layer (halt/step,
//! registers, breakpoints) will be layered on top in later milestones.
//!
//! Everything is generic over `DapTransport`, so the same code drives the
//! bit-banging and PIO SWD transports.

#![cfg_attr(not(test), no_std)]

use rust_dap::{
    ActivePort, ConnectPort, DapConfig, DapError, DapTransport, SwdRequest, DAP_TRANSFER_WAIT,
};

/// Errors from the ARM debug layer.
#[derive(Debug, PartialEq, Eq)]
pub enum ArmError {
    /// The underlying SWD transfer failed (carries the DAP transfer ACK, or a
    /// mapped code for non-ACK errors).
    Transfer(u8),
    /// A WAIT ACK persisted past the retry budget.
    Wait,
    /// The transport does not support the operation (e.g. no SWD).
    NotSupported,
    /// Debug power-up was requested but never acknowledged.
    PowerUpTimeout,
    /// The connected target's DPIDR/IDCODE did not look valid.
    NoTarget,
    /// Internal invariant violated.
    Internal,
}

impl From<DapError> for ArmError {
    fn from(e: DapError) -> Self {
        match e {
            DapError::SwdError(ack) => ArmError::Transfer(ack),
            DapError::NotSupported => ArmError::NotSupported,
            _ => ArmError::Internal,
        }
    }
}

/// ADIv5 access port kind for a raw register access.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Port {
    Dp,
    Ap,
}

// --- Debug Port (DP) register addresses (A[3:2], i.e. offset 0x0/0x4/0x8/0xC).
const DP_DPIDR: u8 = 0x0; // read
const DP_ABORT: u8 = 0x0; // write
const DP_CTRL_STAT: u8 = 0x4; // read/write, when DPBANKSEL == 0
const DP_SELECT: u8 = 0x8; // write
const DP_RDBUFF: u8 = 0xC; // read
const DP_TARGETSEL: u8 = 0xC; // write (SWD multidrop)

// --- MEM-AP register addresses (A[3:2] within the selected AP bank).
const AP_CSW: u8 = 0x00; // bank 0
const AP_TAR: u8 = 0x04; // bank 0
const AP_DRW: u8 = 0x0C; // bank 0

// --- CTRL/STAT bits.
const CDBGPWRUPREQ: u32 = 1 << 28;
const CDBGPWRUPACK: u32 = 1 << 29;
const CSYSPWRUPREQ: u32 = 1 << 30;
const CSYSPWRUPACK: u32 = 1 << 31;

// --- ABORT bits (clear sticky errors).
const ABORT_CLEAR_ALL: u32 = 0x1E; // STKCMPCLR|STKERRCLR|WDERRCLR|ORUNERRCLR

// --- CSW: 32-bit access size with address auto-increment (single).
const CSW_SIZE_WORD: u32 = 0b010;
const CSW_ADDRINC_SINGLE: u32 = 1 << 4;
const CSW_DEFAULT: u32 = CSW_SIZE_WORD | CSW_ADDRINC_SINGLE | 0x2300_0000; // + DbgSwEnable/Prot bits

/// Well-known RP2040 SWD multidrop target-select values (DP TARGETSEL).
pub mod rp2040 {
    pub const CORE0_TARGETSEL: u32 = 0x0100_2927;
    pub const CORE1_TARGETSEL: u32 = 0x1100_2927;
    pub const RESCUE_TARGETSEL: u32 = 0xf100_2927;
    /// SYSINFO.CHIP_ID — reads 0x2000_2927 on RP2040 B2 silicon.
    pub const SYSINFO_CHIP_ID: u32 = 0x4000_0000;
}

/// ADIv5 debug interface over a SWD [`DapTransport`].
pub struct ArmDebug<T: DapTransport> {
    transport: T,
    config: DapConfig,
    /// Cached DP SELECT value so redundant SELECT writes are skipped.
    select: u32,
    /// Cached MEM-AP CSW so it is programmed only once.
    csw_valid: bool,
    /// Retry budget for WAIT acks.
    retries: u32,
}

impl<T: DapTransport> ArmDebug<T> {
    pub fn new(transport: T, config: DapConfig) -> Self {
        Self {
            transport,
            config,
            select: 0xffff_ffff, // force first SELECT write
            csw_valid: false,
            retries: 64,
        }
    }

    pub fn transport(&mut self) -> &mut T {
        &mut self.transport
    }

    /// Single SWD transfer with WAIT retry.
    fn transfer(&mut self, port: Port, rnw: bool, addr: u8, data: u32) -> Result<u32, ArmError> {
        let mut request = SwdRequest::empty();
        if port == Port::Ap {
            request |= SwdRequest::APnDP;
        }
        if rnw {
            request |= SwdRequest::RnW;
        }
        if addr & 0x4 != 0 {
            request |= SwdRequest::A2;
        }
        if addr & 0x8 != 0 {
            request |= SwdRequest::A3;
        }
        let mut retry = 0;
        loop {
            match self.transport.swd_transfer(&self.config, request, data) {
                Ok(v) => return Ok(v),
                Err(DapError::SwdError(ack)) if ack == DAP_TRANSFER_WAIT => {
                    if retry == self.retries {
                        return Err(ArmError::Wait);
                    }
                    retry += 1;
                }
                Err(e) => return Err(e.into()),
            }
        }
    }

    fn dp_read(&mut self, addr: u8) -> Result<u32, ArmError> {
        self.transfer(Port::Dp, true, addr, 0)
    }
    fn dp_write(&mut self, addr: u8, data: u32) -> Result<(), ArmError> {
        self.transfer(Port::Dp, false, addr, data).map(|_| ())
    }

    /// Program DP SELECT for a given AP bank (APSEL fixed to 0, single MEM-AP).
    fn select_ap_bank(&mut self, ap_addr: u8) -> Result<(), ArmError> {
        let bank = (ap_addr as u32 >> 4) & 0xf;
        let select = bank << 4; // APSEL=0, APBANKSEL=bank, DPBANKSEL=0
        if select != self.select {
            self.dp_write(DP_SELECT, select)?;
            self.select = select;
        }
        Ok(())
    }

    fn ap_write(&mut self, addr: u8, data: u32) -> Result<(), ArmError> {
        self.select_ap_bank(addr)?;
        self.transfer(Port::Ap, false, addr & 0xC, data).map(|_| ())
    }

    /// AP reads are posted in SWD: issue the AP read (returns stale data),
    /// then read DP RDBUFF for the real value.
    fn ap_read(&mut self, addr: u8) -> Result<u32, ArmError> {
        self.select_ap_bank(addr)?;
        self.transfer(Port::Ap, true, addr & 0xC, 0)?; // posted
        self.dp_read(DP_RDBUFF)
    }

    /// Connect over SWD to a multidrop target (e.g. RP2040 core 0) and power
    /// up the debug domain. Returns the DPIDR.
    ///
    /// Sequence: dormant→SWD selection, line reset, TARGETSEL, read DPIDR,
    /// clear errors, SELECT=0, CDBGPWRUPREQ/CSYSPWRUPREQ and wait for ack.
    pub fn connect_multidrop(&mut self, targetsel: u32) -> Result<u32, ArmError> {
        self.transport.connect(ConnectPort::Swd, &self.config)?;
        self.swd_line_reset()?;
        self.dormant_to_swd()?;
        self.swd_line_reset()?;
        self.write_targetsel(targetsel)?;

        let dpidr = self.dp_read(DP_DPIDR)?;
        if dpidr == 0 || dpidr == 0xffff_ffff {
            return Err(ArmError::NoTarget);
        }
        self.select = 0xffff_ffff;
        self.csw_valid = false;

        self.dp_write(DP_ABORT, ABORT_CLEAR_ALL)?;
        self.dp_write(DP_SELECT, 0)?;
        self.select = 0;
        self.power_up()?;
        Ok(dpidr)
    }

    fn power_up(&mut self) -> Result<(), ArmError> {
        self.dp_write(DP_CTRL_STAT, CDBGPWRUPREQ | CSYSPWRUPREQ)?;
        let want = CDBGPWRUPACK | CSYSPWRUPACK;
        for _ in 0..1024 {
            if self.dp_read(DP_CTRL_STAT)? & want == want {
                return Ok(());
            }
        }
        Err(ArmError::PowerUpTimeout)
    }

    /// >= 50 clocks with SWDIO high, then two idle bits.
    fn swd_line_reset(&mut self) -> Result<(), ArmError> {
        const HIGH: [u8; 8] = [0xff; 8]; // 64 ones
        self.transport.swj_sequence(&self.config, 51, &HIGH)?;
        self.transport.swj_sequence(&self.config, 2, &[0x00])?;
        Ok(())
    }

    /// Dormant-to-SWD selection alert + activation code (ADIv5.2 / SWD v2).
    fn dormant_to_swd(&mut self) -> Result<(), ArmError> {
        // At least 8 clocks with SWDIOTMS high.
        self.transport.swj_sequence(&self.config, 8, &[0xff])?;
        // 128-bit selection alert sequence (LSB-first as transmitted).
        const SELECTION_ALERT: [u8; 16] = [
            0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86, 0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e,
            0xbc, 0x19,
        ];
        self.transport
            .swj_sequence(&self.config, 128, &SELECTION_ALERT)?;
        // 4 clocks low, then the 8-bit SWD activation code 0x1a.
        self.transport.swj_sequence(&self.config, 4, &[0x00])?;
        self.transport.swj_sequence(&self.config, 8, &[0x1a])?;
        Ok(())
    }

    /// SWD multidrop TARGETSEL write. The target does not drive an ACK for
    /// this write, so it is emitted as a raw sequence: the 8-bit request, a
    /// turnaround/ack skip, then 32-bit value + parity — never reading ACK.
    fn write_targetsel(&mut self, value: u32) -> Result<(), ArmError> {
        // Request byte for a DP write to TARGETSEL (addr 0xC): parity over the
        // four APnDP/RnW/A2/A3 bits.
        let apndp = false;
        let rnw = false;
        let a2 = (DP_TARGETSEL & 0x4) != 0;
        let a3 = (DP_TARGETSEL & 0x8) != 0;
        let parity = (apndp as u8) ^ (rnw as u8) ^ (a2 as u8) ^ (a3 as u8);
        let mut request = 0x81u8; // Start(1) ... Park(1): bit0=Start, bit7=Park
        request |= (apndp as u8) << 1;
        request |= (rnw as u8) << 2;
        request |= (a2 as u8) << 3;
        request |= (a3 as u8) << 4;
        request |= parity << 5;
        // bit6 = Stop = 0, bit7 = Park = 1 (already set in 0x81).

        self.transport.swd_output_enable(true)?;
        self.transport.swd_write_bits(&self.config, 8, &[request])?;
        // Turnaround + 3-bit ACK period, ignored: drive one turnaround and 4
        // clocks so the host stays output and the ACK is not sampled.
        self.transport.swd_write_bits(&self.config, 5, &[0x00])?;
        // 32-bit value LSB-first + even parity bit.
        let data_parity = (value.count_ones() & 1) as u8;
        let bytes = value.to_le_bytes();
        self.transport.swd_write_bits(&self.config, 32, &bytes)?;
        self.transport
            .swd_write_bits(&self.config, 1, &[data_parity])?;
        Ok(())
    }

    fn ensure_csw(&mut self) -> Result<(), ArmError> {
        if !self.csw_valid {
            self.ap_write(AP_CSW, CSW_DEFAULT)?;
            self.csw_valid = true;
        }
        Ok(())
    }

    /// Read a single 32-bit word from target memory via the MEM-AP.
    pub fn read_word(&mut self, addr: u32) -> Result<u32, ArmError> {
        self.ensure_csw()?;
        self.ap_write(AP_TAR, addr)?;
        self.ap_read(AP_DRW)
    }

    /// Write a single 32-bit word to target memory via the MEM-AP.
    pub fn write_word(&mut self, addr: u32, value: u32) -> Result<(), ArmError> {
        self.ensure_csw()?;
        self.ap_write(AP_TAR, addr)?;
        self.ap_write(AP_DRW, value)?;
        // Flush the posted write by reading RDBUFF.
        self.dp_read(DP_RDBUFF)?;
        Ok(())
    }

    /// Read a block of consecutive 32-bit words using TAR auto-increment.
    /// `addr` must be word-aligned and stay within a 1 KiB TAR wrap boundary
    /// (the caller splits larger reads).
    pub fn read_words(&mut self, addr: u32, out: &mut [u32]) -> Result<(), ArmError> {
        self.ensure_csw()?;
        self.ap_write(AP_TAR, addr)?;
        if out.is_empty() {
            return Ok(());
        }
        // Post the first read.
        self.select_ap_bank(AP_DRW)?;
        self.transfer(Port::Ap, true, AP_DRW & 0xC, 0)?;
        let (last, head) = out.split_last_mut().ok_or(ArmError::Internal)?;
        for slot in head {
            // Each AP read returns the previous word and posts the next.
            *slot = self.transfer(Port::Ap, true, AP_DRW & 0xC, 0)?;
        }
        // Final word comes from RDBUFF.
        *last = self.dp_read(DP_RDBUFF)?;
        Ok(())
    }

    /// True once a port has been selected by [`connect_multidrop`].
    pub fn is_swd(&self) -> bool {
        self.transport
            .capabilities()
            .contains(rust_dap::DapCapabilities::SWD)
    }
}

/// Convenience: what port the transport last reported active (for callers that
/// also drive it directly).
pub fn active_is_swd(port: Option<ActivePort>) -> bool {
    matches!(port, Some(ActivePort::Swd))
}

#[cfg(test)]
mod test;
