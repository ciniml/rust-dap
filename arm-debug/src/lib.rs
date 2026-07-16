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
    /// A core-debug operation (halt/step/register access) did not complete.
    CoreTimeout,
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
    /// XIP flash window base address.
    pub const FLASH_BASE: u32 = 0x1000_0000;
    /// Word holding the (u16) bootrom function-table and data-table pointers.
    pub const ROM_TABLE_PTRS: u32 = 0x0000_0014;

    // Bootrom function-table codes (two ASCII chars).
    pub const FN_CONNECT_INTERNAL_FLASH: [u8; 2] = *b"IF";
    pub const FN_FLASH_EXIT_XIP: [u8; 2] = *b"EX";
    pub const FN_FLASH_RANGE_ERASE: [u8; 2] = *b"RE";
    pub const FN_FLASH_RANGE_PROGRAM: [u8; 2] = *b"RP";
    pub const FN_FLASH_FLUSH_CACHE: [u8; 2] = *b"FC";
    pub const FN_FLASH_ENTER_CMD_XIP: [u8; 2] = *b"CX";
}

/// Nordic nRF52 debug constants. The CTRL-AP (APSEL 1) stays reachable even
/// when APPROTECT blocks the AHB-AP, so its APPROTECTSTATUS reports the
/// protection state and ERASEALL performs the (only) unlock.
pub mod nrf52 {
    /// CTRL-AP is Nordic's custom AP at APSEL 1.
    pub const CTRL_AP: u8 = 1;
    /// CTRL-AP.RESET (offset 0x000): 1 asserts soft reset, 0 releases.
    pub const CTRLAP_RESET: u8 = 0x00;
    /// CTRL-AP.ERASEALL (0x004): write 1 to erase flash+UICR+RAM.
    pub const CTRLAP_ERASEALL: u8 = 0x04;
    /// CTRL-AP.ERASEALLSTATUS (0x008): 0=Ready, 1=Busy.
    pub const CTRLAP_ERASEALLSTATUS: u8 = 0x08;
    /// CTRL-AP.APPROTECTSTATUS (0x00C): bit0 1=unprotected, 0=protected.
    pub const CTRLAP_APPROTECTSTATUS: u8 = 0x0C;
    /// CTRL-AP.IDR (0x0FC) reads 0x0288_0000.
    pub const CTRLAP_IDR: u8 = 0xFC;
    pub const CTRLAP_IDR_VALUE: u32 = 0x0288_0000;
    /// SW-DP IDCODE for nRF52.
    pub const DPIDR: u32 = 0x2BA0_1477;

    // --- NVMC: non-volatile memory controller (register-based flashing) ---
    /// NVMC.READY (0x4001E400): bit0 1=ready, 0=busy.
    pub const NVMC_READY: u32 = 0x4001_E400;
    /// NVMC.CONFIG (0x4001E504): 0=read-only, 1=write enable, 2=erase enable.
    pub const NVMC_CONFIG: u32 = 0x4001_E504;
    /// NVMC.ERASEPAGE (0x4001E508): write a page's base address to erase it.
    pub const NVMC_ERASEPAGE: u32 = 0x4001_E508;
    pub const NVMC_CONFIG_REN: u32 = 0;
    pub const NVMC_CONFIG_WEN: u32 = 1;
    pub const NVMC_CONFIG_EEN: u32 = 2;
    /// Code flash page size (nRF52832/833/840 all use 4 KiB).
    pub const FLASH_PAGE: u32 = 4096;
    /// Code flash base.
    pub const FLASH_BASE: u32 = 0x0000_0000;
}

/// Nordic nRF54L debug constants (Cortex-M33, ARMv8-M). Same CoreSight
/// SW-DP family as nRF52 but DPv2 (DPIDR 0x6BA02477), CTRL-AP at APSEL 2,
/// and RRAM (resistive, no erase) programmed via the RRAMC.
pub mod nrf54 {
    /// SW-DP IDCODE for nRF54L (DP architecture v2).
    pub const DPIDR: u32 = 0x6BA0_2477;
    /// CTRL-AP is at APSEL 2 on nRF54L (was 1 on nRF52).
    pub const CTRL_AP: u8 = 2;
    // CTRL-AP register offsets mirror the nRF52 layout.
    pub const CTRLAP_RESET: u8 = 0x00;
    pub const CTRLAP_ERASEALL: u8 = 0x04;
    pub const CTRLAP_ERASEALLSTATUS: u8 = 0x08;
    pub const CTRLAP_APPROTECTSTATUS: u8 = 0x0C;

    // --- RRAMC: resistive-RAM controller (register-driven, erase-free) ---
    // Non-secure alias 0x4004_B000; secure alias 0x5004_B000.
    /// RRAMC.READY (0x400): bit0 1=ready.
    pub const RRAMC_READY: u32 = 0x4004_B400;
    /// RRAMC.READYNEXT (0x404): bit0 1=ready to accept the next write.
    pub const RRAMC_READYNEXT: u32 = 0x4004_B404;
    /// RRAMC.CONFIG (0x500): bit0 WEN (write enable).
    pub const RRAMC_CONFIG: u32 = 0x4004_B500;
    /// RRAMC.TASKS_COMMITWRITEBUF (0x008): write 1 to flush the write buffer.
    pub const RRAMC_COMMIT: u32 = 0x4004_B008;
    pub const RRAMC_CONFIG_WEN: u32 = 1;
    /// RRAM base (code) and size on nRF54L15 (1.5 MiB).
    pub const RRAM_BASE: u32 = 0x0000_0000;
    pub const RRAM_SIZE: u32 = 1524 * 1024;
}

/// ADIv5 debug interface over a SWD [`DapTransport`].
pub struct ArmDebug<T: DapTransport> {
    transport: T,
    config: DapConfig,
    /// Cached DP SELECT value so redundant SELECT writes are skipped.
    select: u32,
    /// Cached MEM-AP CSW so it is programmed only once.
    csw_valid: bool,
    /// Currently selected AP (APSEL). 0 = MEM-AP; Nordic CTRL-AP is 1.
    apsel: u8,
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
            apsel: 0,
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
                Err(e) => {
                    // A FAULT sets a sticky error bit that makes every later AP
                    // access fault too, poisoning the whole session. Clear it
                    // so a bad access (e.g. a GDB read of unmapped memory)
                    // fails cleanly and the next operation works.
                    self.clear_sticky_errors();
                    return Err(e.into());
                }
            }
        }
    }

    /// Write DP ABORT to clear sticky error flags, bypassing the normal retry
    /// path (ABORT is always accessible even while errors are latched).
    fn clear_sticky_errors(&mut self) {
        let abort = SwdRequest::empty(); // DP write, addr 0 (ABORT)
        let _ = self
            .transport
            .swd_transfer(&self.config, abort, ABORT_CLEAR_ALL);
    }

    fn dp_read(&mut self, addr: u8) -> Result<u32, ArmError> {
        self.transfer(Port::Dp, true, addr, 0)
    }
    fn dp_write(&mut self, addr: u8, data: u32) -> Result<(), ArmError> {
        self.transfer(Port::Dp, false, addr, data).map(|_| ())
    }

    /// Program DP SELECT for a given AP (APSEL) and register bank. `ap_addr`
    /// carries the bank in bits[7:4]; the current MEM-AP is `self.apsel`.
    fn select_ap_bank(&mut self, ap_addr: u8) -> Result<(), ArmError> {
        let bank = (ap_addr as u32 >> 4) & 0xf;
        let select = (self.apsel as u32) << 24 | bank << 4; // APSEL|APBANKSEL, DPBANKSEL=0
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

    /// Select which AP (APSEL) subsequent MEM-AP / raw AP accesses target.
    /// Invalidates the cached CSW (a different AP has its own CSW). The
    /// normal MEM-AP is APSEL 0; Nordic's CTRL-AP is APSEL 1.
    pub fn set_apsel(&mut self, apsel: u8) {
        if apsel != self.apsel {
            self.apsel = apsel;
            self.csw_valid = false;
        }
    }

    /// Raw read of AP register `addr` on the currently selected APSEL (does
    /// not touch CSW/TAR). For custom APs like Nordic CTRL-AP.
    pub fn ap_reg_read(&mut self, addr: u8) -> Result<u32, ArmError> {
        self.ap_read(addr)
    }

    /// Raw write of AP register `addr` on the currently selected APSEL.
    pub fn ap_reg_write(&mut self, addr: u8, data: u32) -> Result<(), ArmError> {
        self.ap_write(addr, data)
    }

    /// Connect over SWD to a multidrop target (e.g. RP2040 core 0) and power
    /// up the debug domain. Returns the DPIDR.
    ///
    /// Sequence: dormant→SWD selection, line reset, TARGETSEL, read DPIDR,
    /// clear errors, SELECT=0, CDBGPWRUPREQ/CSYSPWRUPREQ and wait for ack.
    pub fn connect_multidrop(&mut self, targetsel: u32) -> Result<u32, ArmError> {
        self.transport.connect(ConnectPort::Swd, &self.config)?;
        self.swd_line_reset()?;
        // Force a known protocol state: an already-active SWD target is sent
        // to dormant first (no-op if already dormant). Without this, calling
        // connect on a live link desyncs every other attempt.
        self.swd_to_dormant()?;
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

    /// Connect over SWD to a plain (non-multidrop) SW-DP — nRF52 and most
    /// single-core Cortex-M parts. Returns the DPIDR.
    ///
    /// Sequence: line reset, JTAG-to-SWD select (0xE79E), line reset,
    /// read DPIDR, clear errors, SELECT=0, power up. No dormant/TARGETSEL:
    /// SW-DPv1 has neither.
    pub fn connect_swd(&mut self) -> Result<u32, ArmError> {
        self.transport.connect(ConnectPort::Swd, &self.config)?;
        self.swd_line_reset()?;
        self.jtag_to_swd()?;
        self.swd_line_reset()?;
        // A read of DPIDR must be the first transfer after reset.
        let dpidr = self.dp_read(DP_DPIDR)?;
        if dpidr == 0 || dpidr == 0xffff_ffff {
            return Err(ArmError::NoTarget);
        }
        self.select = 0xffff_ffff;
        self.csw_valid = false;
        self.apsel = 0;
        self.dp_write(DP_ABORT, ABORT_CLEAR_ALL)?;
        self.dp_write(DP_SELECT, 0)?;
        self.select = 0;
        self.power_up()?;
        Ok(dpidr)
    }

    /// JTAG-to-SWD select sequence: the 16-bit code 0xE79E (LSB-first).
    /// Switches a SWJ-DP that came up in JTAG mode over to SWD.
    fn jtag_to_swd(&mut self) -> Result<(), ArmError> {
        self.transport
            .swj_sequence(&self.config, 16, &0xE79Eu16.to_le_bytes())?;
        Ok(())
    }

    /// Switch to another multidrop target on an already-active SWD link
    /// (e.g. the other RP2040 core): line reset, TARGETSEL, DPIDR, power
    /// check. Much cheaper than [`connect_multidrop`] — no dormant dance —
    /// but requires that a full connect has happened since power-on.
    pub fn reselect(&mut self, targetsel: u32) -> Result<u32, ArmError> {
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
        // Fast when this DP was already powered (first CTRL/STAT read acks).
        self.power_up()?;
        Ok(dpidr)
    }

    /// Read the nRF52 CTRL-AP APPROTECTSTATUS: `Ok(true)` = debug access
    /// enabled (unprotected), `Ok(false)` = APPROTECT blocking the AHB-AP.
    /// Also returns whether the CTRL-AP IDR matched (sanity check).
    pub fn nrf52_approtect_status(&mut self) -> Result<(bool, bool), ArmError> {
        self.set_apsel(nrf52::CTRL_AP);
        let idr = self.ap_reg_read(nrf52::CTRLAP_IDR)?;
        let status = self.ap_reg_read(nrf52::CTRLAP_APPROTECTSTATUS)?;
        self.set_apsel(0);
        Ok((status & 1 != 0, idr == nrf52::CTRLAP_IDR_VALUE))
    }

    /// nRF52 recovery: CTRL-AP ERASEALL (erases flash+UICR+RAM, the only way
    /// past APPROTECT), poll ERASEALLSTATUS, then soft-reset via CTRL-AP.
    /// `polls` bounds the busy wait (erase-all takes a few hundred ms).
    pub fn nrf52_erase_all(&mut self, polls: u32) -> Result<(), ArmError> {
        self.set_apsel(nrf52::CTRL_AP);
        self.ap_reg_write(nrf52::CTRLAP_ERASEALL, 1)?;
        let mut ok = Err(ArmError::CoreTimeout);
        for _ in 0..polls {
            if self.ap_reg_read(nrf52::CTRLAP_ERASEALLSTATUS)? & 1 == 0 {
                ok = Ok(());
                break;
            }
        }
        // Soft reset so the newly-unprotected part comes up debuggable.
        self.ap_reg_write(nrf52::CTRLAP_RESET, 1)?;
        self.ap_reg_write(nrf52::CTRLAP_RESET, 0)?;
        self.set_apsel(0);
        ok
    }

    fn nrf52_nvmc_wait(&mut self, polls: u32) -> Result<(), ArmError> {
        for _ in 0..polls {
            if self.read_word(nrf52::NVMC_READY)? & 1 != 0 {
                return Ok(());
            }
        }
        Err(ArmError::CoreTimeout)
    }

    /// Erase one nRF52 code-flash page (`addr` need not be page-aligned; the
    /// page base is used). `polls` bounds the NVMC busy wait (~85 ms/page).
    pub fn nrf52_erase_page(&mut self, addr: u32, polls: u32) -> Result<(), ArmError> {
        let page = addr & !(nrf52::FLASH_PAGE - 1);
        self.write_word(nrf52::NVMC_CONFIG, nrf52::NVMC_CONFIG_EEN)?;
        self.nrf52_nvmc_wait(polls)?;
        self.write_word(nrf52::NVMC_ERASEPAGE, page)?;
        self.nrf52_nvmc_wait(polls)?;
        self.write_word(nrf52::NVMC_CONFIG, nrf52::NVMC_CONFIG_REN)?;
        Ok(())
    }

    /// Program word-aligned data to nRF52 code flash (caller erases first).
    /// The NVMC only clears bits, so the target range must already be 0xFF.
    /// `words` are written in order via the MEM-AP (auto-increment), NVMC in
    /// write-enable throughout, then a final READY wait.
    pub fn nrf52_program(&mut self, addr: u32, words: &[u32], polls: u32) -> Result<(), ArmError> {
        if words.is_empty() {
            return Ok(());
        }
        self.write_word(nrf52::NVMC_CONFIG, nrf52::NVMC_CONFIG_WEN)?;
        self.nrf52_nvmc_wait(polls)?;
        // Program in TAR-wrap-bounded runs (write_words handles the MEM-AP).
        let mut a = addr;
        let mut rest = words;
        while !rest.is_empty() {
            let run = Self::tar_run(a, rest.len());
            self.write_words(a, &rest[..run])?;
            self.nrf52_nvmc_wait(polls)?;
            a += (run as u32) * 4;
            rest = &rest[run..];
        }
        self.write_word(nrf52::NVMC_CONFIG, nrf52::NVMC_CONFIG_REN)?;
        Ok(())
    }

    // --- nRF54L (Cortex-M33 / RRAM) --------------------------------------

    /// Read the nRF54 CTRL-AP (APSEL 2) APPROTECTSTATUS: `Ok(true)` = debug
    /// access enabled. Uses the wider CTRL-AP index than nRF52.
    pub fn nrf54_approtect_status(&mut self) -> Result<bool, ArmError> {
        self.set_apsel(nrf54::CTRL_AP);
        let status = self.ap_reg_read(nrf54::CTRLAP_APPROTECTSTATUS)?;
        self.set_apsel(0);
        Ok(status & 1 != 0)
    }

    /// nRF54 recovery via CTRL-AP ERASEALL + soft reset.
    pub fn nrf54_erase_all(&mut self, polls: u32) -> Result<(), ArmError> {
        self.set_apsel(nrf54::CTRL_AP);
        self.ap_reg_write(nrf54::CTRLAP_ERASEALL, 1)?;
        let mut ok = Err(ArmError::CoreTimeout);
        for _ in 0..polls {
            if self.ap_reg_read(nrf54::CTRLAP_ERASEALLSTATUS)? & 1 == 0 {
                ok = Ok(());
                break;
            }
        }
        self.ap_reg_write(nrf54::CTRLAP_RESET, 1)?;
        self.ap_reg_write(nrf54::CTRLAP_RESET, 0)?;
        self.set_apsel(0);
        ok
    }

    fn nrf54_rramc_wait(&mut self, polls: u32) -> Result<(), ArmError> {
        for _ in 0..polls {
            if self.read_word(nrf54::RRAMC_READY)? & 1 != 0 {
                return Ok(());
            }
        }
        Err(ArmError::CoreTimeout)
    }

    /// Program words to nRF54L RRAM. RRAM is resistive: no erase, direct
    /// overwrite. Sequence: CONFIG.WEN=1, write words via MEM-AP (buffered),
    /// TASKS_COMMITWRITEBUF, poll READY, CONFIG=0.
    pub fn nrf54_program(&mut self, addr: u32, words: &[u32], polls: u32) -> Result<(), ArmError> {
        if words.is_empty() {
            return Ok(());
        }
        self.write_word(nrf54::RRAMC_CONFIG, nrf54::RRAMC_CONFIG_WEN)?;
        let mut a = addr;
        let mut rest = words;
        while !rest.is_empty() {
            let run = Self::tar_run(a, rest.len());
            self.write_words(a, &rest[..run])?;
            a += (run as u32) * 4;
            rest = &rest[run..];
        }
        self.write_word(nrf54::RRAMC_COMMIT, 1)?;
        self.nrf54_rramc_wait(polls)?;
        self.write_word(nrf54::RRAMC_CONFIG, 0)?;
        Ok(())
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

    /// Active SWD → dormant: line reset then the 16-bit select sequence
    /// 0xE3BC (LSB-first). Ignored by a target already dormant.
    fn swd_to_dormant(&mut self) -> Result<(), ArmError> {
        self.transport
            .swj_sequence(&self.config, 16, &0xE3BCu16.to_le_bytes())?;
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

    /// Write a block of consecutive 32-bit words using TAR auto-increment.
    /// `addr` must be word-aligned and stay within a 1 KiB TAR wrap boundary
    /// (the caller splits larger writes).
    pub fn write_words(&mut self, addr: u32, words: &[u32]) -> Result<(), ArmError> {
        if words.is_empty() {
            return Ok(());
        }
        self.ensure_csw()?;
        self.ap_write(AP_TAR, addr)?;
        self.select_ap_bank(AP_DRW)?;
        for w in words {
            self.transfer(Port::Ap, false, AP_DRW & 0xC, *w)?;
        }
        // Flush the posted writes.
        self.dp_read(DP_RDBUFF)?;
        Ok(())
    }

    /// Largest word count starting at `addr` that stays inside the 1 KiB TAR
    /// auto-increment wrap boundary.
    fn tar_run(addr: u32, words: usize) -> usize {
        (((1024 - (addr & 1023)) / 4) as usize).min(words)
    }

    /// Read an arbitrary byte range from target memory. Aligned full words
    /// use block reads with TAR auto-increment; ragged edges use word reads.
    pub fn read_mem(&mut self, mut addr: u32, buf: &mut [u8]) -> Result<(), ArmError> {
        let mut i = 0;
        while i < buf.len() {
            let start = (addr & 3) as usize;
            let remaining = buf.len() - i;
            if start == 0 && remaining >= 4 {
                // Bulk path: as many whole words as fit this TAR wrap run.
                let n = Self::tar_run(addr, remaining / 4);
                let mut words = [0u32; 16];
                let n = n.min(words.len());
                self.read_words(addr, &mut words[..n])?;
                for w in &words[..n] {
                    buf[i..i + 4].copy_from_slice(&w.to_le_bytes());
                    i += 4;
                    addr += 4;
                }
            } else {
                let word = self.read_word(addr & !3)?.to_le_bytes();
                for b in word.iter().skip(start) {
                    if i >= buf.len() {
                        break;
                    }
                    buf[i] = *b;
                    i += 1;
                    addr += 1;
                }
            }
        }
        Ok(())
    }

    /// Write an arbitrary byte range to target memory. Aligned full words use
    /// block writes with TAR auto-increment; partial words read-modify-write.
    pub fn write_mem(&mut self, mut addr: u32, buf: &[u8]) -> Result<(), ArmError> {
        let mut i = 0;
        while i < buf.len() {
            let aligned = addr & !3;
            let start = (addr & 3) as usize;
            if start == 0 && buf.len() - i >= 4 {
                let n = Self::tar_run(addr, (buf.len() - i) / 4);
                let mut words = [0u32; 16];
                let n = n.min(words.len());
                for (k, w) in words[..n].iter_mut().enumerate() {
                    let j = i + 4 * k;
                    *w = u32::from_le_bytes([buf[j], buf[j + 1], buf[j + 2], buf[j + 3]]);
                }
                self.write_words(addr, &words[..n])?;
                i += 4 * n;
                addr += 4 * n as u32;
            } else {
                let mut word = self.read_word(aligned)?.to_le_bytes();
                for b in word.iter_mut().skip(start) {
                    if i >= buf.len() {
                        break;
                    }
                    *b = buf[i];
                    i += 1;
                    addr += 1;
                }
                self.write_word(aligned, u32::from_le_bytes(word))?;
            }
        }
        Ok(())
    }

    /// True once a port has been selected by [`connect_multidrop`].
    pub fn is_swd(&self) -> bool {
        self.transport
            .capabilities()
            .contains(rust_dap::DapCapabilities::SWD)
    }
}

/// ARMv6-M / ARMv7-M core-debug register addresses and bits (milestone M2).
/// These are ordinary system-memory addresses reached through the MEM-AP, so
/// the core-debug methods build directly on [`ArmDebug::read_word`] /
/// [`ArmDebug::write_word`].
pub mod cortex_m {
    /// Debug Halting Control and Status Register.
    pub const DHCSR: u32 = 0xE000_EDF0;
    /// Debug Core Register Selector.
    pub const DCRSR: u32 = 0xE000_EDF4;
    /// Debug Core Register Data.
    pub const DCRDR: u32 = 0xE000_EDF8;
    /// Debug Exception and Monitor Control.
    pub const DEMCR: u32 = 0xE000_EDFC;
    /// Application Interrupt and Reset Control (SYSRESETREQ).
    pub const AIRCR: u32 = 0xE000_ED0C;

    /// Key that must be written to the top half of DHCSR for the write to take.
    pub const DBGKEY: u32 = 0xA05F_0000;
    pub const C_DEBUGEN: u32 = 1 << 0;
    pub const C_HALT: u32 = 1 << 1;
    pub const C_STEP: u32 = 1 << 2;
    pub const C_MASKINTS: u32 = 1 << 3;
    /// DHCSR read-side status: core-register transfer ready.
    pub const S_REGRDY: u32 = 1 << 16;
    /// DHCSR read-side status: core is halted.
    pub const S_HALT: u32 = 1 << 17;
    /// DHCSR read-side status: core is sleeping.
    pub const S_SLEEP: u32 = 1 << 18;
    /// DHCSR read-side status: core is locked up.
    pub const S_LOCKUP: u32 = 1 << 19;

    /// DCRSR write direction: 1 = write the register, 0 = read it.
    pub const DCRSR_REGWNR: u32 = 1 << 16;

    /// Debug Fault Status Register (halt reason; bits are write-1-to-clear).
    pub const DFSR: u32 = 0xE000_ED30;
    pub const DFSR_HALTED: u32 = 1 << 0;
    pub const DFSR_BKPT: u32 = 1 << 1;
    pub const DFSR_DWTTRAP: u32 = 1 << 2;
    pub const DFSR_VCATCH: u32 = 1 << 3;

    /// DEMCR: enable the DWT unit (called DWTENA on ARMv6-M, TRCENA on v7-M).
    pub const DEMCR_DWTENA: u32 = 1 << 24;
    /// DEMCR: vector catch — halt the core at the reset vector.
    pub const DEMCR_VC_CORERESET: u32 = 1 << 0;

    /// AIRCR value requesting a system reset (VECTKEY | SYSRESETREQ).
    pub const AIRCR_SYSRESETREQ: u32 = 0x05FA_0004;

    // Core register selector values (a subset; r0-r15 are 0..=15).
    pub const R0: u8 = 0;
    pub const SP: u8 = 13;
    pub const LR: u8 = 14;
    /// Debug return address == PC.
    pub const PC: u8 = 15;
    pub const XPSR: u8 = 16;
    pub const MSP: u8 = 17;
    pub const PSP: u8 = 18;
}

impl<T: DapTransport> ArmDebug<T> {
    /// Enable halting debug (`C_DEBUGEN`) without changing the run state.
    pub fn debug_enable(&mut self) -> Result<(), ArmError> {
        self.write_word(cortex_m::DHCSR, cortex_m::DBGKEY | cortex_m::C_DEBUGEN)
    }

    /// Halt the core and wait until it reports halted.
    pub fn halt(&mut self) -> Result<(), ArmError> {
        self.write_word(
            cortex_m::DHCSR,
            cortex_m::DBGKEY | cortex_m::C_DEBUGEN | cortex_m::C_HALT,
        )?;
        self.wait_status(cortex_m::S_HALT)
    }

    /// Resume execution (clear halt, keep debug enabled).
    pub fn resume(&mut self) -> Result<(), ArmError> {
        self.write_word(cortex_m::DHCSR, cortex_m::DBGKEY | cortex_m::C_DEBUGEN)
    }

    /// Single-step one instruction with interrupts masked, then wait for halt.
    pub fn step(&mut self) -> Result<(), ArmError> {
        self.write_word(
            cortex_m::DHCSR,
            cortex_m::DBGKEY | cortex_m::C_DEBUGEN | cortex_m::C_MASKINTS | cortex_m::C_STEP,
        )?;
        self.wait_status(cortex_m::S_HALT)
    }

    /// Whether the core currently reports halted.
    pub fn is_halted(&mut self) -> Result<bool, ArmError> {
        Ok(self.read_word(cortex_m::DHCSR)? & cortex_m::S_HALT != 0)
    }

    fn wait_status(&mut self, mask: u32) -> Result<(), ArmError> {
        self.wait_status_n(mask, 4096)
    }

    fn wait_status_n(&mut self, mask: u32, polls: u32) -> Result<(), ArmError> {
        for _ in 0..polls {
            if self.read_word(cortex_m::DHCSR)? & mask == mask {
                return Ok(());
            }
        }
        Err(ArmError::CoreTimeout)
    }

    /// Read a core register (see the `cortex_m` selector constants). The core
    /// must be halted.
    pub fn read_core_reg(&mut self, reg: u8) -> Result<u32, ArmError> {
        self.write_word(cortex_m::DCRSR, reg as u32)?;
        self.wait_status(cortex_m::S_REGRDY)?;
        self.read_word(cortex_m::DCRDR)
    }

    /// Write a core register. The core must be halted.
    pub fn write_core_reg(&mut self, reg: u8, value: u32) -> Result<(), ArmError> {
        self.write_word(cortex_m::DCRDR, value)?;
        self.write_word(cortex_m::DCRSR, reg as u32 | cortex_m::DCRSR_REGWNR)?;
        self.wait_status(cortex_m::S_REGRDY)
    }
}

impl<T: DapTransport> ArmDebug<T> {
    /// Look up a function in the RP2040 bootrom table by its two-character
    /// code. Returns None if the code is absent.
    pub fn rom_func_lookup(&mut self, code: [u8; 2]) -> Result<Option<u32>, ArmError> {
        let ptrs = self.read_word(rp2040::ROM_TABLE_PTRS)?;
        let mut entry = ptrs & 0xFFFF; // u16 pointer to the function table
        let want = u16::from_le_bytes(code);
        loop {
            // Each entry is (u16 code, u16 addr); the table ends at code 0.
            let mut bytes = [0u8; 4];
            self.read_mem(entry, &mut bytes)?;
            let tag = u16::from_le_bytes([bytes[0], bytes[1]]);
            if tag == 0 {
                return Ok(None);
            }
            if tag == want {
                return Ok(Some(u16::from_le_bytes([bytes[2], bytes[3]]) as u32));
            }
            entry += 4;
        }
    }

    /// Call a function on the halted target: set up r0-r3/sp, point lr at a
    /// BKPT trampoline planted in target RAM, run with interrupts masked
    /// (C_MASKINTS) and wait for the BKPT to halt the core again. Returns r0.
    ///
    /// `polls` bounds the DHCSR wait (flash erase calls take tens of ms).
    /// The registers it uses (r0-r3, sp, lr, pc, xPSR) are saved and
    /// restored, so the debugger-visible register state survives — critical
    /// when a flash operation runs between GDB setting the PC and resuming.
    pub fn call_function(
        &mut self,
        fn_addr: u32,
        args: &[u32; 4],
        sp: u32,
        trampoline: u32,
        polls: u32,
    ) -> Result<u32, ArmError> {
        const CLOBBERED: [u8; 8] = [
            cortex_m::R0,
            1,
            2,
            3,
            cortex_m::SP,
            cortex_m::LR,
            cortex_m::PC,
            cortex_m::XPSR,
        ];
        let mut saved = [0u32; 8];
        for (slot, reg) in saved.iter_mut().zip(CLOBBERED) {
            *slot = self.read_core_reg(reg)?;
        }

        self.write_word(trampoline & !3, 0xBE00_BE00)?; // BKPT #0, twice
        for (i, a) in args.iter().enumerate() {
            self.write_core_reg(i as u8, *a)?;
        }
        self.write_core_reg(cortex_m::SP, sp)?;
        self.write_core_reg(cortex_m::LR, trampoline | 1)?;
        self.write_core_reg(cortex_m::PC, fn_addr)?;
        self.write_core_reg(cortex_m::XPSR, 0x0100_0000)?; // T bit only
                                                           // C_MASKINTS may only change while halted: set it, then release halt.
        let masked = cortex_m::DBGKEY | cortex_m::C_DEBUGEN | cortex_m::C_MASKINTS;
        self.write_word(cortex_m::DHCSR, masked | cortex_m::C_HALT)?;
        self.write_word(cortex_m::DHCSR, masked)?;
        let waited = self.wait_status_n(cortex_m::S_HALT, polls);
        // Re-halt and clear C_MASKINTS (again: only changeable while halted).
        self.write_word(cortex_m::DHCSR, masked | cortex_m::C_HALT)?;
        self.write_word(
            cortex_m::DHCSR,
            cortex_m::DBGKEY | cortex_m::C_DEBUGEN | cortex_m::C_HALT,
        )?;
        waited?;
        let result = self.read_core_reg(cortex_m::R0)?;
        for (slot, reg) in saved.iter().zip(CLOBBERED) {
            self.write_core_reg(reg, *slot)?;
        }
        Ok(result)
    }
}

/// Flash Patch and Breakpoint unit (ARMv6-M FPB v1) register addresses.
pub mod fpb {
    /// FP_CTRL: NUM_CODE in bits[7:4]; bit1 = KEY (must be 1 for the write to
    /// take), bit0 = ENABLE.
    pub const FP_CTRL: u32 = 0xE000_2000;
    /// First comparator; comparator n is at `FP_COMP0 + 4 * n`.
    pub const FP_COMP0: u32 = 0xE000_2008;
    pub const CTRL_KEY: u32 = 1 << 1;
    pub const CTRL_ENABLE: u32 = 1 << 0;
    pub const COMP_ENABLE: u32 = 1 << 0;
}

/// Data Watchpoint and Trace unit (ARMv6-M subset) register addresses.
pub mod dwt {
    /// DWT_CTRL: NUMCOMP in bits[31:28].
    pub const DWT_CTRL: u32 = 0xE000_1000;
    /// Comparator n registers are at `COMP0/MASK0/FUNCTION0 + 0x10 * n`.
    pub const COMP0: u32 = 0xE000_1020;
    pub const MASK0: u32 = 0xE000_1024;
    pub const FUNCTION0: u32 = 0xE000_1028;
    /// FUNCTION[24]: comparator matched since last read (reading clears it).
    pub const FUNC_MATCHED: u32 = 1 << 24;
    /// FUNCTION[3:0] encodings for data-address watchpoints.
    pub const FUNC_READ: u32 = 0b0101;
    pub const FUNC_WRITE: u32 = 0b0110;
    pub const FUNC_READWRITE: u32 = 0b0111;
}

/// Access kind for a data watchpoint.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WatchAccess {
    Read,
    Write,
    ReadWrite,
}

impl WatchAccess {
    fn function(self) -> u32 {
        match self {
            WatchAccess::Read => dwt::FUNC_READ,
            WatchAccess::Write => dwt::FUNC_WRITE,
            WatchAccess::ReadWrite => dwt::FUNC_READWRITE,
        }
    }
}

/// Why the core halted, decoded from DFSR (milestone M4).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HaltReason {
    /// BKPT instruction or FPB comparator hit.
    Breakpoint,
    /// DWT watchpoint hit; carries the comparator address and access kind.
    Watchpoint(u32, WatchAccess),
    /// Halt request or single-step completion.
    HaltRequest,
    /// Vector catch.
    VectorCatch,
    /// Nothing recorded (e.g. already-cleared DFSR).
    Unknown,
}

impl<T: DapTransport> ArmDebug<T> {
    /// Number of FPB code comparators (0 if no FPB).
    fn fpb_num_comps(&mut self) -> Result<u32, ArmError> {
        // NUM_CODE spans FP_CTRL[14:12]:[7:4] (7 bits) on ARMv7-M; the high
        // field reads 0 on ARMv6-M (Cortex-M0+), so this covers both.
        let ctrl = self.read_word(fpb::FP_CTRL)?;
        Ok(((ctrl >> 12) & 0x7) << 4 | (ctrl >> 4) & 0xF)
    }

    /// FP_COMP value for a breakpoint at `addr`, or None if the address is
    /// not breakable. `rev` is FP_CTRL.REV (0 = FPBv1 on M0+/M3/M4, 1 = FPBv2
    /// on M7 and ARMv8-M).
    ///
    /// FPBv1: `REPLACE[31:30] | addr[28:2]`, code region < 0x2000_0000 only.
    /// FPBv2: `BPADDR[31:1] | BE[0]`, any address (used by nRF54's M33).
    fn fpb_comp_value(addr: u32, rev: u32) -> Option<u32> {
        if rev >= 1 {
            // FPBv2: breakpoint enable bit is bit0, address is [31:1].
            return Some((addr & 0xFFFF_FFFE) | fpb::COMP_ENABLE);
        }
        if addr >= 0x2000_0000 {
            return None;
        }
        // REPLACE selects which halfword of the word triggers.
        let replace = if addr & 2 != 0 { 0b10u32 } else { 0b01u32 };
        Some((replace << 30) | (addr & 0x1FFF_FFFC) | fpb::COMP_ENABLE)
    }

    /// FP_CTRL.REV (bits[31:28]): 0 = FPBv1, 1 = FPBv2.
    fn fpb_rev(&mut self) -> Result<u32, ArmError> {
        Ok((self.read_word(fpb::FP_CTRL)? >> 28) & 0xF)
    }

    /// Set a hardware breakpoint at `addr` (code region only). Returns false
    /// if the address is not breakable or no comparator is free. Comparator
    /// occupancy is read back from the hardware, so no host state is kept.
    pub fn hw_breakpoint_set(&mut self, addr: u32) -> Result<bool, ArmError> {
        let ctrl = self.read_word(fpb::FP_CTRL)?;
        let rev = (ctrl >> 28) & 0xF;
        let Some(value) = Self::fpb_comp_value(addr, rev) else {
            return Ok(false);
        };
        let n = ((ctrl >> 12) & 0x7) << 4 | (ctrl >> 4) & 0xF;
        if n == 0 {
            return Ok(false);
        }
        // NUM_CODE is read-only; KEY reads as zero and must be set for the
        // enable write to take.
        self.write_word(fpb::FP_CTRL, ctrl | fpb::CTRL_KEY | fpb::CTRL_ENABLE)?;
        let mut free = None;
        for i in 0..n {
            let comp = self.read_word(fpb::FP_COMP0 + 4 * i)?;
            if comp == value {
                return Ok(true); // already set
            }
            if free.is_none() && comp & fpb::COMP_ENABLE == 0 {
                free = Some(i);
            }
        }
        match free {
            Some(i) => {
                self.write_word(fpb::FP_COMP0 + 4 * i, value)?;
                Ok(true)
            }
            None => Ok(false),
        }
    }

    /// Clear the hardware breakpoint at `addr`. Returns false if none is set.
    pub fn hw_breakpoint_clear(&mut self, addr: u32) -> Result<bool, ArmError> {
        let rev = self.fpb_rev()?;
        let Some(value) = Self::fpb_comp_value(addr, rev) else {
            return Ok(false);
        };
        let n = self.fpb_num_comps()?;
        for i in 0..n {
            if self.read_word(fpb::FP_COMP0 + 4 * i)? == value {
                self.write_word(fpb::FP_COMP0 + 4 * i, 0)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Whether an enabled FPB comparator covers `addr` (used to distinguish
    /// hardware from software breakpoints when reporting a stop).
    pub fn hw_breakpoint_at(&mut self, addr: u32) -> Result<bool, ArmError> {
        let rev = self.fpb_rev()?;
        let Some(value) = Self::fpb_comp_value(addr, rev) else {
            return Ok(false);
        };
        let n = self.fpb_num_comps()?;
        for i in 0..n {
            if self.read_word(fpb::FP_COMP0 + 4 * i)? == value {
                return Ok(true);
            }
        }
        Ok(false)
    }

    fn dwt_num_comps(&mut self) -> Result<u32, ArmError> {
        Ok((self.read_word(dwt::DWT_CTRL)? >> 28) & 0xF)
    }

    /// Set a data watchpoint. `len` must be a power of two and `addr` aligned
    /// to it (DWT comparators mask low address bits). Returns false if the
    /// parameters are unsupported or no comparator is free.
    pub fn watchpoint_set(
        &mut self,
        addr: u32,
        len: u32,
        access: WatchAccess,
    ) -> Result<bool, ArmError> {
        if len == 0 || !len.is_power_of_two() || addr & (len - 1) != 0 {
            return Ok(false);
        }
        let n = self.dwt_num_comps()?;
        if n == 0 {
            return Ok(false);
        }
        // The DWT unit only raises debug events with DEMCR.DWTENA set.
        let demcr = self.read_word(cortex_m::DEMCR)?;
        self.write_word(cortex_m::DEMCR, demcr | cortex_m::DEMCR_DWTENA)?;
        for i in 0..n {
            let function = self.read_word(dwt::FUNCTION0 + 0x10 * i)?;
            if function & 0xF == 0 {
                self.write_word(dwt::COMP0 + 0x10 * i, addr)?;
                self.write_word(dwt::MASK0 + 0x10 * i, len.trailing_zeros())?;
                self.write_word(dwt::FUNCTION0 + 0x10 * i, access.function())?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Clear the watchpoint matching `addr`/`len`/`access`. Returns false if
    /// none matches.
    pub fn watchpoint_clear(
        &mut self,
        addr: u32,
        len: u32,
        access: WatchAccess,
    ) -> Result<bool, ArmError> {
        if len == 0 || !len.is_power_of_two() {
            return Ok(false);
        }
        let n = self.dwt_num_comps()?;
        for i in 0..n {
            let function = self.read_word(dwt::FUNCTION0 + 0x10 * i)?;
            if function & 0xF == access.function()
                && self.read_word(dwt::COMP0 + 0x10 * i)? == addr
                && self.read_word(dwt::MASK0 + 0x10 * i)? == len.trailing_zeros()
            {
                self.write_word(dwt::FUNCTION0 + 0x10 * i, 0)?;
                return Ok(true);
            }
        }
        Ok(false)
    }

    /// Disable every FPB comparator and DWT watchpoint. Comparators live in
    /// target hardware, so breakpoints left by a dead debug session survive
    /// into the next one; call this when (re)attaching for a clean slate.
    pub fn debug_units_clear(&mut self) -> Result<(), ArmError> {
        let n = self.fpb_num_comps()?;
        for i in 0..n {
            self.write_word(fpb::FP_COMP0 + 4 * i, 0)?;
        }
        let n = self.dwt_num_comps()?;
        for i in 0..n {
            self.write_word(dwt::FUNCTION0 + 0x10 * i, 0)?;
        }
        Ok(())
    }

    /// Decode (and clear) why the core halted. For a watchpoint stop, the
    /// matched comparator supplies the data address and access kind.
    pub fn halt_reason(&mut self) -> Result<HaltReason, ArmError> {
        let dfsr = self.read_word(cortex_m::DFSR)?;
        self.write_word(cortex_m::DFSR, dfsr)?; // write-1-to-clear
        if dfsr & cortex_m::DFSR_DWTTRAP != 0 {
            let n = self.dwt_num_comps()?;
            for i in 0..n {
                let function = self.read_word(dwt::FUNCTION0 + 0x10 * i)?;
                if function & dwt::FUNC_MATCHED != 0 {
                    let addr = self.read_word(dwt::COMP0 + 0x10 * i)?;
                    let access = match function & 0xF {
                        dwt::FUNC_READ => WatchAccess::Read,
                        dwt::FUNC_WRITE => WatchAccess::Write,
                        _ => WatchAccess::ReadWrite,
                    };
                    return Ok(HaltReason::Watchpoint(addr, access));
                }
            }
            return Ok(HaltReason::Watchpoint(0, WatchAccess::ReadWrite));
        }
        if dfsr & cortex_m::DFSR_BKPT != 0 {
            return Ok(HaltReason::Breakpoint);
        }
        if dfsr & cortex_m::DFSR_VCATCH != 0 {
            return Ok(HaltReason::VectorCatch);
        }
        if dfsr & cortex_m::DFSR_HALTED != 0 {
            return Ok(HaltReason::HaltRequest);
        }
        Ok(HaltReason::Unknown)
    }
}

/// Convenience: what port the transport last reported active (for callers that
/// also drive it directly).
pub fn active_is_swd(port: Option<ActivePort>) -> bool {
    matches!(port, Some(ActivePort::Swd))
}

#[cfg(test)]
mod test;
