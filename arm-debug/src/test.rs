// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! Host tests: drive `ArmDebug` against a mock `DapTransport` that records the
//! exact SWD transactions and plays back scripted read results, so the ADIv5
//! DP/AP protocol (posted reads, SELECT bank caching, CSW-once, power-up
//! polling, MEM-AP addressing) is verified without hardware.

use super::*;
use rust_dap::{
    ActivePort, ConnectPort, DapCapabilities, DapConfig, DapError, DapTransport, JtagSequenceInfo,
    SwdRequest, SwjPins,
};
use std::vec::Vec;

/// One recorded SWD transfer.
#[derive(Debug, PartialEq, Eq, Clone)]
struct Xfer {
    ap: bool,
    rnw: bool,
    addr: u8,
    data: u32,
}

struct MockSwd {
    log: Vec<Xfer>,
    /// Scripted return values for reads, consumed front-to-back.
    read_results: Vec<u32>,
    read_index: usize,
    /// Bytes captured from swd_write_bits (for the TARGETSEL test).
    written_bits: Vec<(usize, Vec<u8>)>,
    connected_swd: bool,
}

impl MockSwd {
    fn new(read_results: &[u32]) -> Self {
        Self {
            log: Vec::new(),
            read_results: read_results.to_vec(),
            read_index: 0,
            written_bits: Vec::new(),
            connected_swd: false,
        }
    }
    fn next_read(&mut self) -> u32 {
        let v = *self.read_results.get(self.read_index).unwrap_or(&0);
        self.read_index += 1;
        v
    }
}

impl DapTransport for MockSwd {
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::SWD
    }
    fn connect(&mut self, port: ConnectPort, _c: &DapConfig) -> Result<ActivePort, DapError> {
        assert!(matches!(port, ConnectPort::Swd | ConnectPort::Default));
        self.connected_swd = true;
        Ok(ActivePort::Swd)
    }
    fn disconnect(&mut self, _c: &DapConfig) -> Result<(), DapError> {
        Ok(())
    }
    fn swj_sequence(&mut self, _c: &DapConfig, _n: usize, _d: &[u8]) -> Result<(), DapError> {
        Ok(())
    }
    fn swj_pins(
        &mut self,
        _c: &DapConfig,
        _o: SwjPins,
        _s: SwjPins,
        _w: u32,
    ) -> Result<SwjPins, DapError> {
        Ok(SwjPins::empty())
    }
    fn swj_clock(&mut self, _c: &mut DapConfig, _hz: u32) -> Result<(), DapError> {
        Ok(())
    }
    fn swd_write_bits(
        &mut self,
        _c: &DapConfig,
        count: usize,
        data: &[u8],
    ) -> Result<(), DapError> {
        let bytes = count.div_ceil(8).min(data.len());
        self.written_bits.push((count, data[..bytes].to_vec()));
        Ok(())
    }
    fn swd_output_enable(&mut self, _enable: bool) -> Result<(), DapError> {
        Ok(())
    }
    fn swd_transfer(
        &mut self,
        _c: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        let ap = request.contains(SwdRequest::APnDP);
        let rnw = request.contains(SwdRequest::RnW);
        let addr = ((request.contains(SwdRequest::A3) as u8) << 3)
            | ((request.contains(SwdRequest::A2) as u8) << 2);
        self.log.push(Xfer {
            ap,
            rnw,
            addr,
            data,
        });
        Ok(if rnw { self.next_read() } else { 0 })
    }
    fn jtag_transfer(
        &mut self,
        _c: &DapConfig,
        _i: u8,
        _r: SwdRequest,
        _d: u32,
    ) -> Result<u32, DapError> {
        Err(DapError::NotSupported)
    }
    fn jtag_sequence(
        &mut self,
        _c: &DapConfig,
        _i: &JtagSequenceInfo,
        _t: u64,
    ) -> Result<Option<u64>, DapError> {
        Err(DapError::NotSupported)
    }
}

fn dp_r(addr: u8, data: u32) -> Xfer {
    Xfer {
        ap: false,
        rnw: true,
        addr,
        data,
    }
}
fn dp_w(addr: u8, data: u32) -> Xfer {
    Xfer {
        ap: false,
        rnw: false,
        addr,
        data,
    }
}
fn ap_r(addr: u8) -> Xfer {
    Xfer {
        ap: true,
        rnw: true,
        addr,
        data: 0,
    }
}
fn ap_w(addr: u8, data: u32) -> Xfer {
    Xfer {
        ap: true,
        rnw: false,
        addr,
        data,
    }
}

#[test]
fn read_word_issues_correct_adiv5_sequence() {
    // CSW write, TAR write, posted DRW read (stale), RDBUFF read (real value).
    let mock = MockSwd::new(&[
        0xdead_beef, /* stale DRW */
        0x2000_2927, /* RDBUFF */
    ]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let value = arm.read_word(0x4000_0000).unwrap();
    assert_eq!(value, 0x2000_2927);

    let log = &arm.transport().log;
    assert_eq!(
        log,
        &[
            dp_w(DP_SELECT, 0),        // select AP bank 0 (from initial 0xffffffff)
            ap_w(AP_CSW, CSW_DEFAULT), // CSW programmed once
            ap_w(AP_TAR, 0x4000_0000), // target address
            ap_r(AP_DRW),              // posted read
            dp_r(DP_RDBUFF, 0),        // real value fetched here
        ]
    );
}

#[test]
fn csw_and_select_are_cached() {
    // Two reads in a row: CSW and SELECT must not be re-programmed.
    let mock = MockSwd::new(&[0, 0x11, 0, 0x22]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    assert_eq!(arm.read_word(0x2000_0000).unwrap(), 0x11);
    assert_eq!(arm.read_word(0x2000_0004).unwrap(), 0x22);

    let log = &arm.transport().log;
    // Only one SELECT and one CSW across both reads.
    assert_eq!(log.iter().filter(|x| *x == &dp_w(DP_SELECT, 0)).count(), 1);
    assert_eq!(
        log.iter()
            .filter(|x| *x == &ap_w(AP_CSW, CSW_DEFAULT))
            .count(),
        1
    );
    // Two TAR writes with the two addresses.
    assert!(log.contains(&ap_w(AP_TAR, 0x2000_0000)));
    assert!(log.contains(&ap_w(AP_TAR, 0x2000_0004)));
}

#[test]
fn write_word_flushes_via_rdbuff() {
    let mock = MockSwd::new(&[0 /* RDBUFF flush */]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    arm.write_word(0x2000_0100, 0x1234_5678).unwrap();
    let log = &arm.transport().log;
    assert_eq!(
        log,
        &[
            dp_w(DP_SELECT, 0),
            ap_w(AP_CSW, CSW_DEFAULT),
            ap_w(AP_TAR, 0x2000_0100),
            ap_w(AP_DRW, 0x1234_5678),
            dp_r(DP_RDBUFF, 0), // flush
        ]
    );
}

#[test]
fn read_words_uses_tar_autoincrement() {
    // 3 words: TAR once, then post + (n-1) DRW reads returning stale/prev, and
    // RDBUFF for the last. Reads return: w0 arrives on 2nd DRW, w1 on 3rd DRW,
    // w2 on RDBUFF.
    let mock = MockSwd::new(&[0xaaaa, 0x1111, 0x2222, 0x3333]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let mut buf = [0u32; 3];
    arm.read_words(0x2000_0000, &mut buf).unwrap();
    assert_eq!(buf, [0x1111, 0x2222, 0x3333]);

    let log = &arm.transport().log;
    // Exactly one TAR write; DRW reads = 1 (post) + 2 (loop) = 3; one RDBUFF.
    assert_eq!(
        log.iter()
            .filter(|x| *x == &ap_w(AP_TAR, 0x2000_0000))
            .count(),
        1
    );
    assert_eq!(
        log.iter()
            .filter(|x| x.ap && x.rnw && x.addr == AP_DRW)
            .count(),
        3
    );
    assert_eq!(log.iter().filter(|x| *x == &dp_r(DP_RDBUFF, 0)).count(), 1);
}

#[test]
fn power_up_polls_ctrl_stat() {
    // connect_multidrop: DPIDR read, ABORT, SELECT, CTRL/STAT write, then poll.
    // Script: DPIDR, then two CTRL/STAT reads (first not acked, second acked).
    let acked = CDBGPWRUPACK | CSYSPWRUPACK;
    let mock = MockSwd::new(&[
        0x0bc1_1477, /* DPIDR */
        0,           /* stat not acked */
        acked,
    ]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let dpidr = arm.connect_multidrop(rp2040::CORE0_TARGETSEL).unwrap();
    assert_eq!(dpidr, 0x0bc1_1477);

    let log = &arm.transport().log;
    assert!(log.contains(&dp_r(DP_DPIDR, 0)));
    assert!(log.contains(&dp_w(DP_ABORT, ABORT_CLEAR_ALL)));
    assert!(log.contains(&dp_w(DP_CTRL_STAT, CDBGPWRUPREQ | CSYSPWRUPREQ)));
    // Two CTRL/STAT reads before ack.
    assert_eq!(
        log.iter().filter(|x| *x == &dp_r(DP_CTRL_STAT, 0)).count(),
        2
    );
}

#[test]
fn no_target_when_dpidr_invalid() {
    let mock = MockSwd::new(&[0xffff_ffff /* bad DPIDR */]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    assert_eq!(
        arm.connect_multidrop(rp2040::CORE0_TARGETSEL),
        Err(ArmError::NoTarget)
    );
}

#[test]
fn targetsel_written_without_ack_and_correct_value() {
    let mock = MockSwd::new(&[0x0bc1_1477, CDBGPWRUPACK | CSYSPWRUPACK]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    arm.connect_multidrop(rp2040::CORE0_TARGETSEL).unwrap();

    let writes = &arm.transport().written_bits;
    // Expect: request(8) + ack-skip(5) + value(32) + parity(1).
    let value_write = writes
        .iter()
        .find(|(n, _)| *n == 32)
        .expect("32-bit value write");
    assert_eq!(
        u32::from_le_bytes(value_write.1[..4].try_into().unwrap()),
        rp2040::CORE0_TARGETSEL
    );
    // The parity bit matches the value's popcount parity.
    let parity_write = writes
        .iter()
        .rev()
        .find(|(n, _)| *n == 1)
        .expect("parity bit");
    assert_eq!(
        parity_write.1[0] & 1,
        (rp2040::CORE0_TARGETSEL.count_ones() & 1) as u8
    );
    // No swd_transfer happened for TARGETSEL itself (first transfer is DPIDR read).
    assert_eq!(arm.transport().log.first(), Some(&dp_r(DP_DPIDR, 0)));
}

#[test]
fn reselect_switches_target_without_dormant_dance() {
    let acked = CDBGPWRUPACK | CSYSPWRUPACK;
    let mock = MockSwd::new(&[
        0x0bc1_2477, /* DPIDR core0 */
        acked,
        0x0bc1_2477, /* DPIDR core1 */
        acked,
    ]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    arm.connect_multidrop(rp2040::CORE0_TARGETSEL).unwrap();
    let before = arm.transport().written_bits.len();
    let dpidr = arm.reselect(rp2040::CORE1_TARGETSEL).unwrap();
    assert_eq!(dpidr, 0x0bc1_2477);
    // The TARGETSEL raw-bit write ran again (request + value + parity).
    let writes = &arm.transport().written_bits[before..];
    let value_write = writes
        .iter()
        .find(|(n, _)| *n == 32)
        .expect("TARGETSEL value");
    assert_eq!(
        u32::from_le_bytes(value_write.1[..4].try_into().unwrap()),
        rp2040::CORE1_TARGETSEL
    );
    // SELECT re-initialized and errors cleared on the new DP.
    let log = &arm.transport().log;
    assert_eq!(log.iter().filter(|x| *x == &dp_w(DP_SELECT, 0)).count(), 2);
}

#[test]
fn connect_swd_reads_dpidr_without_targetsel() {
    // nRF52 SW-DP: no TARGETSEL raw-bit write, DPIDR read first.
    let acked = CDBGPWRUPACK | CSYSPWRUPACK;
    let mock = MockSwd::new(&[nrf52::DPIDR, acked]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let dpidr = arm.connect_swd().unwrap();
    assert_eq!(dpidr, nrf52::DPIDR);
    // First transfer is a DPIDR read; no 32-bit TARGETSEL value was written.
    assert_eq!(arm.transport().log.first(), Some(&dp_r(DP_DPIDR, 0)));
    assert!(arm.transport().written_bits.iter().all(|(n, _)| *n != 32));
}

#[test]
fn apsel_selects_ctrl_ap_and_invalidates_csw() {
    // Reading CTRL-AP.APPROTECTSTATUS must set SELECT.APSEL=1, and switching
    // back to APSEL 0 must re-program CSW on the next MEM-AP access.
    let mock = MockSwd::new(&[
        0,                       // stale AP read (IDR posted)
        nrf52::CTRLAP_IDR_VALUE, // RDBUFF: IDR
        0,                       // stale AP read (APPROTECTSTATUS posted)
        1,                       // RDBUFF: unprotected
        0,
        0x42, // later MEM-AP read: stale, then value
    ]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let (unprotected, idr_ok) = arm.nrf52_approtect_status().unwrap();
    assert!(unprotected);
    assert!(idr_ok);
    let log = &arm.transport().log;
    // SELECT was written with APSEL=1 (bit24) at some point.
    assert!(log.contains(&dp_w(DP_SELECT, 1 << 24)));
    // Back on APSEL 0, a MEM-AP read re-programs CSW (was invalidated).
    let before = arm
        .transport()
        .log
        .iter()
        .filter(|x| *x == &ap_w(AP_CSW, CSW_DEFAULT))
        .count();
    arm.read_word(0x2000_0000).unwrap();
    let after = arm
        .transport()
        .log
        .iter()
        .filter(|x| *x == &ap_w(AP_CSW, CSW_DEFAULT))
        .count();
    assert_eq!(after, before + 1);
}

#[test]
fn nrf52_approtect_reports_protected() {
    let mock = MockSwd::new(&[
        0,
        nrf52::CTRLAP_IDR_VALUE, // IDR
        0,
        0, // APPROTECTSTATUS bit0 = 0 → protected
    ]);
    let mut arm = ArmDebug::new(mock, DapConfig::default());
    let (unprotected, idr_ok) = arm.nrf52_approtect_status().unwrap();
    assert!(!unprotected);
    assert!(idr_ok);
}

#[test]
fn nrf52_flash_erase_and_program() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    // Erase a page: CONFIG=EEN, ERASEPAGE=base, CONFIG=REN.
    arm.nrf52_erase_page(0x0000_1234, 16).unwrap();
    let log_mem = |arm: &mut ArmDebug<MemMock>, a: u32| arm.read_word(a).unwrap();
    assert_eq!(log_mem(&mut arm, nrf52::NVMC_ERASEPAGE), 0x0000_1000); // page base
    assert_eq!(
        log_mem(&mut arm, nrf52::NVMC_CONFIG),
        nrf52::NVMC_CONFIG_REN
    );
    // Program words, then read them back through the MEM-AP.
    let data = [0x1111_2222u32, 0x3333_4444, 0x5555_6666];
    arm.nrf52_program(0x0000_1000, &data, 16).unwrap();
    assert_eq!(arm.read_word(0x0000_1000).unwrap(), 0x1111_2222);
    assert_eq!(arm.read_word(0x0000_1004).unwrap(), 0x3333_4444);
    assert_eq!(arm.read_word(0x0000_1008).unwrap(), 0x5555_6666);
    // Left in read-only mode.
    assert_eq!(
        arm.read_word(nrf52::NVMC_CONFIG).unwrap(),
        nrf52::NVMC_CONFIG_REN
    );
}

#[test]
fn wait_ack_retries_then_gives_up() {
    struct AlwaysWait;
    impl DapTransport for AlwaysWait {
        fn capabilities(&self) -> DapCapabilities {
            DapCapabilities::SWD
        }
        fn connect(&mut self, _p: ConnectPort, _c: &DapConfig) -> Result<ActivePort, DapError> {
            Ok(ActivePort::Swd)
        }
        fn disconnect(&mut self, _c: &DapConfig) -> Result<(), DapError> {
            Ok(())
        }
        fn swj_sequence(&mut self, _c: &DapConfig, _n: usize, _d: &[u8]) -> Result<(), DapError> {
            Ok(())
        }
        fn swj_pins(
            &mut self,
            _c: &DapConfig,
            _o: SwjPins,
            _s: SwjPins,
            _w: u32,
        ) -> Result<SwjPins, DapError> {
            Ok(SwjPins::empty())
        }
        fn swj_clock(&mut self, _c: &mut DapConfig, _hz: u32) -> Result<(), DapError> {
            Ok(())
        }
        fn swd_transfer(
            &mut self,
            _c: &DapConfig,
            _r: SwdRequest,
            _d: u32,
        ) -> Result<u32, DapError> {
            Err(DapError::SwdError(DAP_TRANSFER_WAIT))
        }
    }
    let mut arm = ArmDebug::new(AlwaysWait, DapConfig::default());
    assert_eq!(arm.read_word(0x2000_0000), Err(ArmError::Wait));
}

////////////////////////////////////////////////////////////////////////////
// M2: Cortex-M core-debug tests, using a memory-backed mock that models the
// MEM-AP posted-read pipeline plus DHCSR halt / DCRSR core-register behavior.
////////////////////////////////////////////////////////////////////////////

use super::cortex_m as cm;
use std::collections::HashMap;

struct MemMock {
    mem: HashMap<u32, u32>,
    core_regs: [u32; 20],
    halted: bool,
    /// Simulate an instant BKPT: a resume (halt cleared) re-halts at once.
    auto_rehalt: bool,
    // MEM-AP pipeline state.
    tar: u32,
    pending: u32, // value a posted AP read will surface next
    // DCRSR latch for the pending core-register transfer.
    dcrdr: u32,
}

impl MemMock {
    fn new() -> Self {
        Self {
            mem: HashMap::new(),
            core_regs: [0; 20],
            halted: false,
            auto_rehalt: false,
            tar: 0,
            pending: 0,
            dcrdr: 0,
        }
    }

    fn read_mem(&mut self, addr: u32) -> u32 {
        match addr {
            cm::DHCSR => {
                let mut v = cm::C_DEBUGEN | cm::S_REGRDY; // regrdy always ready in the mock
                if self.halted {
                    v |= cm::S_HALT;
                }
                v
            }
            cm::DCRDR => self.dcrdr,
            nrf52::NVMC_READY => 1,  // NVMC always ready in the mock
            nrf54::RRAMC_READY => 1, // RRAMC always ready in the mock
            other => *self.mem.get(&other).unwrap_or(&0),
        }
    }

    fn write_mem(&mut self, addr: u32, val: u32) {
        match addr {
            cm::DHCSR => {
                // Only honor writes carrying the debug key.
                if val & 0xFFFF_0000 == cm::DBGKEY {
                    self.halted = val & cm::C_HALT != 0
                        || (val & cm::C_STEP != 0)
                        || (self.auto_rehalt && val & cm::C_DEBUGEN != 0);
                }
            }
            cm::DCRDR => self.dcrdr = val,
            cm::DCRSR => {
                let reg = (val & 0x7f) as usize;
                if val & cm::DCRSR_REGWNR != 0 {
                    // Register write: value came from DCRDR.
                    if reg < self.core_regs.len() {
                        self.core_regs[reg] = self.dcrdr;
                    }
                } else {
                    // Register read: latch value into DCRDR.
                    self.dcrdr = *self.core_regs.get(reg).unwrap_or(&0);
                }
            }
            other => {
                self.mem.insert(other, val);
            }
        }
    }
}

impl DapTransport for MemMock {
    fn capabilities(&self) -> DapCapabilities {
        DapCapabilities::SWD
    }
    fn connect(&mut self, _p: ConnectPort, _c: &DapConfig) -> Result<ActivePort, DapError> {
        Ok(ActivePort::Swd)
    }
    fn disconnect(&mut self, _c: &DapConfig) -> Result<(), DapError> {
        Ok(())
    }
    fn swj_sequence(&mut self, _c: &DapConfig, _n: usize, _d: &[u8]) -> Result<(), DapError> {
        Ok(())
    }
    fn swj_pins(
        &mut self,
        _c: &DapConfig,
        _o: SwjPins,
        _s: SwjPins,
        _w: u32,
    ) -> Result<SwjPins, DapError> {
        Ok(SwjPins::empty())
    }
    fn swj_clock(&mut self, _c: &mut DapConfig, _hz: u32) -> Result<(), DapError> {
        Ok(())
    }
    fn swd_write_bits(&mut self, _c: &DapConfig, _n: usize, _d: &[u8]) -> Result<(), DapError> {
        Ok(())
    }
    fn swd_output_enable(&mut self, _e: bool) -> Result<(), DapError> {
        Ok(())
    }
    fn swd_transfer(
        &mut self,
        _c: &DapConfig,
        request: SwdRequest,
        data: u32,
    ) -> Result<u32, DapError> {
        let ap = request.contains(SwdRequest::APnDP);
        let rnw = request.contains(SwdRequest::RnW);
        let addr = ((request.contains(SwdRequest::A3) as u8) << 3)
            | ((request.contains(SwdRequest::A2) as u8) << 2);
        if ap {
            match (rnw, addr) {
                (false, AP_CSW) => Ok(0),
                (false, AP_TAR) => {
                    self.tar = data;
                    Ok(0)
                }
                (false, AP_DRW) => {
                    self.write_mem(self.tar, data);
                    self.tar = self.tar.wrapping_add(4);
                    Ok(0)
                }
                (true, AP_DRW) => {
                    // Posted read: return the previously-pending value, latch
                    // the current location as the new pending value.
                    let out = self.pending;
                    self.pending = self.read_mem(self.tar);
                    self.tar = self.tar.wrapping_add(4);
                    Ok(out)
                }
                _ => Ok(0),
            }
        } else {
            match (rnw, addr) {
                (false, _) => Ok(0),             // DP writes (SELECT etc.)
                (true, 0xC) => Ok(self.pending), // RDBUFF surfaces the pending read
                (true, _) => Ok(0),
            }
        }
    }
    fn jtag_transfer(
        &mut self,
        _c: &DapConfig,
        _i: u8,
        _r: SwdRequest,
        _d: u32,
    ) -> Result<u32, DapError> {
        Err(DapError::NotSupported)
    }
    fn jtag_sequence(
        &mut self,
        _c: &DapConfig,
        _i: &JtagSequenceInfo,
        _t: u64,
    ) -> Result<Option<u64>, DapError> {
        Err(DapError::NotSupported)
    }
}

#[test]
fn memmock_read_write_roundtrip() {
    // Sanity: the memory-backed mock models the posted-read pipeline correctly.
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.write_word(0x2000_0000, 0xcafe_babe).unwrap();
    arm.write_word(0x2000_0004, 0x1234_5678).unwrap();
    assert_eq!(arm.read_word(0x2000_0000).unwrap(), 0xcafe_babe);
    assert_eq!(arm.read_word(0x2000_0004).unwrap(), 0x1234_5678);
    let mut buf = [0u32; 2];
    arm.read_words(0x2000_0000, &mut buf).unwrap();
    assert_eq!(buf, [0xcafe_babe, 0x1234_5678]);
}

#[test]
fn halt_resume_and_status() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    assert!(!arm.is_halted().unwrap());
    arm.halt().unwrap();
    assert!(arm.is_halted().unwrap());
    arm.resume().unwrap();
    assert!(!arm.is_halted().unwrap());
}

#[test]
fn step_leaves_core_halted() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.halt().unwrap();
    arm.step().unwrap();
    assert!(arm.is_halted().unwrap());
}

#[test]
fn core_register_read_write() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.halt().unwrap();
    // Preload PC and xPSR in the mock via write_core_reg, then read back.
    arm.write_core_reg(cm::PC, 0x1000_0100).unwrap();
    arm.write_core_reg(cm::XPSR, 0x0100_0000).unwrap();
    arm.write_core_reg(cm::R0, 0xdead_0000).unwrap();
    assert_eq!(arm.read_core_reg(cm::PC).unwrap(), 0x1000_0100);
    assert_eq!(arm.read_core_reg(cm::XPSR).unwrap(), 0x0100_0000);
    assert_eq!(arm.read_core_reg(cm::R0).unwrap(), 0xdead_0000);
}

#[test]
fn dhcsr_write_requires_dbgkey() {
    // A DHCSR write without the debug key must be ignored (mock models this):
    // read_core_reg / halt drive DHCSR only with the key, so halting works;
    // here we assert the mock itself rejects a keyless halt attempt via a
    // direct write_word, proving our real code's key discipline matters.
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.write_word(cm::DHCSR, cm::C_DEBUGEN | cm::C_HALT)
        .unwrap(); // no key
    assert!(!arm.is_halted().unwrap());
    arm.halt().unwrap(); // real code supplies DBGKEY
    assert!(arm.is_halted().unwrap());
}

////////////////////////////////////////////////////////////////////////////
// M5: bootrom function-table lookup and remote function calls.
////////////////////////////////////////////////////////////////////////////

#[test]
fn rom_func_lookup_walks_table() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    // Table pointer word: func table at 0x7a0 (low half), data table high.
    arm.write_word(rp2040::ROM_TABLE_PTRS, 0x0800_07a0).unwrap();
    // Entries: ("IF", 0x2345), ("RE", 0x3456), terminator.
    arm.write_word(0x7a0, (0x2345 << 16) | u16::from_le_bytes(*b"IF") as u32)
        .unwrap();
    arm.write_word(0x7a4, (0x3456 << 16) | u16::from_le_bytes(*b"RE") as u32)
        .unwrap();
    arm.write_word(0x7a8, 0).unwrap();
    assert_eq!(
        arm.rom_func_lookup(rp2040::FN_CONNECT_INTERNAL_FLASH)
            .unwrap(),
        Some(0x2345)
    );
    assert_eq!(
        arm.rom_func_lookup(rp2040::FN_FLASH_RANGE_ERASE).unwrap(),
        Some(0x3456)
    );
    assert_eq!(
        arm.rom_func_lookup(rp2040::FN_FLASH_ENTER_CMD_XIP).unwrap(),
        None
    );
}

#[test]
fn call_function_returns_r0_and_restores_registers() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.transport().auto_rehalt = true; // resume "hits" the BKPT instantly
    arm.halt().unwrap();
    // Debugger-visible register state that must survive the call.
    arm.write_core_reg(cm::R0, 0x1111).unwrap();
    arm.write_core_reg(cm::SP, 0x2000_3000).unwrap();
    arm.write_core_reg(cm::PC, 0x2000_0400).unwrap();
    let r0 = arm
        .call_function(0x0000_2345, &[7, 8, 9, 10], 0x2000_8000, 0x2000_7000, 16)
        .unwrap();
    assert_eq!(r0, 7); // mock function body does nothing, r0 = arg0
                       // BKPT pair planted at the trampoline.
    assert_eq!(arm.read_word(0x2000_7000).unwrap(), 0xBE00_BE00);
    // Core left halted with the original register state restored.
    assert!(arm.is_halted().unwrap());
    assert_eq!(arm.read_core_reg(cm::R0).unwrap(), 0x1111);
    assert_eq!(arm.read_core_reg(cm::SP).unwrap(), 0x2000_3000);
    assert_eq!(arm.read_core_reg(cm::PC).unwrap(), 0x2000_0400);
}

////////////////////////////////////////////////////////////////////////////
// M4: FPB hardware breakpoints, DWT watchpoints, DFSR halt reason.
////////////////////////////////////////////////////////////////////////////

/// MemMock with FPB (4 comparators) and DWT (2 comparators) advertised.
fn arm_with_debug_units() -> ArmDebug<MemMock> {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.write_word(fpb::FP_CTRL, 4 << 4).unwrap(); // NUM_CODE = 4
    arm.write_word(dwt::DWT_CTRL, 2 << 28).unwrap(); // NUMCOMP = 2
    arm
}

#[test]
fn hw_breakpoint_encodes_fpb_v2_when_rev_is_1() {
    // FPBv2 (Cortex-M7 / ARMv8-M, e.g. nRF54 M33): FP_CTRL.REV = 1, and any
    // address is breakable as BPADDR[31:1] | BE.
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    arm.write_word(fpb::FP_CTRL, (1 << 28) | (2 << 4)).unwrap(); // REV=1, NUM_CODE=2
                                                                 // A flash address that FPBv1 would reject (>= 0x2000_0000 in v1 terms is
                                                                 // fine here; use one to prove the region check is bypassed).
    assert!(arm.hw_breakpoint_set(0x2000_1236).unwrap());
    assert_eq!(
        arm.read_word(fpb::FP_COMP0).unwrap(),
        (0x2000_1236 & 0xFFFF_FFFE) | fpb::COMP_ENABLE
    );
    assert!(arm.hw_breakpoint_at(0x2000_1236).unwrap());
    assert!(arm.hw_breakpoint_clear(0x2000_1236).unwrap());
    assert_eq!(arm.read_word(fpb::FP_COMP0).unwrap(), 0);
}

#[test]
fn nrf54_rram_program_via_commit() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    let data = [0xaabb_ccddu32, 0x1122_3344];
    arm.nrf54_program(0x0000_2000, &data, 16).unwrap();
    // CONFIG left at 0 (write disabled) after commit.
    assert_eq!(arm.read_word(nrf54::RRAMC_CONFIG).unwrap(), 0);
    // Commit task was written.
    assert_eq!(arm.read_word(nrf54::RRAMC_COMMIT).unwrap(), 1);
    // Data landed in RRAM via the MEM-AP.
    assert_eq!(arm.read_word(0x0000_2000).unwrap(), 0xaabb_ccdd);
    assert_eq!(arm.read_word(0x0000_2004).unwrap(), 0x1122_3344);
}

#[test]
fn hw_breakpoint_set_encodes_fpb_v1_comp() {
    let mut arm = arm_with_debug_units();
    // Lower halfword (addr bit1 = 0) → REPLACE = 01.
    assert!(arm.hw_breakpoint_set(0x0000_1794).unwrap());
    assert_eq!(
        arm.read_word(fpb::FP_COMP0).unwrap(),
        (0b01 << 30) | 0x0000_1794 | fpb::COMP_ENABLE
    );
    // Upper halfword (addr bit1 = 1) → REPLACE = 10, word-aligned COMP.
    assert!(arm.hw_breakpoint_set(0x1000_0002).unwrap());
    assert_eq!(
        arm.read_word(fpb::FP_COMP0 + 4).unwrap(),
        (0b10u32 << 30) | 0x1000_0000 | fpb::COMP_ENABLE
    );
    // FP_CTRL got enabled with the key.
    assert_eq!(
        arm.read_word(fpb::FP_CTRL).unwrap() & (fpb::CTRL_KEY | fpb::CTRL_ENABLE),
        fpb::CTRL_KEY | fpb::CTRL_ENABLE
    );
    // Setting the same breakpoint twice does not burn a second comparator.
    assert!(arm.hw_breakpoint_set(0x0000_1794).unwrap());
    assert_eq!(arm.read_word(fpb::FP_COMP0 + 8).unwrap(), 0);
    assert!(arm.hw_breakpoint_at(0x0000_1794).unwrap());
}

#[test]
fn hw_breakpoint_rejects_non_code_region_and_exhaustion() {
    let mut arm = arm_with_debug_units();
    // FPB v1 cannot break above 0x2000_0000 (RAM) — SW breakpoints do that.
    assert!(!arm.hw_breakpoint_set(0x2000_0000).unwrap());
    for i in 0..4 {
        assert!(arm.hw_breakpoint_set(0x100 + 4 * i).unwrap());
    }
    assert!(!arm.hw_breakpoint_set(0x1000).unwrap()); // all comparators busy
                                                      // Clearing frees the comparator for reuse.
    assert!(arm.hw_breakpoint_clear(0x104).unwrap());
    assert!(!arm.hw_breakpoint_clear(0x104).unwrap()); // already gone
    assert!(arm.hw_breakpoint_set(0x1000).unwrap());
}

#[test]
fn watchpoint_set_programs_dwt_and_dwtena() {
    let mut arm = arm_with_debug_units();
    assert!(arm
        .watchpoint_set(0x2000_1000, 4, WatchAccess::Write)
        .unwrap());
    assert_eq!(arm.read_word(dwt::COMP0).unwrap(), 0x2000_1000);
    assert_eq!(arm.read_word(dwt::MASK0).unwrap(), 2); // log2(4)
    assert_eq!(arm.read_word(dwt::FUNCTION0).unwrap(), dwt::FUNC_WRITE);
    assert_eq!(
        arm.read_word(cm::DEMCR).unwrap() & cm::DEMCR_DWTENA,
        cm::DEMCR_DWTENA
    );
    // Unaligned / non-power-of-two rejected.
    assert!(!arm
        .watchpoint_set(0x2000_1001, 4, WatchAccess::Read)
        .unwrap());
    assert!(!arm
        .watchpoint_set(0x2000_1000, 3, WatchAccess::Read)
        .unwrap());
    // Second comparator, then exhaustion.
    assert!(arm
        .watchpoint_set(0x2000_2000, 1, WatchAccess::Read)
        .unwrap());
    assert!(!arm
        .watchpoint_set(0x2000_3000, 4, WatchAccess::ReadWrite)
        .unwrap());
    // Clear requires matching parameters.
    assert!(!arm
        .watchpoint_clear(0x2000_1000, 4, WatchAccess::Read)
        .unwrap());
    assert!(arm
        .watchpoint_clear(0x2000_1000, 4, WatchAccess::Write)
        .unwrap());
    assert!(arm
        .watchpoint_set(0x2000_3000, 4, WatchAccess::ReadWrite)
        .unwrap());
}

#[test]
fn halt_reason_decodes_and_clears_dfsr() {
    let mut arm = arm_with_debug_units();
    // Breakpoint.
    arm.write_word(cm::DFSR, cm::DFSR_BKPT | cm::DFSR_HALTED)
        .unwrap();
    assert_eq!(arm.halt_reason().unwrap(), HaltReason::Breakpoint);
    // Watchpoint: DWTTRAP + comparator 1 flagged as matched.
    arm.write_word(cm::DFSR, cm::DFSR_DWTTRAP).unwrap();
    arm.write_word(dwt::COMP0 + 0x10, 0x2000_4000).unwrap();
    arm.write_word(dwt::FUNCTION0 + 0x10, dwt::FUNC_MATCHED | dwt::FUNC_WRITE)
        .unwrap();
    assert_eq!(
        arm.halt_reason().unwrap(),
        HaltReason::Watchpoint(0x2000_4000, WatchAccess::Write)
    );
    // Plain halt request.
    arm.write_word(cm::DFSR, cm::DFSR_HALTED).unwrap();
    assert_eq!(arm.halt_reason().unwrap(), HaltReason::HaltRequest);
    // Nothing recorded.
    arm.write_word(cm::DFSR, 0).unwrap();
    assert_eq!(arm.halt_reason().unwrap(), HaltReason::Unknown);
}

#[test]
fn bulk_mem_ops_roundtrip_across_tar_wrap() {
    // 64 bytes starting just below a 1 KiB TAR wrap boundary: exercises the
    // block read/write paths and their boundary splitting.
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    let data: Vec<u8> = (0u8..64).collect();
    arm.write_mem(0x2000_03e0, &data).unwrap();
    let mut back = [0u8; 64];
    arm.read_mem(0x2000_03e0, &mut back).unwrap();
    assert_eq!(&back[..], &data[..]);
    // Unaligned tail after a bulk run.
    let mut tail = [0u8; 7];
    arm.read_mem(0x2000_03e1, &mut tail).unwrap();
    assert_eq!(&tail[..], &data[1..8]);
}

#[test]
fn read_mem_and_write_mem_byte_granular() {
    let mut arm = ArmDebug::new(MemMock::new(), DapConfig::default());
    // Seed two words, then read an unaligned 6-byte span crossing them.
    arm.write_word(0x2000_0000, 0x04030201).unwrap();
    arm.write_word(0x2000_0004, 0x08070605).unwrap();
    let mut buf = [0u8; 6];
    arm.read_mem(0x2000_0001, &mut buf).unwrap();
    assert_eq!(buf, [0x02, 0x03, 0x04, 0x05, 0x06, 0x07]);

    // Unaligned partial write (read-modify-write) must not disturb neighbors.
    // Bytes at offsets 2,3 of word0 become 0xaa,0xbb; 0xcc lands in word1[0].
    arm.write_mem(0x2000_0002, &[0xaa, 0xbb, 0xcc]).unwrap();
    assert_eq!(arm.read_word(0x2000_0000).unwrap(), 0xbbaa0201);
    assert_eq!(arm.read_word(0x2000_0004).unwrap(), 0x080706cc);
    let mut check = [0u8; 4];
    arm.read_mem(0x2000_0002, &mut check[..3]).unwrap();
    assert_eq!(&check[..3], &[0xaa, 0xbb, 0xcc]);
    // Aligned full-word write fast path.
    arm.write_mem(0x2000_0008, &[0x11, 0x22, 0x33, 0x44])
        .unwrap();
    assert_eq!(arm.read_word(0x2000_0008).unwrap(), 0x44332211);
}
