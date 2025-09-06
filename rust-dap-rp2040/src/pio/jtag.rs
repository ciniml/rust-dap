// Copyright 2023 Toshifumi Nishinaga
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

use cortex_m::asm;
use hal::{
    gpio::PinId,
    pac::{self, PIO0},
    pio::PIOExt,
};
use rp2040_hal as hal;
use rust_dap::*;
pub mod pio0 {
    use crate::pio::hal::{self, gpio::FunctionPio0, gpio::PullUp};
    pub type Pin<P> = hal::gpio::Pin<P, FunctionPio0, PullUp>;
}
use bitvec::prelude::*;

use super::{swj_pins_program, DEFAULT_CORE_CLOCK, DEFAULT_PIO_DIVISOR};

// TCKを1周期実行するのに必要なサイクル数
// number of cycles required to send 1 TCK clock
#[cfg(feature = "set_clock")]
const NUM_OF_CYCLE_PER_TCK_CLOCK: u32 = 8;
#[cfg(feature = "set_clock")]
const DEFAULT_SWJ_CLOCK_HZ: u32 =
    ((DEFAULT_CORE_CLOCK / NUM_OF_CYCLE_PER_TCK_CLOCK) as f32 / DEFAULT_PIO_DIVISOR) as u32;

const MAX_IR_LENGTH: usize = 128;

struct JtagPioContext {
    pio: hal::pio::PIO<PIO0>,
    running_sm: hal::pio::StateMachine<hal::pio::PIO0SM0, hal::pio::Running>,
    rx_fifo: hal::pio::Rx<hal::pio::PIO0SM0>,
    tx_fifo: hal::pio::Tx<hal::pio::PIO0SM0>,
}
pub struct JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> {
    tck_pin_id: u8,
    tms_pin_id: u8,
    tdi_pin_id: u8,
    tdo_pin_id: u8,
    trst_pin_id: Option<u8>,
    srst_pin_id: Option<u8>,
    context: Option<JtagPioContext>,
    divisor: f32,
    _pins: core::marker::PhantomData<(Tck, Tms, Tdi, Tdo, Trst, Srst)>,
}

fn jtag_pin_set() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    type Assembler = pio::Assembler<{ pio::RP2040_MAX_PROGRAM_SIZE }>;
    let mut a = Assembler::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);
    // load pin out
    a.pull(false, true);
    a.out(pio::OutDestination::PINS, 32);
    // load pindirs
    a.pull(false, true);
    a.out(pio::OutDestination::PINDIRS, 32);
    // return ack(0xFFFFFFFF)
    a.mov(
        pio::MovDestination::ISR,
        pio::MovOperation::Invert,
        pio::MovSource::NULL,
    );
    a.push(false, true);
    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);
    program
}

fn jtag_sequence_program() -> pio::Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    type Assembler = pio::Assembler<{ pio::RP2040_MAX_PROGRAM_SIZE }>;
    let mut a = Assembler::new_with_side_set(pio::SideSet::new(true, 1, false));
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();
    let mut tdi_shift_start = a.label();
    let mut push_unaligned = a.label();

    // data format
    // [
    //      bit length(31bit) | tms bit(1bit)
    //      data bits(n-bit)
    // ]

    // WARNING: bit length must be greater than 0

    a.bind(&mut wrap_target);

    // pull size(31bit) | tms_value(1bit)
    a.pull(false, true);
    // out tms_value
    a.out(pio::OutDestination::Y, 1);
    // out bit length to X
    a.out(pio::OutDestination::X, 31);

    // set tms value
    a.set(pio::SetDestination::PINS, 0);
    a.jmp(pio::JmpCondition::YIsZero, &mut tdi_shift_start);
    a.set(pio::SetDestination::PINS, 1);

    {
        a.bind(&mut tdi_shift_start);
        // falling part(4 clock)
        // pull if (32 <= bit count)
        a.pull(true, true);
        a.out_with_delay_and_side_set(pio::OutDestination::PINS, 1, 3, 0);
        // rising part(4 clock)
        a.in_with_side_set(pio::InSource::PINS, 1, 1);
        // push if (32 <= bit count)
        a.push(true, true);
        // jmp
        a.jmp(pio::JmpCondition::XDecNonZero, &mut tdi_shift_start);
    }

    a.jmp(
        pio::JmpCondition::OutputShiftRegisterNotEmpty,
        &mut push_unaligned,
    );
    a.jmp(pio::JmpCondition::Always, &mut wrap_target);
    a.bind(&mut push_unaligned);
    // push unaligned part
    a.push(false, true);

    a.bind(&mut wrap_source);

    let program = a.assemble_with_wrap(wrap_source, wrap_target);
    program
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst>
    JtagIoSet<
        pio0::Pin<Tck>,
        pio0::Pin<Tms>,
        pio0::Pin<Tdi>,
        pio0::Pin<Tdo>,
        pio0::Pin<Trst>,
        pio0::Pin<Srst>,
    >
where
    Tck: PinId,
    Tms: PinId,
    Tdi: PinId,
    Tdo: PinId,
    Trst: PinId,
    Srst: PinId,
{
    pub fn new(
        pio0: pac::PIO0,
        tck: pio0::Pin<Tck>,
        tms: pio0::Pin<Tms>,
        tdi: pio0::Pin<Tdi>,
        tdo: pio0::Pin<Tdo>,
        trst: Option<pio0::Pin<Trst>>,
        srst: Option<pio0::Pin<Srst>>,
        resets: &mut pac::RESETS,
    ) -> Self {
        let tck_pin_id = tck.id().num;
        let tms_pin_id = tms.id().num;
        let tdi_pin_id = tdi.id().num;
        let tdo_pin_id = tdo.id().num;
        let trst_pin_id = if let Some(trst) = trst {
            Some(trst.id().num)
        } else {
            None
        };
        let srst_pin_id: Option<u8> = if let Some(srst) = srst {
            Some(srst.id().num)
        } else {
            None
        };

        let program = jtag_pin_set();
        let divisor = crate::pio::DEFAULT_PIO_DIVISOR;

        // Initialize and start PIO
        let (mut pio, sm0, _, _, _) = pio0.split(resets);
        let installed = pio.install(&program).unwrap();
        let (sm, rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(tms_pin_id, 1)
            .side_set_pin_base(tck_pin_id)
            .out_pins(tdi_pin_id, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(tdo_pin_id)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .build(sm0);

        let running_sm = sm.start();

        let mut jtagioset = Self {
            tck_pin_id,
            tms_pin_id,
            tdi_pin_id,
            tdo_pin_id,
            trst_pin_id,
            srst_pin_id,
            context: Some(JtagPioContext {
                pio,
                running_sm,
                rx_fifo: rx,
                tx_fifo: tx,
            }),
            divisor: DEFAULT_PIO_DIVISOR,
            _pins: core::marker::PhantomData,
        };

        jtagioset.disconnect_inner();

        jtagioset
    }
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst> JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> {
    // based on SwdIo
    #[cfg(feature = "set_clock")]
    fn set_clock(&mut self, frequency_hz: u32) {
        // Calculate divisor.
        let divisor = if frequency_hz > 0 {
            ((DEFAULT_CORE_CLOCK / NUM_OF_CYCLE_PER_TCK_CLOCK as u32) / frequency_hz) as f32
        } else {
            DEFAULT_PIO_DIVISOR
        };
        let divisor = if divisor < 1.0f32 {
            DEFAULT_PIO_DIVISOR
        } else {
            divisor
        };
        self.divisor = divisor;

        // Stop SM, Reconstruct SM with new divisor.
        let mut context = None;
        core::mem::swap(&mut self.context, &mut context);
        let context = context.unwrap();
        let (sm0, installed) = context.running_sm.uninit(context.rx_fifo, context.tx_fifo);
        let (sm, rx, tx) = hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(self.tms_pin_id, 1)
            .side_set_pin_base(self.tck_pin_id)
            .out_pins(self.tdi_pin_id, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(self.tdo_pin_id)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .build(sm0);
        let running_sm = sm.start();
        let pio = context.pio;

        let mut new_context = Some(JtagPioContext {
            pio,
            running_sm,
            rx_fifo: rx,
            tx_fifo: tx,
        });
        core::mem::swap(&mut self.context, &mut new_context);
    }

    fn get_context(&mut self) -> &mut JtagPioContext {
        self.context.as_mut().unwrap()
    }

    fn install_swj_pins_program(&mut self) {
        // Calculate divisor.
        // set 0.1us = 10 MHz
        // core frequency(MHz) / 100(MHz) = 0.1us/clock
        let system_clock = f32::try_from((DEFAULT_CORE_CLOCK / 1_000_000) as u16).unwrap();
        let divisor = system_clock / 10f32;

        // Stop SM, Reconstruct SM with new program.
        let mut context = None;
        core::mem::swap(&mut self.context, &mut context);
        let context = context.unwrap();
        let (sm0, installed) = context.running_sm.uninit(context.rx_fifo, context.tx_fifo);
        let mut pio = context.pio;
        pio.uninstall(installed);
        let installed = pio.install(&swj_pins_program()).unwrap();
        let (sm, rx_fifo, tx_fifo) = hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(0, 1)
            .side_set_pin_base(0)
            .out_pins(0, 32)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(0)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .build(sm0);
        let running_sm = sm.start();
        let mut new_context = Some(JtagPioContext {
            pio,
            running_sm,
            rx_fifo,
            tx_fifo,
        });
        core::mem::swap(&mut self.context, &mut new_context);
    }

    fn install_jtag_pin_set_program(&mut self) {
        // Stop SM, Reconstruct SM with new program.
        let mut context = None;
        core::mem::swap(&mut self.context, &mut context);
        let context = context.unwrap();
        let (sm0, installed) = context.running_sm.uninit(context.rx_fifo, context.tx_fifo);
        let mut pio = context.pio;
        pio.uninstall(installed);
        let installed = pio.install(&jtag_pin_set()).unwrap();
        let divisor = crate::pio::DEFAULT_PIO_DIVISOR;
        let (sm, rx_fifo, tx_fifo) = hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(0, 1)
            .side_set_pin_base(0)
            .out_pins(0, 32)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(0)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .build(sm0);
        let running_sm = sm.start();
        let mut new_context = Some(JtagPioContext {
            pio,
            running_sm,
            rx_fifo,
            tx_fifo,
        });
        core::mem::swap(&mut self.context, &mut new_context);
    }

    fn install_jtag_sequence_program(&mut self) {
        // Stop SM, Reconstruct SM with new program.
        let mut context = None;
        core::mem::swap(&mut self.context, &mut context);
        let context = context.unwrap();
        let (sm0, installed) = context.running_sm.uninit(context.rx_fifo, context.tx_fifo);
        let mut pio = context.pio;
        pio.uninstall(installed);
        let installed = pio.install(&jtag_sequence_program()).unwrap();
        let divisor = self.divisor;
        let (sm, rx_fifo, tx_fifo) = hal::pio::PIOBuilder::from_installed_program(installed)
            .set_pins(self.tms_pin_id, 1)
            .side_set_pin_base(self.tck_pin_id)
            .out_pins(self.tdi_pin_id, 1)
            .out_shift_direction(hal::pio::ShiftDirection::Right)
            .in_pin_base(self.tdo_pin_id)
            .in_shift_direction(hal::pio::ShiftDirection::Right)
            .clock_divisor_fixed_point(divisor as u16, (divisor * 256.0) as u8)
            .pull_threshold(32)
            .push_threshold(32)
            .build(sm0);
        let running_sm = sm.start();
        let mut new_context = Some(JtagPioContext {
            pio,
            running_sm,
            rx_fifo,
            tx_fifo,
        });
        core::mem::swap(&mut self.context, &mut new_context);
    }

    fn jtag_sequence_program_interface(&mut self, data: &mut BitSlice<u32>, tms: bool) {
        if data.len() == 0 {
            panic!("data.len() must be greater than 0");
        }
        let con = self.get_context();
        // send length
        while !con
            .tx_fifo
            .write((u32::try_from(data.len()).unwrap() - 1) << 1 | if tms { 1 } else { 0 })
        {
            asm::nop()
        }
        // send bits
        for x in data.chunks_mut(32) {
            let bits = x.load();
            // send bits
            while !con.tx_fifo.write(bits) {
                asm::nop()
            }
            // recv bits
            let mut rx = loop {
                if let Some(x) = con.rx_fifo.read() {
                    break x;
                } else {
                    asm::nop()
                }
            };
            if x.len() < 32 {
                rx = rx >> (32 - x.len());
            }
            x.store(rx);
        }
    }

    fn jtag_pin_set_program_interface(&mut self, pin_out: u32, pindirs: u32) {
        let con = self.get_context();
        while !con.tx_fifo.write(pin_out) {
            asm::nop()
        }
        while !con.tx_fifo.write(pindirs) {
            asm::nop()
        }
        // wait for ack
        let ack: u32 = loop {
            if let Some(x) = con.rx_fifo.read() {
                break x;
            } else {
                asm::nop()
            }
        };
        assert_eq!(ack, 0xFFFF_FFFF);
    }

    fn connect_inner(&mut self) {
        self.install_jtag_pin_set_program();
        // output high: trst, srst, tms, tck, tdi
        // input: tdo
        let pin_out = 0xFFFF_FFFF;
        let pindirs = 1 << self.tck_pin_id
            | 1 << self.tms_pin_id
            | 1 << self.tdi_pin_id
            | if let Some(id) = self.trst_pin_id {
                1 << id
            } else {
                0
            }
            | if let Some(id) = self.srst_pin_id {
                1 << id
            } else {
                0
            };
        self.jtag_pin_set_program_interface(pin_out, pindirs);

        self.install_jtag_sequence_program();
    }

    fn disconnect_inner(&mut self) {
        self.install_jtag_pin_set_program();
        // all input
        let pin_out = 0xFFFF_FFFF;
        let pindirs = 0;
        self.jtag_pin_set_program_interface(pin_out, pindirs);
        // Reset clock
        #[cfg(feature = "set_clock")]
        self.set_clock(DEFAULT_SWJ_CLOCK_HZ);
    }

    fn write_tms(&mut self, tms: bool) {
        let data = bits![mut u32, Lsb0; 0;1];
        self.jtag_sequence_program_interface(data, tms);
    }
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst> JtagIo for JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> {
    fn connect(&mut self, _config: &JtagIoConfig) {
        // output high: trst, srst, tms, tck, tdi
        // input: tdo
        self.connect_inner();
    }

    fn disconnect(&mut self, _config: &JtagIoConfig) {
        // all input
        self.disconnect_inner();
    }

    fn swj_clock(
        &mut self,
        _config: &mut JtagIoConfig,
        #[allow(unused_variables)] frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        #[cfg(feature = "set_clock")]
        self.set_clock(frequency_hz);
        Ok(())
    }

    fn swj_sequence(&mut self, _config: &JtagIoConfig, count: usize, data: &[u8]) {
        // https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__SWJ__Sequence.html

        let data = data.view_bits::<Lsb0>();
        let (tms_data, _) = data.split_at(count);
        for tms in tms_data {
            self.write_tms(*tms);
        }
    }

    fn jtag_read_sequence(
        &mut self,
        _config: &JtagIoConfig,
        clock_count: usize,
        tms_value: bool,
        tdi_data: u64,
    ) -> u64 {
        let tdi_data_lo = tdi_data as u32;
        let tdi_data_hi = (tdi_data >> 32) as u32;
        // we can't use view_bits on 32bit cpu
        // https://docs.rs/bitvec/latest/src/bitvec/store.rs.html#194-195
        let mut data = bitarr!(u32, Lsb0; 0; 64);
        data.data.copy_from_slice(&[tdi_data_lo, tdi_data_hi]);
        self.jtag_sequence_program_interface(&mut data.as_mut_bitslice()[..clock_count], tms_value);

        data.load()
    }

    fn jtag_write_sequence(
        &mut self,
        config: &JtagIoConfig,
        clock_count: usize,
        tms_value: bool,
        tdi_data: u64,
    ) {
        let _ = self.jtag_read_sequence(config, clock_count, tms_value, tdi_data);
    }

    fn jtag_idcode(
        &mut self,
        _config: &JtagIoConfig,
        _index: u8,
    ) -> core::result::Result<u32, DapError> {
        // https://arm-software.github.io/CMSIS_5/DAP/html/group__DAP__jtag__idcode.html#details
        Err(DapError::InvalidCommand)
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
        let mut dr = build_acc(data, a3, a2, true);
        self.read_write_dr(config, dap_index, dr.as_mut_bitslice());
        if read {
            // TODO: OK_FALSE等の情報を返す必要がないか調べる
            dr.shift_right(3);
            let result: u32 = dr.load();
            Ok(result)
        } else {
            Ok(0)
        }
    }

    fn write_ir(&mut self, config: &JtagIoConfig, dap_index: u8, ir: u32) {
        // to ShiftIR from Run-Test-Idle
        self.write_tms(true); // SelectDR
        self.write_tms(true); // SelectIR
        self.write_tms(false); // CaptureIR
        self.write_tms(false); // ShiftIR

        // dap_indexの示すIRレジスタ以外はBYPASS(all 1)にする
        // TDI -> other devices(head) -> target device -> other devices(tail) -> TDO
        let device_count = config.device_count as usize;
        let total_ir_bit_length: usize = (&config.ir_length[0..device_count])
            .iter()
            .map(|x| *x as usize)
            .sum();
        let target_ir_bit_length = *config.ir_length.get(dap_index as usize).unwrap() as usize;

        let tail_ir_bit_length: usize = if 1 < device_count {
            // not tested
            config.ir_length[(device_count + 1)..]
                .iter()
                .map(|x| *x as usize)
                .sum()
        } else {
            0
        };

        let mut ir = ir;
        let ir_bits = bits![mut u32, Lsb0; 1;MAX_IR_LENGTH];
        // target_ir_bits << tail | ir する
        for i in 0..target_ir_bit_length {
            ir_bits.set(tail_ir_bit_length + i, ir & 1 != 0);
            ir = ir >> 1;
        }

        // write ir bits
        self.jtag_sequence_program_interface(&mut ir_bits[0..total_ir_bit_length - 1], false);
        self.jtag_sequence_program_interface(
            &mut ir_bits[total_ir_bit_length - 1..total_ir_bit_length],
            true,
        );

        // to Run from Exit-IR
        self.write_tms(true); // Update-IR
        self.write_tms(false); // Run
    }

    fn read_write_dr(&mut self, config: &JtagIoConfig, dap_index: u8, dr: &mut BitSlice<u32>) {
        // to ShiftIR from Run-Test-Idle
        self.write_tms(true); // SelectDR
        self.write_tms(false); // CaptureDR
        self.write_tms(false); // ShiftDR

        let target_position_idx = usize::from(dap_index); // zero indexed
        let total_bits = usize::from(config.device_count) + dr.len() - 1;
        let dr_bits = bits![mut u32, Lsb0; 1;MAX_IR_LENGTH];
        // copy dr
        for (i, d) in dr.iter().enumerate() {
            dr_bits.set(target_position_idx + i, *d);
        }

        // send bits
        self.jtag_sequence_program_interface(&mut dr_bits[0..total_bits - 1], false);
        self.jtag_sequence_program_interface(&mut dr_bits[total_bits - 1..total_bits], true);

        // take out target dr bits
        for (i, mut x) in dr.iter_mut().enumerate() {
            x.set(dr_bits[target_position_idx + i]);
        }

        // to Run from Exit-DR
        self.write_tms(true); // Update-DR
        self.write_tms(false); // Run
    }
}

impl<Tck, Tms, Tdi, Tdo, Trst, Srst> CmsisDapCommandInner
    for JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst>
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
        wait_us: u32,
    ) -> core::result::Result<u8, DapError> {
        self.install_swj_pins_program();

        // clean rx fifo
        // while !self.get_context().rx_fifo.is_empty() {
        //     let _ = self.get_context().rx_fifo.read();
        // }

        let pin_output = SwjPins::from_bits(pin_output).unwrap();
        let pin_select = SwjPins::from_bits(pin_select).unwrap();
        let flags = [
            (SwjPins::TCK_SWDCLK, self.tck_pin_id),
            (SwjPins::TMS_SWDIO, self.tms_pin_id),
            (SwjPins::TDI, self.tdi_pin_id),
            (SwjPins::TDO, self.tdo_pin_id),
            (
                SwjPins::N_TRST,
                if let Some(x) = self.trst_pin_id { x } else { 0 },
            ),
            (
                SwjPins::N_RESET,
                if let Some(x) = self.srst_pin_id { x } else { 0 },
            ),
        ];

        // output
        let pio_pin_out: u32 = flags.iter().fold(0, |out, (flag, pin_id)| {
            out | if pin_output.contains(*flag) && pin_select.contains(*flag) {
                1 << pin_id
            } else {
                0
            }
        });
        self.context.as_mut().unwrap().tx_fifo.write(pio_pin_out);

        // directions
        let pio_pin_directions = flags.iter().fold(0, |dir, (flag, pin_id)| {
            dir | (match flag {
                // output
                &SwjPins::TCK_SWDCLK
                | &SwjPins::TMS_SWDIO
                | &SwjPins::TDI
                | &SwjPins::N_RESET
                | &SwjPins::N_TRST => 1,
                // input
                &SwjPins::TDO => 0,
                _ => 0,
            } << pin_id)
        });
        self.context
            .as_mut()
            .unwrap()
            .tx_fifo
            .write(pio_pin_directions);

        // wait_us
        self.context.as_mut().unwrap().tx_fifo.write(wait_us);

        // input
        let tmp = loop {
            if let Some(x) = self.context.as_mut().unwrap().rx_fifo.read() {
                break x;
            } else {
                asm::nop();
            };
        };
        // convert
        let input = flags
            .iter()
            .fold(SwjPins::empty(), |input, (flag, pin_id)| {
                input
                    | if tmp & (1 << pin_id) != 0 {
                        *flag
                    } else {
                        SwjPins::empty()
                    }
            });

        // restore JTAG program
        self.install_jtag_sequence_program();

        Ok(input.bits())
    }

    fn swj_clock(
        &mut self,
        config: &mut CmsisDapConfig,
        frequency_hz: u32,
    ) -> core::result::Result<(), DapError> {
        JtagIo::swj_clock(self, &mut config.jtag, frequency_hz)
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
