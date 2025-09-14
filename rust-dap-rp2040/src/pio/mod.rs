// Copyright 2022 Ein Terakawa
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

use pio::Program;
use rp2040_hal as hal;
pub mod pio0 {
    use crate::pio::hal::{self, gpio::FunctionPio0, gpio::PullUp};
    pub type Pin<P> = hal::gpio::Pin<P, FunctionPio0, PullUp>;
}

pub mod jtag;
pub mod swd;

const DEFAULT_CORE_CLOCK: u32 = 125000000;
/// Default PIO Divisor.
/// Generate 125/8 = 15.625[MHz] SWCLK clock.
/// Generate 125/8 = 15.625[MHz] TCK clock(max)
const DEFAULT_PIO_DIVISOR: f32 = 1.0f32;

fn swj_pins_program() -> Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    type Assembler = pio::Assembler<{ pio::RP2040_MAX_PROGRAM_SIZE }>;
    let mut a = Assembler::new();
    let mut delay_loop = a.label();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);

    // command data
    // [
    //      pin_output_values: u32,
    //      pin_directions: u32,
    //      wait_us:    u32
    // ]

    // load and set output value
    a.pull(false, true);
    a.out(pio::OutDestination::PINS, 32);
    // load and set direction
    a.pull(false, true);
    a.out(pio::OutDestination::PINDIRS, 32);

    // load output delay to Y
    a.pull(false, true);
    a.out(pio::OutDestination::Y, 32);

    // wait_us
    a.bind(&mut delay_loop);
    // delay 9(+1) cycles * 0.1us/cycle = 1us
    a.jmp_with_delay(pio::JmpCondition::YDecNonZero, &mut delay_loop, 9);

    // check all pin status
    a.r#in(pio::InSource::PINS, 32);
    a.push(false, true);

    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}

fn all_pins_to_input_program() -> Program<{ pio::RP2040_MAX_PROGRAM_SIZE }> {
    type Assembler = pio::Assembler<{ pio::RP2040_MAX_PROGRAM_SIZE }>;
    let mut a = Assembler::new();
    let mut wrap_target = a.label();
    let mut wrap_source = a.label();

    a.bind(&mut wrap_target);

    a.mov(
        pio::MovDestination::X,
        pio::MovOperation::None,
        pio::MovSource::NULL,
    );
    a.out(pio::OutDestination::PINDIRS, 32);

    a.bind(&mut wrap_source);

    a.assemble_with_wrap(wrap_source, wrap_target)
}
