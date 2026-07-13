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

//! Bit-banging transports over RP2040 GPIO pins.

use hal::gpio::{Floating, Input, Output, Pin, PinId, PushPull};
use rp2040_hal as hal;
use rust_dap::bitbang::{BidirPin, BitBangJtag, BitBangSwd, BitBangSwj};
use rust_dap::Delay;

/// Bidirectional pin backed by the rp2040-hal type-state GPIO API.
/// Holds the pin as an enum of its two mode states, so one type covers
/// both directions without the IoPin input/output type pair.
pub enum PicoBidirPin<I: PinId> {
    Input(Pin<I, Input<Floating>>),
    Output(Pin<I, Output<PushPull>>),
    /// Transient state during a mode switch; never observable.
    Invalid,
}

impl<I: PinId> PicoBidirPin<I> {
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self::Input(pin)
    }
}

impl<I: PinId> BidirPin for PicoBidirPin<I> {
    fn set_mode_output(&mut self, high: bool) {
        let state = if high {
            embedded_hal::digital::v2::PinState::High
        } else {
            embedded_hal::digital::v2::PinState::Low
        };
        *self = match core::mem::replace(self, Self::Invalid) {
            Self::Input(pin) => Self::Output(pin.into_push_pull_output_in_state(state)),
            Self::Output(mut pin) => {
                use embedded_hal::digital::v2::OutputPin;
                pin.set_state(state).ok();
                Self::Output(pin)
            }
            Self::Invalid => unreachable!(),
        };
    }

    fn set_mode_input(&mut self) {
        *self = match core::mem::replace(self, Self::Invalid) {
            Self::Input(pin) => Self::Input(pin),
            Self::Output(pin) => Self::Input(pin.into_floating_input()),
            Self::Invalid => unreachable!(),
        };
    }

    fn write(&mut self, high: bool) {
        if let Self::Output(pin) = self {
            use embedded_hal::digital::v2::OutputPin;
            if high {
                pin.set_high().ok();
            } else {
                pin.set_low().ok();
            }
        }
    }

    fn read(&mut self) -> bool {
        use embedded_hal::digital::v2::InputPin;
        match self {
            Self::Input(pin) => pin.is_high().unwrap_or(false),
            _ => false,
        }
    }
}

/// Cycle delay based on cortex_m::asm::delay.
pub struct CortexMDelay;

impl Delay for CortexMDelay {
    fn delay_cycles(&self, cycles: u32) {
        cortex_m::asm::delay(cycles);
    }
}

/// Bit-banging SWD transport over three RP2040 GPIO pins.
pub type SwdIoSet<Clk, Dio, Rst> =
    BitBangSwd<PicoBidirPin<Clk>, PicoBidirPin<Dio>, PicoBidirPin<Rst>, CortexMDelay>;

/// Bit-banging JTAG transport over six RP2040 GPIO pins.
pub type JtagIoSet<Tck, Tms, Tdi, Tdo, Trst, Srst> = BitBangJtag<
    PicoBidirPin<Tck>,
    PicoBidirPin<Tms>,
    PicoBidirPin<Tdi>,
    PicoBidirPin<Tdo>,
    PicoBidirPin<Trst>,
    PicoBidirPin<Srst>,
    CortexMDelay,
>;

/// Combined SWD+JTAG bit-banging transport with runtime port switching.
/// `Clk`=SWCLK/TCK, `Dio`=SWDIO/TMS, `Srst`=RESET/nSRST are shared between
/// protocols; `Tdi`/`Tdo`/`Trst` are JTAG-only.
pub type SwjIoSet<Clk, Dio, Tdi, Tdo, Trst, Srst> = BitBangSwj<
    PicoBidirPin<Clk>,
    PicoBidirPin<Dio>,
    PicoBidirPin<Tdi>,
    PicoBidirPin<Tdo>,
    PicoBidirPin<Trst>,
    PicoBidirPin<Srst>,
    CortexMDelay,
>;
