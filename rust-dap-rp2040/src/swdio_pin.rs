// Copyright 2022 Kenta Ida
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

use embedded_hal::digital::v2::{InputPin, IoPin, OutputPin, PinState};
use hal::gpio::{Disabled, Floating, Input, Output, Pin, PinId, PushPull};
use rp2040_hal as hal;

/// InputPin implementation for SWD pin
pub struct PicoSwdInputPin<I>
where
    I: PinId,
{
    pin: Pin<I, Input<Floating>>,
}

impl<I> PicoSwdInputPin<I>
where
    I: PinId,
{
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self { pin }
    }
}

impl<I> InputPin for PicoSwdInputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
    fn is_low(&self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdInputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        Ok(self)
    }
    fn into_output_pin(self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output_in_state(state);
        Ok(PicoSwdOutputPin::new(output_pin))
    }
}

/// OutputPin implementation for SWD pin
pub struct PicoSwdOutputPin<I>
where
    I: PinId,
{
    pin: Pin<I, Output<PushPull>>,
}

impl<I> PicoSwdOutputPin<I>
where
    I: PinId,
{
    pub fn new(pin: Pin<I, Output<PushPull>>) -> Self {
        Self { pin }
    }
}

impl<I> OutputPin for PicoSwdOutputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
    fn set_state(&mut self, state: embedded_hal::digital::v2::PinState) -> Result<(), Self::Error> {
        self.pin.set_state(state)
    }
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdOutputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        let input_pin = self.pin.into_floating_input();
        Ok(PicoSwdInputPin::new(input_pin))
    }
    fn into_output_pin(mut self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        self.set_state(state)?;
        Ok(self)
    }
}

/// Pico SWD pin
pub struct PicoSwdPin<I>
where
    I: PinId,
{
    pin: Pin<I, Disabled<Floating>>,
}

impl<I> IoPin<PicoSwdInputPin<I>, PicoSwdOutputPin<I>> for PicoSwdPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
    fn into_input_pin(self) -> Result<PicoSwdInputPin<I>, Self::Error> {
        let input_pin = self.pin.into_floating_input();
        Ok(PicoSwdInputPin::new(input_pin))
    }
    fn into_output_pin(self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output_in_state(state);
        Ok(PicoSwdOutputPin::new(output_pin))
    }
}
