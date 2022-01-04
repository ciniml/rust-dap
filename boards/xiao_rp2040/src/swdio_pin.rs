
use embedded_hal::digital::v2::{InputPin, OutputPin, IoPin, PinState};
use rp_pico::hal::gpio::{Pin, PinId, Input, Output, Disabled, Floating, PushPull, OutputOverride};

pub struct PicoSwdInputPin<I> 
where
    I: PinId
{
    pin: Pin<I, Input<Floating>>,
}

impl<I> PicoSwdInputPin<I> 
where
    I: PinId
{
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self {
            pin: pin,
        }
    }
}

impl<I> InputPin for PicoSwdInputPin<I> 
where
    I: PinId
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
    fn into_output_pin(mut self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        let output_override = if state == PinState::High { OutputOverride::AlwaysHigh } else { OutputOverride::AlwaysLow };
        self.pin.set_output_override(output_override);
        let mut output_pin = self.pin.into_push_pull_output();
        output_pin.set_output_override(OutputOverride::DontInvert);
        let mut new_pin = PicoSwdOutputPin::new(output_pin);
        new_pin.set_state(state)?;
        Ok(new_pin)
    }
}
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
        Self {
            pin: pin,
        }
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
        let new_pin = self.pin.into_floating_input();
        Ok(PicoSwdInputPin::new(new_pin))
    }
    fn into_output_pin(mut self, state: PinState) -> Result<PicoSwdOutputPin<I>, Self::Error> {
        self.set_state(state)?;
        Ok(self)
    }
}

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
        let output_pin = self.pin.into_push_pull_output();
        let mut new_pin = PicoSwdOutputPin::new(output_pin);
        new_pin.set_state(state)?;
        Ok(new_pin)
    }
}
