use embedded_hal::digital::{ErrorType, InputPin, OutputPin, PinState};
use rust_dap::bitbang::IoPin;
use xiao_m0::hal::gpio::pin::{Disabled, Floating, Input, Output, Pin, PinId, PushPull};

pub struct XiaoSwdInputPin<I>
where
    I: PinId,
{
    pin: Pin<I, Input<Floating>>,
}

impl<I> XiaoSwdInputPin<I>
where
    I: PinId,
{
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self { pin }
    }
}

impl<I> ErrorType for XiaoSwdInputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
}

impl<I> InputPin for XiaoSwdInputPin<I>
where
    I: PinId,
{
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_high()
    }
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        self.pin.is_low()
    }
}

impl<I> IoPin<XiaoSwdInputPin<I>, XiaoSwdOutputPin<I>> for XiaoSwdInputPin<I>
where
    I: PinId,
{
    fn into_input_pin(self) -> Result<XiaoSwdInputPin<I>, Self::Error> {
        Ok(self)
    }
    fn into_output_pin(self, state: PinState) -> Result<XiaoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output();
        let mut new_pin = XiaoSwdOutputPin::new(output_pin);
        new_pin.set_state(state)?;
        Ok(new_pin)
    }
}
pub struct XiaoSwdOutputPin<I>
where
    I: PinId,
{
    pin: Pin<I, Output<PushPull>>,
}

impl<I> XiaoSwdOutputPin<I>
where
    I: PinId,
{
    pub fn new(pin: Pin<I, Output<PushPull>>) -> Self {
        Self { pin }
    }
}

impl<I> ErrorType for XiaoSwdOutputPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
}

impl<I> OutputPin for XiaoSwdOutputPin<I>
where
    I: PinId,
{
    fn set_high(&mut self) -> Result<(), Self::Error> {
        self.pin.set_high()
    }
    fn set_low(&mut self) -> Result<(), Self::Error> {
        self.pin.set_low()
    }
    fn set_state(&mut self, state: PinState) -> Result<(), Self::Error> {
        self.pin.set_state(state)
    }
}

impl<I> IoPin<XiaoSwdInputPin<I>, XiaoSwdOutputPin<I>> for XiaoSwdOutputPin<I>
where
    I: PinId,
{
    fn into_input_pin(self) -> Result<XiaoSwdInputPin<I>, Self::Error> {
        let new_pin = self.pin.into_floating_input();
        Ok(XiaoSwdInputPin::new(new_pin))
    }
    fn into_output_pin(mut self, state: PinState) -> Result<XiaoSwdOutputPin<I>, Self::Error> {
        self.set_state(state)?;
        Ok(self)
    }
}

pub struct XiaoSwdPin<I>
where
    I: PinId,
{
    pin: Pin<I, Disabled<Floating>>,
}

impl<I> ErrorType for XiaoSwdPin<I>
where
    I: PinId,
{
    type Error = core::convert::Infallible;
}

impl<I> IoPin<XiaoSwdInputPin<I>, XiaoSwdOutputPin<I>> for XiaoSwdPin<I>
where
    I: PinId,
{
    fn into_input_pin(self) -> Result<XiaoSwdInputPin<I>, Self::Error> {
        let input_pin = self.pin.into_floating_input();
        Ok(XiaoSwdInputPin::new(input_pin))
    }
    fn into_output_pin(self, state: PinState) -> Result<XiaoSwdOutputPin<I>, Self::Error> {
        let output_pin = self.pin.into_push_pull_output();
        let mut new_pin = XiaoSwdOutputPin::new(output_pin);
        new_pin.set_state(state)?;
        Ok(new_pin)
    }
}
