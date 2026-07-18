use rust_dap::bitbang::BidirPin;
use xiao_m0::hal::gpio::{Floating, Input, Output, Pin, PinId, PushPull};

/// Bidirectional pin backed by the atsamd-hal type-state GPIO API.
/// Holds the pin as an enum of its two mode states, so one type covers
/// both directions.
pub enum XiaoBidirPin<I: PinId> {
    Input(Pin<I, Input<Floating>>),
    Output(Pin<I, Output<PushPull>>),
    /// Transient state during a mode switch; never observable.
    Invalid,
}

impl<I: PinId> XiaoBidirPin<I> {
    pub fn new(pin: Pin<I, Input<Floating>>) -> Self {
        Self::Input(pin)
    }
}

impl<I: PinId> BidirPin for XiaoBidirPin<I> {
    fn set_mode_output(&mut self, high: bool) {
        *self = match core::mem::replace(self, Self::Invalid) {
            Self::Input(pin) => {
                let mut pin = pin.into_push_pull_output();
                use embedded_hal::digital::OutputPin;
                if high {
                    pin.set_high().ok();
                } else {
                    pin.set_low().ok();
                }
                Self::Output(pin)
            }
            Self::Output(mut pin) => {
                use embedded_hal::digital::OutputPin;
                if high {
                    pin.set_high().ok();
                } else {
                    pin.set_low().ok();
                }
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
            use embedded_hal::digital::OutputPin;
            if high {
                pin.set_high().ok();
            } else {
                pin.set_low().ok();
            }
        }
    }

    fn read(&mut self) -> bool {
        use embedded_hal::digital::InputPin;
        match self {
            Self::Input(pin) => pin.is_high().unwrap_or(false),
            _ => false,
        }
    }
}
