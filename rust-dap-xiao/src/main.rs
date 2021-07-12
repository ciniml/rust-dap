#![no_std]
#![no_main]

use embedded_hal::digital::v2::InputPin;
use hal::gpio::Port;
use hal::prelude::_atsamd_hal_embedded_hal_digital_v2_OutputPin;
use panic_halt as _;
use rust_dap::DapError;
use rust_dap::SwdIo;
use rust_dap::SwdIoConfig;
use rust_dap::SwdRequest;
use rust_dap::DAP_TRANSFER_MISMATCH;
use rust_dap::DAP_TRANSFER_OK;
use rust_dap::USB_CLASS_MISCELLANEOUS;
use rust_dap::USB_PROTOCOL_IAD;
use rust_dap::USB_SUBCLASS_COMMON;
use xiao_m0 as hal;

use hal::clock::GenericClockController;
use hal::entry;
use hal::gpio::v2::{PA05, PA07};
use hal::gpio::{Floating, Input, OpenDrain, Output, Pa18, Pin, PushPull};
use hal::pac::{interrupt, CorePeripherals, Peripherals};

use hal::usb::UsbBus;
use usb_device::bus::UsbBusAllocator;

use rust_dap::CmsisDap;
use usb_device::prelude::*;
use usbd_serial::{SerialPort, USB_CLASS_CDC};

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

type SwdInputPin<P> = Pin<P, Input<Floating>>;
type SwdOutputPin<P> = Pin<P, Output<PushPull>>;
type SwdIoPin = PA05;
type SwClkPin = PA07;
type SwdIoInputPin = SwdInputPin<SwdIoPin>;
type SwdIoOutputPin = SwdOutputPin<SwdIoPin>;
type SwClkInputPin = SwdInputPin<SwClkPin>;
type SwClkOutputPin = SwdOutputPin<SwClkPin>;

struct XiaoSwdIo {
    swdio_in: Option<SwdIoInputPin>,
    swdio_out: Option<SwdIoOutputPin>,
    swclk_in: Option<SwClkInputPin>,
    swclk_out: Option<SwClkOutputPin>,
}

impl XiaoSwdIo {
    fn swclk_out(&mut self) -> Option<&mut SwClkOutputPin> {
        self.swclk_out.as_mut()
    }
    fn swdio_out(&mut self) -> Option<&mut SwdIoOutputPin> {
        self.swdio_out.as_mut()
    }
    fn swdio_in(&mut self) -> Option<&mut SwdIoInputPin> {
        self.swdio_in.as_mut()
    }

    fn clock_wait(&self, config: &SwdIoConfig) {
        cycle_delay(config.clock_wait_cycles);
    }
    fn cycle_clock(&mut self, config: &SwdIoConfig) {
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
    }
    fn turn_around(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.turn_around_cycles {
            self.cycle_clock(config);
        }
    }
    fn idle_cycle(&mut self, config: &SwdIoConfig) {
        for _ in 0..config.idle_cycles {
            self.write_bit(config, false);
        }
    }
    fn write_bit(&mut self, config: &SwdIoConfig, value: bool) {
        self.swdio_out().and_then(|p| {
            if value {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
    }
    fn read_bit(&mut self, config: &SwdIoConfig) -> bool {
        self.swclk_out().and_then(|p| p.set_low().ok());
        self.clock_wait(config);
        let value = self
            .swdio_in()
            .map_or_else(|| false, |p| p.is_high().unwrap_or(false));
        self.swclk_out().and_then(|p| p.set_high().ok());
        self.clock_wait(config);
        value
    }
    fn set_swdio(&mut self, value: bool) {
        self.swdio_out().and_then(|p| {
            if value {
                p.set_high().ok()
            } else {
                p.set_low().ok()
            }
        });
    }
    #[allow(dead_code)]
    fn get_timestamp(&mut self) -> u32 {
        0
    }
}

impl SwdIo for XiaoSwdIo {
    fn connect(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swdio_out = Some(new);
            }
            if let Some(old) = self.swclk_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swclk_out = Some(new);
            }
        });
    }
    fn disconnect(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_out.take() {
                let new = old.into_floating_input(port);
                self.swdio_in = Some(new);
            }
            if let Some(old) = self.swclk_out.take() {
                let new = old.into_floating_input(port);
                self.swclk_in = Some(new);
            }
        });
    }
    fn swj_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut index = 0;
        let mut value = 0;
        let mut bits = 0;
        let mut count = count;

        while count > 0 {
            count -= 1;
            if bits == 0 {
                value = data[index];
                index += 1;
                bits = 8;
            }
            self.write_bit(config, value & 1 != 0);
            value >>= 1;
            bits -= 1;
        }
    }
    fn swd_read_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &mut [u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = 0;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                let bit_value = self.read_bit(config);
                value = if bit_value {
                    (value >> 1) | 0x80
                } else {
                    value >> 1
                };
            }
            value >>= bits;
            data[index] = value;
            index += 1;
        }
    }

    fn swd_write_sequence(&mut self, config: &SwdIoConfig, count: usize, data: &[u8]) {
        let mut count = count;
        let mut index = 0;
        while count > 0 {
            let mut value = data[index];
            index += 1;
            let mut bits = 8;
            while bits > 0 && count > 0 {
                bits -= 1;
                count -= 1;

                self.write_bit(config, value & 1 != 0);
                value >>= 1;
            }
        }
    }

    fn swd_transfer(
        &mut self,
        config: &SwdIoConfig,
        request: SwdRequest,
        data: u32,
    ) -> core::result::Result<u32, rust_dap::DapError> {
        // write request
        self.enable_output();
        {
            let mut parity = false;
            self.write_bit(config, true); // Start
            let bit = request.contains(SwdRequest::APnDP);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::RnW);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A2);
            self.write_bit(config, bit);
            parity ^= bit;
            let bit = request.contains(SwdRequest::A3);
            self.write_bit(config, bit);
            parity ^= bit;
            self.write_bit(config, parity); // Parity
            self.write_bit(config, false); // Stop
            self.write_bit(config, true); // Park
        }

        // turnaround + read ack.
        self.disable_output();
        self.turn_around(config);
        let ack = {
            let mut ack = 0u8;
            ack |= if self.read_bit(config) { 0b001 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b010 } else { 0b000 };
            ack |= if self.read_bit(config) { 0b100 } else { 0b000 };
            ack
        };
        if ack == rust_dap::DAP_TRANSFER_OK {
            let ack = if request.contains(SwdRequest::RnW) {
                // READ request
                let mut value = 0u32;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = self.read_bit(config);
                    parity ^= bit;
                    value = (value >> 1) | if bit { 0x80000000 } else { 0x00000000 };
                }
                let parity_expected = self.read_bit(config);
                self.turn_around(config);
                self.enable_output();
                if parity == parity_expected {
                    Ok(value)
                } else {
                    Err(DapError::SwdError(DAP_TRANSFER_MISMATCH))
                }
            } else {
                // WRITE request
                self.turn_around(config);
                self.enable_output();
                let mut value = data;
                let mut parity = false;
                for _ in 0..32 {
                    let bit = value & 1 != 0;
                    self.write_bit(config, bit);
                    parity ^= bit;
                    value >>= 1;
                }
                self.write_bit(config, parity);
                Ok(0)
            };
            // TODO: capture timestamp
            self.idle_cycle(config);
            self.set_swdio(true);
            return ack;
        }

        // An error occured.
        if ack == rust_dap::DAP_TRANSFER_WAIT || ack == rust_dap::DAP_TRANSFER_FAULT {
            self.disable_output();
            if config.always_generate_data_phase && request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.cycle_clock(config);
                }
            }
            self.turn_around(config);
            self.enable_output();
            if config.always_generate_data_phase && !request.contains(SwdRequest::RnW) {
                for _ in 0..33 {
                    self.write_bit(config, false);
                }
            }
            self.set_swdio(true);
            return Err(DapError::SwdError(ack));
        }

        // Protocol error
        self.turn_around(config);
        for _ in 0..33 {
            self.cycle_clock(config);
        }
        self.enable_output();
        self.set_swdio(true);
        return Err(DapError::SwdError(ack));
    }

    fn enable_output(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_in.take() {
                let mut new = old.into_push_pull_output(port);
                new.set_low().ok();
                self.swdio_out = Some(new);
            }
        });
    }

    fn disable_output(&mut self) {
        let port = unsafe { PORT.as_mut() };
        port.map(|port| {
            if let Some(old) = self.swdio_out.take() {
                let new = old.into_floating_input(port);
                self.swdio_in = Some(new);
            }
        });
    }
}

#[entry]
fn main() -> ! {
    let mut peripherals = Peripherals::take().unwrap();
    let mut core = CorePeripherals::take().unwrap();
    let mut clocks = GenericClockController::with_internal_32kosc(
        peripherals.GCLK,
        &mut peripherals.PM,
        &mut peripherals.SYSCTRL,
        &mut peripherals.NVMCTRL,
    );

    let mut pins = hal::Pins::new(peripherals.PORT);
    let mut led0 = pins.led0.into_open_drain_output(&mut pins.port);

    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(hal::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
            &mut pins.port,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    let swdio = XiaoSwdIo {
        swclk_in: Some(pins.a8),
        swdio_in: Some(pins.a9),
        swclk_out: None,
        swdio_out: None,
    };

    unsafe {
        USB_SERIAL = Some(SerialPort::new(&bus_allocator));
        USB_DAP = Some(CmsisDap::new(&bus_allocator, swdio));
        USB_BUS = Some(
            UsbDeviceBuilder::new(&bus_allocator, UsbVidPid(0x6666, 0x4444))
                .manufacturer("fugafuga.org")
                .product("CMSIS-DAP")
                .serial_number("test")
                .device_class(USB_CLASS_MISCELLANEOUS) 
                .device_class(USB_SUBCLASS_COMMON)
                .device_protocol(USB_PROTOCOL_IAD)
                .composite_with_iads()
                .max_packet_size_0(64)
                .build(),
        );
        LED = Some(pins.led1.into_open_drain_output(&mut pins.port));
    }

    unsafe {
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    unsafe {
        PORT = Some(pins.port);
    }

    loop {
        // unsafe {
        //     USB_DAP.as_mut().map(|dap| {
        //         let _ = dap.process();
        //     });
        // }
        cycle_delay(15 * 1024 * 1024);
        led0.toggle();
    }
}

static mut PORT: Option<Port> = None;
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus, XiaoSwdIo, 64>> = None;
static mut LED: Option<Pa18<Output<OpenDrain>>> = None;

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                USB_DAP.as_mut().map(|dap| {
                    usb_dev.poll(&mut [serial, dap]);

                    dap.process().ok();

                    let mut buf = [0u8; 64];

                    if let Ok(count) = serial.read(&mut buf) {
                        for (i, c) in buf.iter().enumerate() {
                            if i >= count {
                                break;
                            }
                            serial.write(&[c.clone()]).unwrap();
                            LED.as_mut().map(|led| led.toggle());
                        }
                    };
                });
            });
        });
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}
