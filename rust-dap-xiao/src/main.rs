// Copyright 2021 Kenta Ida
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

#![no_std]
#![no_main]

use core::sync::atomic::{AtomicU32, Ordering};

use bsp::hal::sercom::v2::uart::Oversampling;
use embedded_hal::digital::v2::{InputPin, OutputPin, ToggleableOutputPin};
use panic_halt as _;
use rust_dap::DapError;
use rust_dap::SwdIo;
use rust_dap::SwdIoConfig;
use rust_dap::SwdRequest;
use rust_dap::DAP_TRANSFER_MISMATCH;
use rust_dap::USB_CLASS_MISCELLANEOUS;
use rust_dap::USB_PROTOCOL_IAD;
use rust_dap::USB_SUBCLASS_COMMON;
use rust_dap::CmsisDap;

use xiao_m0 as bsp;
use bsp::{entry, hal, pac};
use pac::{interrupt, CorePeripherals, Peripherals};
use hal::prelude::*;
use hal::clock::GenericClockController;
use hal::usb::UsbBus;
use hal::gpio::v2::{Floating, Input, Output, Pin, PushPull};
use hal::sercom::v2::uart;
use usb_device::{bus::UsbBusAllocator, prelude::*};
use usbd_serial::{SerialPort};

use cortex_m::asm::delay as cycle_delay;
use cortex_m::peripheral::NVIC;

// Import pin types
use hal::gpio::v2::{PA05, PA07};

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
        if let Some(old) = self.swdio_in.take() {
            let mut new = old.into_push_pull_output();
            new.set_low().ok();
            self.swdio_out = Some(new);
        }
        if let Some(old) = self.swclk_in.take() {
            let mut new = old.into_push_pull_output();
            new.set_low().ok();
            self.swclk_out = Some(new);
        }
    }
    fn disconnect(&mut self) {
        if let Some(old) = self.swdio_out.take() {
            let new = old.into_floating_input();
            self.swdio_in = Some(new);
        }
        if let Some(old) = self.swclk_out.take() {
            let new = old.into_floating_input();
            self.swclk_in = Some(new);
        }
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
        if let Some(old) = self.swdio_in.take() {
            let mut new = old.into_push_pull_output();
            new.set_low().ok();
            self.swdio_out = Some(new);
        }
    }

    fn disable_output(&mut self) {
        if let Some(old) = self.swdio_out.take() {
            let new = old.into_floating_input();
            self.swdio_in = Some(new);
        }
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

    let pins = bsp::Pins::new(peripherals.PORT);
    let mut led0 = pins.led0.into_push_pull_output();
    
    let bus_allocator = unsafe {
        USB_ALLOCATOR = Some(bsp::usb_allocator(
            peripherals.USB,
            &mut clocks,
            &mut peripherals.PM,
            pins.usb_dm,
            pins.usb_dp,
        ));
        USB_ALLOCATOR.as_ref().unwrap()
    };

    let swdio = XiaoSwdIo {
        swclk_in: Some(pins.a8.into_floating_input()),
        swdio_in: Some(pins.a9.into_floating_input()),
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
        LED = Some(pins.led1.into_push_pull_output());
        UART_SERIAL = Some(bsp::uart(&mut clocks, DEFAULT_BAUD_RATE.hz(), peripherals.SERCOM4, &mut peripherals.PM, pins.a7, pins.a6));
    }

    // The above shared context MUST BE initialized before enabling interrupts.
    unsafe {
        // Configure UART (SERCOM4) interrupt
        UART_SERIAL.as_mut().map(|uart| {
            enable_uart_interrupts(uart);
        });
        core.NVIC.set_priority(interrupt::SERCOM4, 2);
        NVIC::unmask(interrupt::SERCOM4);
        // USB interrupt
        core.NVIC.set_priority(interrupt::USB, 1);
        NVIC::unmask(interrupt::USB);
    }

    let mut current_baud_rate = DEFAULT_BAUD_RATE;
    loop {
        unsafe {
            UART_SERIAL.as_mut().map(|uart| {
                let expected_baud_rate = BAUD_RATE.load(Ordering::Relaxed);
                if current_baud_rate != expected_baud_rate {
                    // Update the UART configuration.
                    NVIC::mask(interrupt::USB);
                    disable_uart_interrupts(uart);
                    uart.reconfigure(|config| config.set_baud((expected_baud_rate as u32).hz(), uart::BaudMode::Arithmetic(uart::Oversampling::Bits16)));
                    enable_uart_interrupts(uart);
                    NVIC::unmask(interrupt::USB);
                    current_baud_rate = expected_baud_rate;
                }
            });
            USB_SERIAL.as_mut().map(|serial| {       
                let mut buf: [u8; 64] = core::mem::MaybeUninit::uninit().assume_init();
                let mut serial_read_consumer = USB_SERIAL_READ_QUEUE.split().1;
                let mut index = 0;
                while index < buf.len() {
                    if let Some(c) = serial_read_consumer.dequeue() {
                        buf[index] = c;
                        index += 1;
                    } else {
                        break;
                    }
                }
                if index > 0 {
                    serial.write(&buf[0..index]).ok();
                }
            });
        }
        //cycle_delay(15 * 1024 * 1024);
        led0.toggle().unwrap();
    }
}

fn enable_uart_interrupts(uart: &mut bsp::Uart)
{
    let serial_write_consumer = unsafe { USB_SERIAL_WRITE_QUEUE.split().1 };
    let flags = if serial_write_consumer.ready() { uart::Flags::RXC | uart::Flags::DRE } else { uart::Flags::RXC };
    uart.enable_interrupts(flags);
}
fn disable_uart_interrupts(uart: &mut bsp::Uart)
{
    uart.disable_interrupts(uart::Flags::RXC | uart::Flags::DRE);
}

const UART_READ_QUEUE_SIZE: usize = 128;
const UART_WRITE_QUEUE_SIZE: usize = 128;
static mut USB_SERIAL_READ_QUEUE: heapless::spsc::Queue<u8, UART_READ_QUEUE_SIZE> = heapless::spsc::Queue::new();
static mut USB_SERIAL_WRITE_QUEUE: heapless::spsc::Queue<u8, UART_WRITE_QUEUE_SIZE> = heapless::spsc::Queue::new();
static mut UART_SERIAL: Option<bsp::Uart> = None;
static mut USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None;
static mut USB_BUS: Option<UsbDevice<UsbBus>> = None;
static mut USB_SERIAL: Option<SerialPort<UsbBus>> = None;
static mut USB_DAP: Option<CmsisDap<UsbBus, XiaoSwdIo, 64>> = None;
static mut LED: Option<bsp::Led1> = None;

const DEFAULT_BAUD_RATE: u32 = 115200u32;
static mut BAUD_RATE: AtomicU32 = AtomicU32::new(DEFAULT_BAUD_RATE);

fn poll_usb() {
    unsafe {
        USB_BUS.as_mut().map(|usb_dev| {
            USB_SERIAL.as_mut().map(|serial| {
                USB_DAP.as_mut().map(|dap| {
                    usb_dev.poll(&mut [serial, dap]);

                    dap.process().ok();

                    // Store the new baudrate.
                    let baud_rate = serial.line_coding().data_rate();
                    BAUD_RATE.store(baud_rate, Ordering::Relaxed);
                
                    let mut buf: [u8; 64] = core::mem::MaybeUninit::uninit().assume_init();
                    let mut serial_write_producer = USB_SERIAL_WRITE_QUEUE.split().0;
                    if let Ok(count) = serial.read(&mut buf) {
                        for c in buf[0..count].iter() {
                            if serial_write_producer.enqueue(*c).is_err() {
                                break;
                            }
                        }
                    };

                    // Enable UART interrupts.
                    UART_SERIAL.as_mut().map(|uart| enable_uart_interrupts(uart));

                });
            });
        });
        LED.as_mut().map(|led| led.toggle());
    };
}

#[interrupt]
fn USB() {
    poll_usb();
}

// UART interrupts 
#[interrupt]
fn SERCOM4() {
    let mut serial_read_producer = unsafe { USB_SERIAL_READ_QUEUE.split().0 };
    let mut serial_write_consumer = unsafe { USB_SERIAL_WRITE_QUEUE.split().1  };
    unsafe {
        UART_SERIAL.as_mut().map(|uart| {
            let flags = uart.read_flags();
            if flags.contains(uart::Flags::DRE) {
                if let Some(c) = serial_write_consumer.dequeue() {
                    uart.write_data(c as u16);
                } else {
                    uart.disable_interrupts(uart::Flags::DRE);  // Disable DRE interrupt if there are no data in the queue.
                }
            }
            if flags.contains(uart::Flags::RXC) {
                let data = uart.read_data() as u8;
                serial_read_producer.enqueue_unchecked(data);
            }
        });
    }
}