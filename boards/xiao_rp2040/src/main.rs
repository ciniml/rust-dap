// Copyright 2021-2022 Kenta Ida
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

/// The linker will place this boot block at the start of our program image.
/// We need this to help the ROM bootloader get our code up and running.
/// W25Q080 matches the flash chip of this board; execute-in-SRAM builds use
/// the RAM_MEMCPY loader instead.
#[cfg(feature = "ram-exec")]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_RAM_MEMCPY;
#[cfg(not(feature = "ram-exec"))]
#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

/// `#[rtic::app]` bypasses `#[rp2040_hal::entry]`, so the SIO spinlocks that
/// the hal entry point would normally release must be released here.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    rust_dap_rp2040::clear_spinlocks();
}

#[rtic::app(device = rp_pico::hal::pac, peripherals = true, dispatchers = [PIO1_IRQ_0])]
mod app {
    use panic_halt as _;

    use hal::clocks::Clock;
    use hal::gpio::{Output, Pin, PushPull};
    use hal::pac;
    use rp_pico::hal;

    use hal::usb::UsbBus;
    use usb_device::bus::UsbBusAllocator;

    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    use embedded_hal::digital::v2::{OutputPin, ToggleableOutputPin};

    use rust_dap::{DapConfig, DapIdentity};
    use rust_dap_rp2040::line_coding::*;
    use rust_dap_rp2040::util::{UartConfigAndClock, UsbIdentity};
    type SwdIoSet = rust_dap_rp2040::util::SwdIoSet<GpioSwClk, GpioSwdIo, GpioReset>;
    type UsbDap = rust_dap::CmsisDap<'static, UsbBus, SwdIoSet, 64>;

    // GPIO mappings
    type GpioSwClk = hal::gpio::bank0::Gpio2;
    type GpioSwdIo = hal::gpio::bank0::Gpio4;
    type GpioReset = hal::gpio::bank0::Gpio26;
    type GpioUartTx = hal::gpio::bank0::Gpio0;
    type GpioUartRx = hal::gpio::bank0::Gpio1;
    type GpioUsbLed = hal::gpio::bank0::Gpio25;
    type GpioIdleLed = hal::gpio::bank0::Gpio17;
    type GpioDebugOut = hal::gpio::bank0::Gpio6;
    type GpioDebugIrqOut = hal::gpio::bank0::Gpio28;
    type GpioDebugUsbIrqOut = hal::gpio::bank0::Gpio27;

    // UART Interrupt context
    const UART_RX_QUEUE_SIZE: usize = 256;
    const UART_TX_QUEUE_SIZE: usize = 128;
    // UART Shared context
    type UartPins = (
        hal::gpio::Pin<GpioUartTx, hal::gpio::Function<hal::gpio::Uart>>,
        hal::gpio::Pin<GpioUartRx, hal::gpio::Function<hal::gpio::Uart>>,
    );
    use rust_dap_rp2040::bridge::{self, UartReader, UartWriter};

    #[shared]
    struct Shared {
        uart_reader: Option<UartReader<pac::UART0, UartPins>>,
        uart_writer: Option<UartWriter<pac::UART0, UartPins>>,
        usb_serial: SerialPort<'static, UsbBus>,
        usb_dap: UsbDap,
        uart_rx_consumer: heapless::spsc::Consumer<'static, u8, UART_RX_QUEUE_SIZE>,
        uart_tx_producer: heapless::spsc::Producer<'static, u8, UART_TX_QUEUE_SIZE>,
        uart_tx_consumer: heapless::spsc::Consumer<'static, u8, UART_TX_QUEUE_SIZE>,
    }

    #[local]
    struct Local {
        uart_config: UartConfigAndClock,
        uart_rx_producer: heapless::spsc::Producer<'static, u8, UART_RX_QUEUE_SIZE>,
        usb_bus: UsbDevice<'static, UsbBus>,
        usb_led: Pin<GpioUsbLed, Output<PushPull>>,
        idle_led: Pin<GpioIdleLed, Output<PushPull>>,
        debug_out: Pin<GpioDebugOut, Output<PushPull>>,
        debug_irq_out: Pin<GpioDebugIrqOut, Output<PushPull>>,
        debug_usb_irq_out: Pin<GpioDebugUsbIrqOut, Output<PushPull>>,
    }

    #[init(local = [
        uart_rx_queue: heapless::spsc::Queue<u8, UART_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        uart_tx_queue: heapless::spsc::Queue<u8, UART_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        USB_ALLOCATOR: Option<UsbBusAllocator<UsbBus>> = None,
        ])]
    fn init(c: init::Context) -> (Shared, Local, init::Monotonics) {
        let mut resets = c.device.RESETS;
        let sio = hal::Sio::new(c.device.SIO);
        let pins = rp_pico::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            rp_pico::XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let uart_pins = (
            pins.gpio0.into_mode::<hal::gpio::FunctionUart>(), // TxD
            pins.gpio1.into_mode::<hal::gpio::FunctionUart>(), // RxD
        );
        let uart_config = UartConfigAndClock {
            config: UartConfig::from(hal::uart::UartConfig::default()),
            clock: clocks.peripheral_clock.freq(),
        };
        let mut uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable((&uart_config.config).into(), uart_config.clock)
            .unwrap();
        // Enable RX interrupt. Note that TX interrupt is enabled when some TX data is available.
        uart.enable_rx_interrupt();
        let (uart_reader, uart_writer) = uart.split();
        let uart_reader = Some(UartReader(uart_reader));
        let uart_writer = Some(UartWriter(uart_writer));

        let usb_allocator = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USBCTRL_REGS,
            c.device.USBCTRL_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        c.local.USB_ALLOCATOR.replace(usb_allocator);
        let usb_allocator = c.local.USB_ALLOCATOR.as_ref().unwrap();

        let (usb_serial, usb_dap, usb_bus) = {
            // Initialize MCU reset pin.
            // RESET pin of Cortex Debug 10-pin connector is negative logic
            // https://developer.arm.com/documentation/101453/0100/CoreSight-Technology/Connectors
            let reset_pin = pins.gpio26.into_floating_input();

            let swdio;
            #[cfg(feature = "bitbang")]
            {
                use rust_dap_rp2040::bitbang::{CortexMDelay, PicoBidirPin};
                let swclk_pin = PicoBidirPin::new(pins.gpio2.into_floating_input());
                let swdio_pin = PicoBidirPin::new(pins.gpio4.into_floating_input());
                let reset_pin = PicoBidirPin::new(reset_pin);
                swdio = SwdIoSet::new(swclk_pin, swdio_pin, reset_pin, CortexMDelay);
            }
            #[cfg(not(feature = "bitbang"))]
            {
                let mut swclk_pin = pins.gpio2.into_mode();
                let mut swdio_pin = pins.gpio4.into_mode();
                let mut reset_pin = reset_pin.into_mode();
                swclk_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                swdio_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                reset_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
                swdio = SwdIoSet::new(c.device.PIO0, swclk_pin, swdio_pin, reset_pin, &mut resets);
            }
            rust_dap_rp2040::util::initialize_usb(
                swdio,
                usb_allocator,
                UsbIdentity {
                    serial: "xiao-rp2040",
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number: "xiao-rp2040",
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        let usb_led = pins.led.into_push_pull_output();
        let (uart_rx_producer, uart_rx_consumer) = c.local.uart_rx_queue.split();
        let (uart_tx_producer, uart_tx_consumer) = c.local.uart_tx_queue.split();

        let mut debug_out = pins.gpio6.into_push_pull_output();
        debug_out.set_low().ok();
        let mut debug_irq_out = pins.gpio28.into_push_pull_output();
        debug_irq_out.set_low().ok();
        let mut debug_usb_irq_out = pins.gpio27.into_push_pull_output();
        debug_usb_irq_out.set_low().ok();

        pins.gpio16.into_push_pull_output().set_high().ok();
        let mut idle_led = pins.gpio17.into_push_pull_output();
        idle_led.set_high().ok();
        (
            Shared {
                uart_reader,
                uart_writer,
                usb_serial,
                usb_dap,
                uart_rx_consumer,
                uart_tx_producer,
                uart_tx_consumer,
            },
            Local {
                uart_config,
                uart_rx_producer,
                usb_bus,
                usb_led,
                idle_led,
                debug_out,
                debug_irq_out,
                debug_usb_irq_out,
            },
            init::Monotonics(),
        )
    }

    #[idle(shared = [uart_reader, uart_writer, usb_serial, uart_rx_consumer, uart_tx_producer, uart_tx_consumer], local = [idle_led])]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer)
                .lock(|usb_serial, uart_tx_producer| {
                    bridge::drain_usb_to_uart_tx(usb_serial, uart_tx_producer)
                });
            (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer)
                .lock(|uart_writer, uart_tx_consumer| {
                    bridge::drain_uart_tx_queue(uart_writer, uart_tx_consumer)
                });

            // Process RX data.
            let rx_dequeued = (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer)
                .lock(|usb_serial, uart_rx_consumer| {
                    bridge::drain_uart_rx_queue(usb_serial, uart_rx_consumer)
                });
            if rx_dequeued {
                // The RX queue has room again, so restart the UART RX interrupt
                // in case uart_irq stopped it while the queue was full.
                c.shared
                    .uart_reader
                    .lock(|uart| uart.as_mut().unwrap().0.enable_rx_interrupt());
            }

            c.local.idle_led.toggle().ok();
        }
    }

    #[task(
        binds = UART0_IRQ,
        priority = 3,   // Higher priority than USBCTRL_IRQ and dap_process so that UART RX data is not lost during long DAP transfers.
        shared = [uart_reader],
        local = [uart_rx_producer, debug_out, debug_irq_out],
    )]
    fn uart_irq(mut c: uart_irq::Context) {
        c.local.debug_irq_out.set_high().ok();
        let debug_out = c.local.debug_out;
        let uart_rx_producer = c.local.uart_rx_producer;
        c.shared.uart_reader.lock(|uart_reader| {
            bridge::on_uart_rx_irq(uart_reader, uart_rx_producer, || {
                debug_out.toggle().ok();
            })
        });
        c.local.debug_irq_out.set_low().ok();
    }

    /// Processes CMSIS-DAP commands outside of the USB interrupt so that long
    /// SWD transfers (transfer retries, DAP_SWJ_Pins waits, etc.) cannot
    /// block the UART interrupt.
    #[task(priority = 1, capacity = 2, shared = [usb_dap])]
    fn dap_process(mut c: dap_process::Context) {
        c.shared.usb_dap.lock(|usb_dap| {
            usb_dap.process().ok();
        });
    }

    #[task(
        binds = USBCTRL_IRQ,
        priority = 2,
        shared = [uart_reader, uart_writer, usb_serial, usb_dap, uart_rx_consumer, uart_tx_producer, uart_tx_consumer],
        local = [usb_bus, uart_config, usb_led, debug_usb_irq_out],
    )]
    fn usbctrl_irq(mut c: usbctrl_irq::Context) {
        c.local.debug_usb_irq_out.set_high().ok();

        let poll_result = (&mut c.shared.usb_serial, &mut c.shared.usb_dap)
            .lock(|usb_serial, usb_dap| c.local.usb_bus.poll(&mut [usb_serial, usb_dap]));
        if !poll_result {
            c.local.debug_usb_irq_out.set_low().ok();
            return; // Nothing to do at this time...
        }
        // Defer DAP command processing to the low priority dap_process task.
        dap_process::spawn().ok();

        // Process TX data.
        (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer)
            .lock(|usb_serial, uart_tx_producer| bridge::drain_usb_to_uart_tx(usb_serial, uart_tx_producer));
        (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer)
            .lock(|uart_writer, uart_tx_consumer| bridge::drain_uart_tx_queue(uart_writer, uart_tx_consumer));

        // Process RX data.
        let rx_dequeued = (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer)
            .lock(|usb_serial, uart_rx_consumer| bridge::drain_uart_rx_queue(usb_serial, uart_rx_consumer));
        if rx_dequeued {
            // The RX queue has room again, so restart the UART RX interrupt
            // in case uart_irq stopped it while the queue was full.
            c.shared
                .uart_reader
                .lock(|uart| uart.as_mut().unwrap().0.enable_rx_interrupt());
        }

        // Check if the UART transmitter must be re-configured.
        if let Ok(expected_config) = c
            .shared
            .usb_serial
            .lock(|usb_serial| UartConfig::try_from(usb_serial.line_coding()))
        {
            if expected_config != c.local.uart_config.config {
                (&mut c.shared.uart_reader, &mut c.shared.uart_writer).lock(|reader, writer| {
                    bridge::reconfigure_uart(reader, writer, c.local.uart_config, &expected_config)
                });
            }
        }

        c.local.usb_led.toggle().ok();
        c.local.debug_usb_irq_out.set_low().ok();
    }
}
