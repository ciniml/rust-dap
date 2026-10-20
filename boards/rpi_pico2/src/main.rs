// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0

#![no_std]
#![no_main]

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

/// Tell the RP2350 Boot ROM that this is an Arm executable image.
#[link_section = ".start_block"]
#[used]
pub static IMAGE_DEF: rp235x_hal::block::ImageDef = rp235x_hal::block::ImageDef::secure_exe();

/// `#[rtic::app]` bypasses the RP2350 HAL entry point, so release all SIO
/// spinlocks before the first critical section is entered.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    rust_dap_rp::clear_spinlocks();
}

#[rtic::app(
    device = rp235x_hal::pac,
    peripherals = true,
    dispatchers = [SW0_IRQ]
)]
mod app {
    #[cfg(not(feature = "defmt"))]
    use panic_halt as _;
    #[cfg(feature = "defmt")]
    use {defmt_rtt as _, panic_probe as _};

    use embedded_hal::digital::{OutputPin, StatefulOutputPin};
    use hal::clocks::Clock;
    use hal::gpio::{FunctionSioOutput, FunctionUart, Pin, PullDown};
    use hal::pac;
    use rp235x_hal as hal;

    use usb_device::bus::UsbBusAllocator;
    use usb_device::prelude::*;
    use usbd_serial::SerialPort;

    use crate::XOSC_CRYSTAL_FREQ;
    use rust_dap_rp::bridge::{self, UartReader, UartWriter};
    use rust_dap_rp::line_coding::*;
    use rust_dap_rp::util::UartConfigAndClock;

    // util::SwdIoSet/JtagIoSet select the PIO or bit-banging transport via
    // the `bitbang` feature.
    #[cfg(all(feature = "swd", not(feature = "jtag"), not(feature = "swj")))]
    type SwdIoSet = rust_dap_rp::util::SwdIoSet<GpioSwClk, GpioSwdIo, GpioReset>;
    #[cfg(all(feature = "jtag", not(feature = "swj")))]
    type JtagIoSet = rust_dap_rp::util::JtagIoSet<
        JtagTckPin,
        JtagTmsPin,
        JtagTdiPin,
        JtagTdoPin,
        JtagTrstPin,
        JtagResetPin,
    >;
    #[cfg(feature = "swj")]
    type SwjIoSet = rust_dap_rp::bitbang::SwjIoSet<
        GpioSwClk,
        GpioSwdIo,
        JtagTdiPin,
        JtagTdoPin,
        JtagTrstPin,
        GpioReset,
    >;
    // Transports are mutually exclusive; when several are enabled at once
    // (e.g. `--features swj` keeps the default `swd`), swj wins, then jtag,
    // then swd.
    #[cfg(all(feature = "swd", not(feature = "jtag"), not(feature = "swj")))]
    type IoSet = SwdIoSet;
    #[cfg(all(feature = "jtag", not(feature = "swj")))]
    type IoSet = JtagIoSet;
    #[cfg(feature = "swj")]
    type IoSet = SwjIoSet;
    type UsbDap = rust_dap::CmsisDap<'static, hal::usb::UsbBus, IoSet, 64>;

    // GPIO mappings
    type GpioUartTx = hal::gpio::bank0::Gpio0;
    type GpioUartRx = hal::gpio::bank0::Gpio1;
    type GpioUsbLed = hal::gpio::bank0::Gpio25;
    type GpioIdleLed = hal::gpio::bank0::Gpio17;
    type GpioDebugOut = hal::gpio::bank0::Gpio15;
    type GpioDebugIrqOut = hal::gpio::bank0::Gpio28;
    type GpioDebugUsbIrqOut = hal::gpio::bank0::Gpio27;
    // SWD / SWJ shared clock, data and reset pins
    #[cfg(any(feature = "swd", feature = "swj"))]
    type GpioSwClk = hal::gpio::bank0::Gpio2;
    #[cfg(any(feature = "swd", feature = "swj"))]
    type GpioSwdIo = hal::gpio::bank0::Gpio3;
    #[cfg(any(feature = "swd", feature = "swj"))]
    type GpioReset = hal::gpio::bank0::Gpio4;
    // JTAG / SWJ TCK/TMS reuse the SWD clock/data pins; TDI/TDO/TRST are extra
    // pins.
    #[cfg(feature = "jtag")]
    type JtagTckPin = hal::gpio::bank0::Gpio2;
    #[cfg(feature = "jtag")]
    type JtagTmsPin = hal::gpio::bank0::Gpio3;
    #[cfg(any(feature = "jtag", feature = "swj"))]
    type JtagTdoPin = hal::gpio::bank0::Gpio5;
    #[cfg(any(feature = "jtag", feature = "swj"))]
    type JtagTdiPin = hal::gpio::bank0::Gpio6;
    #[cfg(any(feature = "jtag", feature = "swj"))]
    type JtagTrstPin = hal::gpio::bank0::Gpio7;
    #[cfg(feature = "jtag")]
    type JtagResetPin = hal::gpio::bank0::Gpio4;

    const UART_RX_QUEUE_SIZE: usize = 256;
    const UART_TX_QUEUE_SIZE: usize = 128;

    type UartPins = (
        Pin<GpioUartTx, FunctionUart, PullDown>,
        Pin<GpioUartRx, FunctionUart, PullDown>,
    );

    #[shared]
    struct Shared {
        uart_reader: Option<UartReader<pac::UART0, UartPins>>,
        uart_writer: Option<UartWriter<pac::UART0, UartPins>>,
        usb_serial: SerialPort<'static, hal::usb::UsbBus>,
        usb_dap: UsbDap,
        uart_rx_consumer: heapless::spsc::Consumer<'static, u8, UART_RX_QUEUE_SIZE>,
        uart_tx_producer: heapless::spsc::Producer<'static, u8, UART_TX_QUEUE_SIZE>,
        uart_tx_consumer: heapless::spsc::Consumer<'static, u8, UART_TX_QUEUE_SIZE>,
    }

    #[local]
    struct Local {
        uart_config: UartConfigAndClock,
        uart_rx_producer: heapless::spsc::Producer<'static, u8, UART_RX_QUEUE_SIZE>,
        usb_bus: UsbDevice<'static, hal::usb::UsbBus>,
        usb_led: Pin<GpioUsbLed, FunctionSioOutput, PullDown>,
        idle_led: Pin<GpioIdleLed, FunctionSioOutput, PullDown>,
        debug_out: Pin<GpioDebugOut, FunctionSioOutput, PullDown>,
        debug_irq_out: Pin<GpioDebugIrqOut, FunctionSioOutput, PullDown>,
        debug_usb_irq_out: Pin<GpioDebugUsbIrqOut, FunctionSioOutput, PullDown>,
    }

    #[init(local = [
        uart_rx_queue: heapless::spsc::Queue<u8, UART_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        uart_tx_queue: heapless::spsc::Queue<u8, UART_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        USB_ALLOCATOR: Option<UsbBusAllocator<hal::usb::UsbBus>> = None,
    ])]
    fn init(c: init::Context) -> (Shared, Local) {
        let mut resets = c.device.RESETS;
        let sio = hal::Sio::new(c.device.SIO);
        let pins = hal::gpio::Pins::new(
            c.device.IO_BANK0,
            c.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let mut watchdog = hal::Watchdog::new(c.device.WATCHDOG);
        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            c.device.XOSC,
            c.device.CLOCKS,
            c.device.PLL_SYS,
            c.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let serial_number = rust_dap_rp::unique_id::serial_number();

        let uart_pins = (
            pins.gpio0.into_function::<hal::gpio::FunctionUart>(), // TxD
            pins.gpio1.into_function::<hal::gpio::FunctionUart>(), // RxD
        );
        let uart_config = UartConfigAndClock {
            config: UartConfig::from(hal::uart::UartConfig::default()),
            clock: clocks.peripheral_clock.freq(),
        };
        let mut uart = hal::uart::UartPeripheral::new(c.device.UART0, uart_pins, &mut resets)
            .enable((&uart_config.config).into(), uart_config.clock)
            .unwrap();
        uart.enable_rx_interrupt();
        let (uart_reader, uart_writer) = uart.split();
        let uart_reader = Some(UartReader(uart_reader));
        let uart_writer = Some(UartWriter(uart_writer));

        let usb_allocator = UsbBusAllocator::new(hal::usb::UsbBus::new(
            c.device.USB,
            c.device.USB_DPRAM,
            clocks.usb_clock,
            true,
            &mut resets,
        ));
        c.local.USB_ALLOCATOR.replace(usb_allocator);
        let usb_allocator = c.local.USB_ALLOCATOR.as_ref().unwrap();

        #[cfg(all(
            feature = "swd",
            feature = "bitbang",
            not(feature = "jtag"),
            not(feature = "swj")
        ))]
        let (usb_serial, usb_dap, usb_bus) = {
            use rust_dap::{DapConfig, DapIdentity};
            use rust_dap_rp::bitbang::{CortexMDelay, PicoBidirPin};
            use rust_dap_rp::util::UsbIdentity;

            let reset_pin = PicoBidirPin::new(pins.gpio4.into_floating_input());
            let swclk_pin = PicoBidirPin::new(pins.gpio2.into_floating_input());
            let swdio_pin = PicoBidirPin::new(pins.gpio3.into_floating_input());
            let swdio = SwdIoSet::new(swclk_pin, swdio_pin, reset_pin, CortexMDelay);
            rust_dap_rp::util::initialize_usb(
                swdio,
                usb_allocator,
                UsbIdentity {
                    serial: serial_number,
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number,
                        product_firmware_version: env!("GIT_REV"),
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        #[cfg(all(
            feature = "swd",
            not(feature = "bitbang"),
            not(feature = "jtag"),
            not(feature = "swj")
        ))]
        let (usb_serial, usb_dap, usb_bus) = {
            use rust_dap::{DapConfig, DapIdentity};
            use rust_dap_rp::util::UsbIdentity;

            let mut swclk_pin = pins.gpio2.into_function::<hal::gpio::FunctionPio0>();
            let mut swdio_pin = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
            let mut reset_pin = pins.gpio4.into_function::<hal::gpio::FunctionPio0>();
            swclk_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            swdio_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            reset_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);

            let swdio = SwdIoSet::new(
                c.device.PIO0,
                swclk_pin,
                swdio_pin,
                reset_pin,
                clocks.system_clock.freq().to_Hz(),
                &mut resets,
            );
            rust_dap_rp::util::initialize_usb(
                swdio,
                usb_allocator,
                UsbIdentity {
                    serial: serial_number,
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number,
                        product_firmware_version: env!("GIT_REV"),
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        #[cfg(all(feature = "jtag", feature = "bitbang", not(feature = "swj")))]
        let (usb_serial, usb_dap, usb_bus) = {
            use rust_dap::{DapConfig, DapIdentity};
            use rust_dap_rp::bitbang::{CortexMDelay, PicoBidirPin};
            use rust_dap_rp::util::UsbIdentity;

            let tck_pin = PicoBidirPin::new(pins.gpio2.into_floating_input());
            let tms_pin = PicoBidirPin::new(pins.gpio3.into_floating_input());
            let tdo_pin = PicoBidirPin::new(pins.gpio5.into_floating_input());
            let tdi_pin = PicoBidirPin::new(pins.gpio6.into_floating_input());
            let trst_pin = PicoBidirPin::new(pins.gpio7.into_floating_input());
            let srst_pin = PicoBidirPin::new(pins.gpio4.into_floating_input());
            let jtagio = JtagIoSet::new(
                tck_pin,
                tms_pin,
                tdi_pin,
                tdo_pin,
                trst_pin,
                srst_pin,
                CortexMDelay,
            );
            rust_dap_rp::util::initialize_usb(
                jtagio,
                usb_allocator,
                UsbIdentity {
                    serial: serial_number,
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number,
                        product_firmware_version: env!("GIT_REV"),
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        #[cfg(all(feature = "jtag", not(feature = "bitbang"), not(feature = "swj")))]
        let (usb_serial, usb_dap, usb_bus) = {
            use rust_dap::{DapConfig, DapIdentity};
            use rust_dap_rp::util::UsbIdentity;

            let mut tck_pin = pins.gpio2.into_function::<hal::gpio::FunctionPio0>();
            let mut tms_pin = pins.gpio3.into_function::<hal::gpio::FunctionPio0>();
            let mut tdo_pin = pins.gpio5.into_function::<hal::gpio::FunctionPio0>();
            let mut tdi_pin = pins.gpio6.into_function::<hal::gpio::FunctionPio0>();
            let mut trst_pin = pins.gpio7.into_function::<hal::gpio::FunctionPio0>();
            let mut srst_pin = pins.gpio4.into_function::<hal::gpio::FunctionPio0>();
            tck_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            tms_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            tdo_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            tdi_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            trst_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);
            srst_pin.set_slew_rate(hal::gpio::OutputSlewRate::Fast);

            let jtagio = JtagIoSet::new(
                c.device.PIO0,
                tck_pin,
                tms_pin,
                tdi_pin,
                tdo_pin,
                Some(trst_pin),
                Some(srst_pin),
                clocks.system_clock.freq().to_Hz(),
                &mut resets,
            );
            rust_dap_rp::util::initialize_usb(
                jtagio,
                usb_allocator,
                UsbIdentity {
                    serial: serial_number,
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number,
                        product_firmware_version: env!("GIT_REV"),
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        // SWJ switches between SWD and JTAG at runtime, so it intentionally
        // uses the common bit-banging transport.
        #[cfg(feature = "swj")]
        let (usb_serial, usb_dap, usb_bus) = {
            use rust_dap::{DapConfig, DapIdentity};
            use rust_dap_rp::bitbang::{CortexMDelay, PicoBidirPin};
            use rust_dap_rp::util::UsbIdentity;

            let clk_pin = PicoBidirPin::new(pins.gpio2.into_floating_input());
            let dio_pin = PicoBidirPin::new(pins.gpio3.into_floating_input());
            let tdi_pin = PicoBidirPin::new(pins.gpio6.into_floating_input());
            let tdo_pin = PicoBidirPin::new(pins.gpio5.into_floating_input());
            let trst_pin = PicoBidirPin::new(pins.gpio7.into_floating_input());
            let srst_pin = PicoBidirPin::new(pins.gpio4.into_floating_input());
            let swjio = SwjIoSet::new(
                clk_pin,
                dio_pin,
                tdi_pin,
                tdo_pin,
                trst_pin,
                srst_pin,
                CortexMDelay,
            );
            rust_dap_rp::util::initialize_usb(
                swjio,
                usb_allocator,
                UsbIdentity {
                    serial: serial_number,
                    ..UsbIdentity::default()
                },
                DapConfig::new(
                    DapIdentity {
                        serial_number,
                        product_firmware_version: env!("GIT_REV"),
                        ..DapIdentity::default()
                    },
                    clocks.system_clock.freq().to_Hz(),
                ),
            )
        };

        let usb_led = pins.gpio25.into_push_pull_output();
        let (uart_rx_producer, uart_rx_consumer) = c.local.uart_rx_queue.split();
        let (uart_tx_producer, uart_tx_consumer) = c.local.uart_tx_queue.split();

        let mut debug_out = pins.gpio15.into_push_pull_output();
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
        )
    }

    #[idle(
        shared = [
            uart_reader,
            uart_writer,
            usb_serial,
            uart_rx_consumer,
            uart_tx_producer,
            uart_tx_consumer
        ],
        local = [idle_led]
    )]
    fn idle(mut c: idle::Context) -> ! {
        loop {
            (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer).lock(
                |usb_serial, uart_tx_producer| {
                    bridge::drain_usb_to_uart_tx(usb_serial, uart_tx_producer)
                },
            );
            (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer).lock(
                |uart_writer, uart_tx_consumer| {
                    bridge::drain_uart_tx_queue(uart_writer, uart_tx_consumer)
                },
            );

            let rx_dequeued = (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer).lock(
                |usb_serial, uart_rx_consumer| {
                    // Forward only while the host has the port open (DTR); drop
                    // otherwise so a closed session's tail doesn't leak to the
                    // next open. Discarding leaves the queue empty (room again).
                    if usb_serial.dtr() {
                        bridge::drain_uart_rx_queue(usb_serial, uart_rx_consumer)
                    } else {
                        bridge::discard_uart_rx_queue(uart_rx_consumer);
                        true
                    }
                },
            );
            if rx_dequeued {
                c.shared
                    .uart_reader
                    .lock(|uart| uart.as_mut().unwrap().0.enable_rx_interrupt());
            }

            c.local.idle_led.toggle().ok();
        }
    }

    #[task(
        binds = UART0_IRQ,
        priority = 3,
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
    /// SWD/JTAG transfers (transfer retries, DAP_SWJ_Pins waits, etc.) cannot
    /// block the UART interrupt.
    ///
    /// This is a hardware task on an otherwise unused interrupt, pended from
    /// `usbctrl_irq`, rather than an RTIC software task: `spawn()` fails when
    /// the task is already running, and a request that arrives during a long
    /// command then has nothing left to process it — the OUT packet was
    /// already read from the endpoint, so no further USB interrupt comes and
    /// the probe stops answering until a bus reset (#87). Pending an NVIC
    /// interrupt is idempotent and never lost: if the task is running it
    /// simply runs again afterwards.
    #[task(binds = SW1_IRQ, priority = 1, shared = [usb_dap])]
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
            return;
        }
        rtic::pend(pac::Interrupt::SW1_IRQ);

        (&mut c.shared.usb_serial, &mut c.shared.uart_tx_producer).lock(
            |usb_serial, uart_tx_producer| {
                bridge::drain_usb_to_uart_tx(usb_serial, uart_tx_producer)
            },
        );
        (&mut c.shared.uart_writer, &mut c.shared.uart_tx_consumer).lock(
            |uart_writer, uart_tx_consumer| {
                bridge::drain_uart_tx_queue(uart_writer, uart_tx_consumer)
            },
        );

        let rx_dequeued = (&mut c.shared.usb_serial, &mut c.shared.uart_rx_consumer).lock(
            |usb_serial, uart_rx_consumer| {
                // See the USB-task drain: only forward while DTR is asserted,
                // otherwise discard so a closed session's tail doesn't leak.
                if usb_serial.dtr() {
                    bridge::drain_uart_rx_queue(usb_serial, uart_rx_consumer)
                } else {
                    bridge::discard_uart_rx_queue(uart_rx_consumer);
                    true
                }
            },
        );
        if rx_dequeued {
            c.shared
                .uart_reader
                .lock(|uart| uart.as_mut().unwrap().0.enable_rx_interrupt());
        }

        // A zero baud rate makes `UartConfig::try_from` fail, so
        // `reconfigure_uart` is not called when the host closes the CDC port.
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
