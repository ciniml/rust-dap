// Copyright 2026 Kenta Ida
//
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

//! GDB debugger milestone M3: a standalone gdbserver on the probe RP2040.
//!
//! Presents a USB-CDC serial port that speaks the GDB Remote Serial Protocol
//! (via the `gdbstub` crate, no_std/no-alloc). GDB attaches directly to the
//! probe — no OpenOCD/probe-rs needed:
//!
//!   gdb-multiarch -ex 'target remote /dev/ttyACMx' \
//!                 -ex 'set architecture armv4t' -ex 'info registers'
//!
//! The gdbstub `Target` is backed by the `arm-debug` ADIv5 + Cortex-M stack
//! over a bit-banging SWD transport. Supports: read/write registers, read/write
//! memory, continue, single-step, Ctrl-C interrupt, and RAM software
//! breakpoints (BKPT patching). Pins: GPIO2=SWCLK, GPIO3=SWDIO, GPIO4=RESET.

#![no_std]
#![no_main]

const XOSC_CRYSTAL_FREQ: u32 = 12_000_000;

use arm_debug::ArmDebug;
use core::convert::Infallible;
use gdb_server_core::{core_tid, GdbTarget};
use rp2040_hal as hal;

use gdbstub::common::Signal;
use gdbstub::conn::Connection;
use gdbstub::stub::state_machine::GdbStubStateMachine;
use gdbstub::stub::{GdbStubBuilder, MultiThreadStopReason};

use hal::clocks::Clock;
use hal::pac;
use rust_dap::{
    DapConfig, DapIdentity, USB_CLASS_MISCELLANEOUS, USB_PROTOCOL_IAD, USB_SUBCLASS_COMMON,
};
use rust_dap_rp::bitbang::{CortexMDelay, PicoBidirPin, SwdIoSet};
#[allow(unused_imports)]
use rust_dap_rp::bridge::{UartReader, UartWriter};
#[allow(unused_imports)]
use rust_dap_rp::line_coding::UartConfig;
#[allow(unused_imports)]
use rust_dap_rp::util::UartConfigAndClock;
use usb_device::prelude::*;
use usbd_serial::SerialPort;

#[link_section = ".boot2"]
#[no_mangle]
#[used]
pub static BOOT2_FIRMWARE: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

type Swd = SwdIoSet<hal::gpio::bank0::Gpio2, hal::gpio::bank0::Gpio3, hal::gpio::bank0::Gpio4>;

// The gdbstub Target (arm-debug over SWD, target families, RTT) lives in
// the board-independent `gdb-server-core` crate; this binary supplies the
// bit-banged SWD transport, the USB-CDC Connection, and the session loop.
type ProbeTarget = GdbTarget<Swd, CortexMDelay>;

// --- USB-CDC as a gdbstub Connection --------------------------------------

const RX_QUEUE_SIZE: usize = 1024;
const TX_QUEUE_SIZE: usize = 2048;
/// RTT bridge queues (second CDC port <-> target ring buffers).
const RTT_TX_QUEUE_SIZE: usize = 2048;
const RTT_RX_QUEUE_SIZE: usize = 256;

/// UART bridge (optional third CDC port <-> target UART0 on GPIO0/1).
#[allow(dead_code)]
const UART_RX_QUEUE_SIZE: usize = 256;
#[allow(dead_code)]
const UART_TX_QUEUE_SIZE: usize = 128;
#[allow(dead_code)]
type UartPins = (
    hal::gpio::Pin<hal::gpio::bank0::Gpio0, hal::gpio::FunctionUart, hal::gpio::PullDown>,
    hal::gpio::Pin<hal::gpio::bank0::Gpio1, hal::gpio::FunctionUart, hal::gpio::PullDown>,
);

/// The idle-side view of the USB connection: lock-free SPSC queues shared
/// with the USBCTRL_IRQ task, which owns the USB device and services it at
/// interrupt priority. This keeps USB responsive during long blocking SWD
/// operations (flash erases, connect sequences) that previously starved the
/// polled `pump()` loop.
struct QueueConn {
    rx: heapless::spsc::Consumer<'static, u8, RX_QUEUE_SIZE>,
    tx: heapless::spsc::Producer<'static, u8, TX_QUEUE_SIZE>,
    /// One byte un-read by drop_stray_acks, delivered by the next read_byte.
    pushback: Option<u8>,
}

impl QueueConn {
    /// Nudge the USB task so it drains the TX queue without waiting for the
    /// next host-initiated bus event.
    fn kick() {
        pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ);
    }
    /// Try to read one incoming byte.
    fn read_byte(&mut self) -> Option<u8> {
        if let Some(b) = self.pushback.take() {
            return Some(b);
        }
        self.rx.dequeue()
    }
    /// Flush pending TX (bounded, in case the host stopped reading), then
    /// drop any unread RX, so a new GDB session starts with a clean channel
    /// (stale detach responses desync the next session's RSP).
    fn purge(&mut self) {
        for _ in 0..500_000 {
            if self.tx.len() == 0 {
                break;
            }
            Self::kick();
        }
        self.pushback = None;
        while self.rx.dequeue().is_some() {}
    }
}

impl Connection for QueueConn {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        // If the queue is full, keep nudging the USB task to make room (it
        // preempts this priority, so progress only stalls while the host
        // isn't reading). Bounded: if the host has stopped reading entirely
        // the queue never drains, so cap the spins and drop the byte rather
        // than hang the RSP task forever. A dropped byte desyncs the current
        // session's RSP framing, but the next GDB session rebuilds the stub
        // and purges the channel — recoverable, unlike a permanent hang.
        for _ in 0..1_000_000 {
            match self.tx.enqueue(byte) {
                Ok(()) => return Ok(()),
                Err(_) => Self::kick(),
            }
        }
        Ok(())
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Self::kick();
        Ok(())
    }
}

/// Borrow of the connection for one GDB session. The gdbstub state machine
/// carries per-session protocol state (notably no-ack mode negotiated via
/// QStartNoAckMode), so it cannot be reused across sessions: the next GDB
/// starts in ack mode and its leading '+' errors a stub still in no-ack mode.
/// Handing the machine a reborrow lets the session loop rebuild it per
/// session while keeping ownership of the queues.
struct ConnRef<'b>(&'b mut QueueConn);

impl Connection for ConnRef<'_> {
    type Error = Infallible;
    fn write(&mut self, byte: u8) -> Result<(), Self::Error> {
        Connection::write(self.0, byte)
    }
    fn flush(&mut self) -> Result<(), Self::Error> {
        Connection::flush(self.0)
    }
}

/// `#[rtic::app]` bypasses `#[rp2040_hal::entry]`, so the SIO spinlocks that
/// the hal entry point would normally release must be released here.
#[cortex_m_rt::pre_init]
unsafe fn pre_init() {
    rust_dap_rp::clear_spinlocks();
}

/// Set by the USB task once the host has configured the device. A plain
/// atomic (thumbv6 supports load/store) instead of an RTIC shared resource.
static USB_CONFIGURED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);

#[rtic::app(device = rp2040_hal::pac, peripherals = true)]
mod app {
    use super::*;
    use usb_device::class_prelude::UsbBusAllocator;

    // UART reader is shared: UART0_IRQ drains the RX FIFO, and USBCTRL_IRQ needs
    // it (with the writer) to re-configure the UART on a host CDC line-coding
    // change. Always present (RTIC 2.3 mis-handles #[cfg] on shared fields);
    // it is `None` and unused when the uart-bridge feature is off.
    #[shared]
    struct Shared {
        uart_reader: Option<UartReader<pac::UART0, UartPins>>,
    }

    #[local]
    struct Local {
        // --- USBCTRL_IRQ task ---
        usb_dev: UsbDevice<'static, hal::usb::UsbBus>,
        serial: SerialPort<'static, hal::usb::UsbBus>,
        rtt_serial: SerialPort<'static, hal::usb::UsbBus>,
        rx_prod: heapless::spsc::Producer<'static, u8, RX_QUEUE_SIZE>,
        tx_cons: heapless::spsc::Consumer<'static, u8, TX_QUEUE_SIZE>,
        rtt_tx_cons: heapless::spsc::Consumer<'static, u8, RTT_TX_QUEUE_SIZE>,
        rtt_rx_prod: heapless::spsc::Producer<'static, u8, RTT_RX_QUEUE_SIZE>,
        // --- UART bridge: USBCTRL_IRQ side (third CDC <-> UART TX/RX queues) ---
        #[cfg(feature = "uart-bridge")]
        uart_serial: SerialPort<'static, hal::usb::UsbBus>,
        #[cfg(feature = "uart-bridge")]
        uart_writer: Option<UartWriter<pac::UART0, UartPins>>,
        #[cfg(feature = "uart-bridge")]
        uart_rx_cons: heapless::spsc::Consumer<'static, u8, UART_RX_QUEUE_SIZE>,
        #[cfg(feature = "uart-bridge")]
        uart_tx_prod: heapless::spsc::Producer<'static, u8, UART_TX_QUEUE_SIZE>,
        #[cfg(feature = "uart-bridge")]
        uart_tx_cons: heapless::spsc::Consumer<'static, u8, UART_TX_QUEUE_SIZE>,
        // Current UART line coding, compared against the CDC's to detect a
        // host-requested baud/format change.
        #[cfg(feature = "uart-bridge")]
        uart_config: UartConfigAndClock,
        // --- UART bridge: UART0_IRQ side (RX FIFO -> queue) ---
        #[cfg(feature = "uart-bridge")]
        uart_rx_prod: heapless::spsc::Producer<'static, u8, UART_RX_QUEUE_SIZE>,
        // --- idle (GDB session loop) ---
        conn: QueueConn,
        target: ProbeTarget,
        rtt_tx_prod: heapless::spsc::Producer<'static, u8, RTT_TX_QUEUE_SIZE>,
        rtt_rx_cons: heapless::spsc::Consumer<'static, u8, RTT_RX_QUEUE_SIZE>,
    }

    #[init(local = [
        rx_queue: heapless::spsc::Queue<u8, RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        tx_queue: heapless::spsc::Queue<u8, TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        rtt_tx_queue: heapless::spsc::Queue<u8, RTT_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        rtt_rx_queue: heapless::spsc::Queue<u8, RTT_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        #[cfg(feature = "uart-bridge")]
        uart_rx_queue: heapless::spsc::Queue<u8, UART_RX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        #[cfg(feature = "uart-bridge")]
        uart_tx_queue: heapless::spsc::Queue<u8, UART_TX_QUEUE_SIZE> = heapless::spsc::Queue::new(),
        USB_ALLOCATOR: Option<UsbBusAllocator<hal::usb::UsbBus>> = None,
    ])]
    fn init(ctx: init::Context) -> (Shared, Local) {
        let mut resets = ctx.device.RESETS;
        let mut watchdog = hal::Watchdog::new(ctx.device.WATCHDOG);
        let sio = hal::Sio::new(ctx.device.SIO);
        let pins = hal::gpio::Pins::new(
            ctx.device.IO_BANK0,
            ctx.device.PADS_BANK0,
            sio.gpio_bank0,
            &mut resets,
        );

        let clocks = hal::clocks::init_clocks_and_plls(
            XOSC_CRYSTAL_FREQ,
            ctx.device.XOSC,
            ctx.device.CLOCKS,
            ctx.device.PLL_SYS,
            ctx.device.PLL_USB,
            &mut resets,
            &mut watchdog,
        )
        .ok()
        .unwrap();

        let usb_allocator =
            ctx.local
                .USB_ALLOCATOR
                .insert(UsbBusAllocator::new(hal::usb::UsbBus::new(
                    ctx.device.USBCTRL_REGS,
                    ctx.device.USBCTRL_DPRAM,
                    clocks.usb_clock,
                    true,
                    &mut resets,
                )));
        // Interface order fixes host tty numbering: first CDC = RSP,
        // second CDC = RTT terminal.
        let serial = SerialPort::new(usb_allocator);
        let rtt_serial = SerialPort::new(usb_allocator);
        // Third CDC (created before build() so it claims IF04): UART bridge.
        #[cfg(feature = "uart-bridge")]
        let uart_serial = SerialPort::new(usb_allocator);
        let usb_dev = UsbDeviceBuilder::new(usb_allocator, UsbVidPid(0x6666, 0x4444))
            .strings(&[
                usb_device::device::StringDescriptors::new(usb_device::LangID::EN_US)
                    .manufacturer("fugafuga.org")
                    .product("rust-dap GDB server")
                    .serial_number("raspberry-pi-pico-gdb"),
            ])
            .unwrap()
            // Two CDC-ACM functions -> IAD composite device.
            .device_class(USB_CLASS_MISCELLANEOUS)
            .device_sub_class(USB_SUBCLASS_COMMON)
            .device_protocol(USB_PROTOCOL_IAD)
            .build();

        // Bit-banging SWD transport + arm-debug.
        let swclk = PicoBidirPin::new(pins.gpio2.into_floating_input());
        let swdio = PicoBidirPin::new(pins.gpio3.into_floating_input());
        let reset = PicoBidirPin::new(pins.gpio4.into_floating_input());
        let swd = SwdIoSet::new(swclk, swdio, reset, CortexMDelay);
        let config = DapConfig::new(
            DapIdentity {
                serial_number: "raspberry-pi-pico-gdb",
                product_firmware_version: env!("GIT_REV"),
                ..DapIdentity::default()
            },
            clocks.system_clock.freq().to_Hz(),
        );
        let arm = ArmDebug::new(swd, config);

        // UART bridge: UART0 on GPIO0(TX)/GPIO1(RX). Starts at 115200 8N1 and
        // follows the host CDC line coding thereafter. RX is interrupt-driven
        // (UART0_IRQ); TX drains from a queue in USBCTRL_IRQ.
        #[cfg(feature = "uart-bridge")]
        let (uart_reader, uart_writer, uart_config) = {
            let uart_config = UartConfigAndClock {
                config: UartConfig::from(hal::uart::UartConfig::default()),
                clock: clocks.peripheral_clock.freq(),
            };
            let uart_pins = (
                pins.gpio0.into_function::<hal::gpio::FunctionUart>(),
                pins.gpio1.into_function::<hal::gpio::FunctionUart>(),
            );
            let mut uart = hal::uart::UartPeripheral::new(ctx.device.UART0, uart_pins, &mut resets)
                .enable((&uart_config.config).into(), uart_config.clock)
                .unwrap();
            uart.enable_rx_interrupt();
            let (reader, writer) = uart.split();
            (
                Some(UartReader(reader)),
                Some(UartWriter(writer)),
                uart_config,
            )
        };
        // Feature off: the shared resource still exists but stays empty/unused.
        #[cfg(not(feature = "uart-bridge"))]
        let uart_reader: Option<UartReader<pac::UART0, UartPins>> = None;

        let target = ProbeTarget::new(
            arm,
            CortexMDelay,
            2_000_000, // reset settle: ~16 ms @ 125 MHz
            // Reset site + count of the previous boot (survive sys_reset).
            unsafe { WATCHDOG_SCRATCH0.read_volatile() },
            unsafe { WATCHDOG_SCRATCH1.read_volatile() },
        );

        let (rx_prod, rx_cons) = ctx.local.rx_queue.split();
        let (tx_prod, tx_cons) = ctx.local.tx_queue.split();
        let (rtt_tx_prod, rtt_tx_cons) = ctx.local.rtt_tx_queue.split();
        let (rtt_rx_prod, rtt_rx_cons) = ctx.local.rtt_rx_queue.split();
        #[cfg(feature = "uart-bridge")]
        let (uart_rx_prod, uart_rx_cons) = ctx.local.uart_rx_queue.split();
        #[cfg(feature = "uart-bridge")]
        let (uart_tx_prod, uart_tx_cons) = ctx.local.uart_tx_queue.split();
        let conn = QueueConn {
            rx: rx_cons,
            tx: tx_prod,
            pushback: None,
        };

        (
            Shared { uart_reader },
            Local {
                usb_dev,
                serial,
                rtt_serial,
                rx_prod,
                tx_cons,
                rtt_tx_cons,
                rtt_rx_prod,
                #[cfg(feature = "uart-bridge")]
                uart_serial,
                #[cfg(feature = "uart-bridge")]
                uart_writer,
                #[cfg(feature = "uart-bridge")]
                uart_rx_cons,
                #[cfg(feature = "uart-bridge")]
                uart_tx_prod,
                #[cfg(feature = "uart-bridge")]
                uart_tx_cons,
                #[cfg(feature = "uart-bridge")]
                uart_config,
                #[cfg(feature = "uart-bridge")]
                uart_rx_prod,
                conn,
                target,
                rtt_tx_prod,
                rtt_rx_cons,
            },
        )
    }

    /// Service USB at interrupt priority: enumeration and CDC transfers stay
    /// responsive while idle blocks in long SWD operations.
    #[task(binds = USBCTRL_IRQ, priority = 2,
        shared = [#[cfg(feature = "uart-bridge")] uart_reader],
        local = [usb_dev, serial, rtt_serial, rx_prod, tx_cons, rtt_tx_cons, rtt_rx_prod,
        #[cfg(feature = "uart-bridge")] uart_serial,
        #[cfg(feature = "uart-bridge")] uart_writer,
        #[cfg(feature = "uart-bridge")] uart_rx_cons,
        #[cfg(feature = "uart-bridge")] uart_tx_prod,
        #[cfg(feature = "uart-bridge")] uart_tx_cons,
        #[cfg(feature = "uart-bridge")] uart_config])]
    // `mut` is only exercised by the uart-bridge line-coding lock below.
    #[cfg_attr(not(feature = "uart-bridge"), allow(unused_mut))]
    fn usb_irq(mut ctx: usb_irq::Context) {
        let usb_dev = ctx.local.usb_dev;
        let serial = ctx.local.serial;
        let rtt_serial = ctx.local.rtt_serial;
        #[cfg(feature = "uart-bridge")]
        let uart_serial = ctx.local.uart_serial;
        #[cfg(not(feature = "uart-bridge"))]
        usb_dev.poll(&mut [serial, rtt_serial]);
        #[cfg(feature = "uart-bridge")]
        usb_dev.poll(&mut [serial, rtt_serial, uart_serial]);
        // 1200 bps touch → reboot into the bootloader (reflash without BOOTSEL).
        rust_dap_rp::util::bootsel_on_1200bps_touch(serial);
        USB_CONFIGURED.store(
            usb_dev.state() == UsbDeviceState::Configured,
            core::sync::atomic::Ordering::Relaxed,
        );
        // RX: CDC → queue (drop on overflow; RSP retransmits via its acks).
        let mut buf = [0u8; 64];
        while let Ok(n) = serial.read(&mut buf) {
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                let _ = ctx.local.rx_prod.enqueue(b);
            }
        }
        // TX: queue → CDC. Unlike the old polled loop there is no "next
        // iteration" to push the write buffer out, so flush explicitly —
        // without it a partial packet sits in usbd-serial's buffer forever
        // (no endpoint armed → no further IRQ → deadlock).
        while let Some(&b) = ctx.local.tx_cons.peek() {
            match serial.write(&[b]) {
                Ok(1) => {
                    ctx.local.tx_cons.dequeue();
                }
                _ => break,
            }
        }
        let _ = serial.flush();
        // RTT CDC: drain the up-stream queue to the host, collect host input.
        while let Some(&b) = ctx.local.rtt_tx_cons.peek() {
            match rtt_serial.write(&[b]) {
                Ok(1) => {
                    ctx.local.rtt_tx_cons.dequeue();
                }
                _ => break,
            }
        }
        let _ = rtt_serial.flush();
        while let Ok(n) = rtt_serial.read(&mut buf) {
            if n == 0 {
                break;
            }
            for &b in &buf[..n] {
                let _ = ctx.local.rtt_rx_prod.enqueue(b);
            }
        }
        // UART bridge (third CDC): target UART RX queue -> host, host -> UART TX.
        // The bridge helpers honour queue readiness, so a burst larger than the
        // UART can drain back-pressures the host CDC (NAK) instead of dropping.
        #[cfg(feature = "uart-bridge")]
        {
            use rust_dap_rp::bridge;
            let uart_writer = ctx.local.uart_writer;
            let uart_config = ctx.local.uart_config;
            bridge::drain_uart_rx_queue(uart_serial, ctx.local.uart_rx_cons);
            bridge::drain_usb_to_uart_tx(uart_serial, ctx.local.uart_tx_prod);
            bridge::drain_uart_tx_queue(uart_writer, ctx.local.uart_tx_cons);
            // Follow a host-requested line coding change (baud / parity / etc.).
            if let Ok(expected) = UartConfig::try_from(uart_serial.line_coding()) {
                if expected != uart_config.config {
                    ctx.shared.uart_reader.lock(|reader| {
                        bridge::reconfigure_uart(reader, uart_writer, uart_config, &expected);
                    });
                }
            }
        }
    }

    /// UART bridge RX: drain the UART0 FIFO into the RX queue, then pend
    /// USBCTRL_IRQ so it flushes the queue to the host CDC promptly (nothing
    /// else would trigger that task between host writes). On queue overflow
    /// bytes are dropped and the RX interrupt stays enabled so flow resumes.
    #[cfg(feature = "uart-bridge")]
    #[task(binds = UART0_IRQ, priority = 3, shared = [uart_reader], local = [uart_rx_prod])]
    fn uart_irq(mut ctx: uart_irq::Context) {
        use embedded_hal_nb::serial::Read;
        let rx_prod = ctx.local.uart_rx_prod;
        let received = ctx.shared.uart_reader.lock(|reader| {
            let reader = reader.as_mut().unwrap();
            let mut received = false;
            while let Ok(b) = reader.0.read() {
                let _ = rx_prod.enqueue(b);
                received = true;
            }
            received
        });
        if received {
            pac::NVIC::pend(pac::Interrupt::USBCTRL_IRQ);
        }
    }

    #[idle(local = [conn, target, rtt_tx_prod, rtt_rx_cons])]
    fn idle(ctx: idle::Context) -> ! {
        let conn = ctx.local.conn;
        let target = ctx.local.target;
        let rtt_tx = ctx.local.rtt_tx_prod;
        let rtt_rx = ctx.local.rtt_rx_cons;
        // Decimate RTT polling: only on quiet iterations (no RSP byte), and
        // only every N of those, so RSP throughput is unaffected.
        let mut rtt_tick: u32 = 0;
        boot_progress(1); // idle entered

        // Wait for USB enumeration before touching the target, then connect
        // + halt so GDB attaches to stopped cores.
        while !USB_CONFIGURED.load(core::sync::atomic::Ordering::Relaxed) {}
        boot_progress(2); // USB configured
        target.connect_and_halt();
        // Discard anything a fast-attaching GDB piled up (retransmitted
        // qSupported) while the initial connect ran, so the first session's
        // stub answers a single clean packet.
        conn.purge();
        boot_progress(3); // SWD connected

        // One iteration per GDB session: the gdbstub state machine holds
        // per-session protocol state (e.g. negotiated no-ack mode), so it
        // must be rebuilt from scratch after every disconnect — reusing it
        // makes the next session's opening ack error the stub.
        let mut packet_buffer = [0u8; 1024];
        loop {
            let gdb = match GdbStubBuilder::new(ConnRef(conn))
                .with_packet_buffer(&mut packet_buffer)
                .build()
            {
                Ok(g) => g,
                Err(_) => reset_self(1),
            };
            let mut sm = gdb
                .run_state_machine(target)
                .unwrap_or_else(|_| reset_self(2));
            boot_progress(4); // session loop live

            // Drive this session until GDB disconnects. A gdbstub error
            // (e.g. a new GDB attaching while the previous session's state
            // machine is still Running) ends the session the same way — the
            // outer loop rebuilds a fresh stub — instead of rebooting.
            loop {
                let next = match sm {
                    GdbStubStateMachine::Idle(mut inner) => {
                        match inner.borrow_conn().0.read_byte() {
                            Some(b) => inner.incoming_data(target, b).ok(),
                            None => {
                                rtt_tick = rtt_tick.wrapping_add(1);
                                if rtt_tick % 64 == 0 && target.rtt_stream(rtt_tx, rtt_rx) {
                                    QueueConn::kick();
                                }
                                Some(GdbStubStateMachine::Idle(inner))
                            }
                        }
                    }
                    GdbStubStateMachine::Running(mut inner) => {
                        if let Some(b) = inner.borrow_conn().0.read_byte() {
                            // Typically a Ctrl-C (0x03) to interrupt.
                            inner.incoming_data(target, b).ok()
                        } else if let Some(reason) = target.poll_stopped() {
                            // A core stopped on its own (breakpoint /
                            // watchpoint / step done); the others were
                            // halted with it.
                            inner.report_stop(target, reason).ok()
                        } else {
                            rtt_tick = rtt_tick.wrapping_add(1);
                            if rtt_tick % 4 == 0 && target.rtt_stream(rtt_tx, rtt_rx) {
                                QueueConn::kick();
                            }
                            Some(GdbStubStateMachine::Running(inner))
                        }
                    }
                    GdbStubStateMachine::CtrlCInterrupt(inner) => {
                        target.halt_running(None);
                        inner
                            .interrupt_handled(
                                target,
                                Some(MultiThreadStopReason::SignalWithThread {
                                    tid: core_tid(0),
                                    signal: Signal::SIGINT,
                                }),
                            )
                            .ok()
                    }
                    GdbStubStateMachine::Disconnected(_) => None,
                };
                match next {
                    Some(s) => sm = s,
                    None => break,
                }
            }
            target.note_session_end();
            // GDB detached: the state machine (and its borrow of conn) is
            // dropped. Drop the ended session's stale RX, re-establish the
            // SWD link + halt (may be slow — it can pulse SRST to recover a
            // wedged target), then purge AGAIN: a new GDB that attached
            // during the slow reconnect will have retransmitted its opening
            // qSupported several times, and processing those stale copies
            // desyncs the RSP framing. Discarding them lets the fresh stub
            // answer GDB's next (clean) retransmit exactly once.
            conn.purge();
            target.connect_and_halt();
            conn.purge();
        }
    }
}

// Watchdog scratch registers survive SYSRESETREQ; used to carry the reset
// site + count across reset_self for the diagnostic window.
const WATCHDOG_SCRATCH0: *mut u32 = 0x4005_800c as *mut u32;
const WATCHDOG_SCRATCH1: *mut u32 = 0x4005_8010 as *mut u32;

/// Boot-progress marker at the top of SRAM: survives a 1200bps-touch reboot,
/// so `picotool save -r 0x20041f00 0x20041f08` can show how far the firmware
/// got even when RSP is dead. Written as 0xb007_00XX stage codes.
const BOOT_PROGRESS: *mut u32 = 0x2004_1f00 as *mut u32;

fn boot_progress(stage: u32) {
    unsafe { BOOT_PROGRESS.write_volatile(0xb007_0000 | stage) };
}

/// Record panics like reset sites (0xfa) instead of hanging silently with
/// interrupts still enabled (which keeps USB alive but the stub dead).
#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    reset_self(0xfa)
}

/// Last-resort recovery: a gdbstub protocol/state error leaves the stub
/// unusable, so reboot the whole firmware instead of going dead (the old
/// `loop_forever` stopped USB polling, wedging the port until replug).
/// `site` identifies the caller in the diagnostic window (diag[8]).
fn reset_self(site: u32) -> ! {
    unsafe {
        WATCHDOG_SCRATCH0.write_volatile(0x5e1f_0000 | site);
        WATCHDOG_SCRATCH1.write_volatile(WATCHDOG_SCRATCH1.read_volatile().wrapping_add(1));
    }
    // Detach from USB cleanly first — rebooting mid-enumeration can wedge
    // the host's hub port (see util::usb_detach_for_reset).
    rust_dap_rp::util::usb_detach_for_reset();
    cortex_m::peripheral::SCB::sys_reset();
}
