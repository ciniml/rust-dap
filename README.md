# A CMSIS-DAP implementation in Rust

English [日本語](./README.ja.md)

## About this project

![Debug Board](./doc/figure/debug_board.drawio.svg)

This is a Rust implementation of CMSIS-DAP, which is a protocol and firmware standard of debug adapters for Arm processors.

It returns the correct WCID (Windows Compatibility ID), so it can be used on Windows without manual driver installation.
Some boards can also run as a standalone GDB server: GDB connects to the probe directly over USB-CDC, without OpenOCD, pyOCD or probe-rs on the host.

## Supported boards

The following boards are currently supported.
The implementation and the build/usage instructions for each board are placed under the [boards](./boards) directory.

| Board name           | Supported features | Directory     |
|:------------------|:----------------|:--------------------|
| Seeeduino XIAO    | CMSIS-DAP       | [./boards/xiao_m0](./boards/xiao_m0) | 
| XIAO RP2040       | CMSIS-DAP, UART | [./boards/xiao_rp2040](./boards/xiao_rp2040) | 
| Raspberry Pi Pico | CMSIS-DAP (SWD/JTAG), UART, GDB server (SWD / 2-pin cJTAG), RTT | [./boards/rpi_pico](./boards/rpi_pico) | 
| Raspberry Pi Pico 2 | CMSIS-DAP (SWD/JTAG), UART | [./boards/rpi_pico2](./boards/rpi_pico2) | 
| Dabao Board (Baochip-1x) | GDB server (RSP over USB), RTT | [./boards/dabao](./boards/dabao) |

## Getting the firmware

Prebuilt firmware (UF2 / ELF) for each board is attached to [GitHub Releases](https://github.com/ciniml/rust-dap/releases).
To build from source, run `cargo build --release` in the board directory; see the README of each board for the available features.

## USB identification

The firmware enumerates as VID:PID `6666:4444`, manufacturer `fugafuga.org`.

The USB serial number (and `DAP_Info` serial number) is derived from the board-unique ID, formatted as 16 uppercase hex digits in the same way as pico-sdk / debugprobe (RP2040: QSPI flash unique ID, RP2350: OTP chip ID), so multiple probes can be told apart, e.g. `probe-rs --probe 6666:4444:<SERIAL>`.
Earlier releases used a fixed string such as `raspberry-pi-pico-swd`; update udev rules or probe selectors that relied on it.

## License

Distributed under `Apache-2.0 License` 

Check the [LICENSE](./LICENSE) file for details.
