# A CMSIS-DAP implementation in Rust

English [日本語](./README.ja.md)

## About this project

![Debug Board](./doc/figure/debug_board.drawio.svg)

This is a Rust implementation of CMSIS-DAP, which is a protocol and firmware standard of debug adapters for Arm processors.

At this time, it has no performance or feature advantage over other implementations as a debug adapter , but by returning the correct WCID (Windows Compatibility ID), it can be used on Windows without manual driver installation.

## Supported boards

The following boards are currently supported.
The implementation for each board is placed under the [boards](./boards) directory.

| Bord name           | Supported features | Directory     |
|:------------------|:----------------|:--------------------|
| Seeeduino XIAO    | CMSIS-DAP       | [./boards/xiao_m0](./boards/xiao_m0) | 
| XIAO RP2040       | CMSIS-DAP, UART | [./boards/xiao_rp2040](./boards/xiao_rp2040) | 
| Raspberry Pi Pico | CMSIS-DAP, UART | [./boards/rpi_pico](./boards/rpi_pico) | 

## License

Distributed under `Apache-2.0 License` 

Check the [LICENSE](./LICENSE) file for details.