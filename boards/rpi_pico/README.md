# Raspberry Pi Pico port

English [日本語](./README.ja.md)

## Pin assignments

![Pin assignments](./rust-dap-pico.svg)

| Pin Number | Pin Name | SWD pin | JTAG pin |
|:--------|:-------|:-------------|:--------------|
| 3       | GND    | GND          | GND           |
| 4       | GPIO2  | SWCLK        | TCK           |
| 5       | GPIO3  | SWDIO        | TMS           |
| 6       | GPIO4  | RESET        | nSRST         |
| 7       | GPIO5  |              | TDO           |
| 9       | GPIO6  |              | TDI           |
| 10      | GPIO7  |              | nTRST         |

## How to build

### For SWD

By default, this project is configured to use `features` to build the firmware for SWD with PIO.

```
cargo build --release
```

When the feature `bitbang` is enabled, the firmware performs SWD communication by controlling GPIO from CPU instead of PIO.

```
cargo build --release --features bitbang
```

When the feature `set_clock` is enabled, the firmware supports setting clock rate from the host. If the firmware uses PIO, the accuracy of the clock rate is fairly accurate. If the firmware uses bitbang, the accuracy is not good.

```
cargo build --release --features set_clock # PIO
cargo build --release --features set_clock,bitbang # bitbang
```

### For JTAG

In order to build firmware for JTAG, disable the default SWD features by specifying `--no-default-features` option and then enable the `jtag` feature.

```
cargo build --release --no-default-features --features jtag
```

Enable the feature `set_clock` to enable setting clock rate from the host PC.

```
cargo build --release --no-default-features --features jtag,set_clock
```

## How to use

### Use with pyOCD

#### Install pyOCD

```
python3 -m pip install pyocd
```

#### Run pyOCD and GDB connection

```sh
pyocd gdbserver --target rp2040_core0
```

Run GDB in another terminal.

`gdb-multiarch`, which is installed with `apt`, seems to fail to recognize the architecture correctly, so [download the toolchain from Arm's website](https://developer.arm.com/downloads/-/gnu-rm)
 and use GDB in it.

```sh
arm-none-eabi-gdb <target elf file> -ex "target extended-remote localhost:3333"
```
