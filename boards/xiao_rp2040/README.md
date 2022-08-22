# XIAO RP2040 port

English [日本語](./README.ja.md)

## Pin assignments

| Pin Number | Pin Name | SWD pin |
|:--------|:-------|:-------------|
| GND     | GND    | GND          |
| 8       | GPIO2  | SWCLK        |
| 9       | GPIO4  | SWDIO        |
| 0       | GPIO26 | RESET        |

## How to build

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
