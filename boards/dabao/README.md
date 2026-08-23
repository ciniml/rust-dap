# Dabao Board (Baochip-1x) port — standalone GDB server

English [日本語](./README.ja.md)

The [Dabao board](https://github.com/betrusted-io/xous-core) (Baochip-1x,
RISC-V, running the Xous OS) becomes an SWD debug probe that speaks the GDB
Remote Serial Protocol directly over its USB CDC-ACM port — GDB attaches to
it without OpenOCD or probe-rs in between.

Unlike the other boards in this repository, the firmware is **not a crate in
this tree**: it is a Xous application, `apps-dabao/dabao-gdb`, that lives in a
fork of xous-core and consumes this repository's crates (`rust-dap`,
`arm-debug`, `gdb-server-core`) through relative path dependencies. This
directory holds the documentation, a build script, and the CI glue.

| Piece | Location |
|---|---|
| Protocol / target layer (board independent) | [`gdb-server-core`](../../gdb-server-core), [`arm-debug`](../../arm-debug), [`rust-dap`](../../rust-dap) (this repo) |
| Xous application (SWD on a BIO core, USB session loop) | [ciniml/xous-core](https://github.com/ciniml/xous-core) branch `dabao-gdb-server`, `apps-dabao/dabao-gdb` |
| Build script / CI | [`build.sh`](./build.sh), [`.github/workflows/dabao.yml`](../../.github/workflows/dabao.yml), [`release.yml`](../../.github/workflows/release.yml) |

## Features

- GDB RSP over USB CDC-ACM (VID:PID `1d50:6197`, product `Dabao`): registers,
  memory, hardware breakpoints/watchpoints, single step, `monitor reset`,
  multi-core thread view, `load` to RP2040 flash (resident async loader),
  nRF52/nRF54L flash + `monitor approtect` / `erase_all`.
- Target families: RP2040, nRF52, nRF54L (auto-detected by DPIDR).
- SWD runs on one of the Baochip BIO cores at 8 MHz (measured: RAM read
  ~225 KiB/s, RAM write ~207 KiB/s, RP2040 `load` ~58 KB/s).
- RTT: `monitor rtt scan | attach <addr> | dump`; while the target runs, RTT
  up-channel text is forwarded into the GDB console (`O` packets), so it
  shows up in the GDB window in real time (~80 KiB/s). There is no separate
  RTT CDC port on this board (the Baochip USB core has only 4 IN + 4 OUT
  endpoints, all taken by HID + the RSP port), and no down channel.
- `monitor diag` prints link statistics (SWD ACK histogram, RSP queue
  high-water marks).

## Wiring

| Dabao pin | SWD signal |
|---|---|
| PB1 | SWCLK |
| PB2 | SWDIO |
| PB3 | nRESET (target RUN / reset) |
| GND | GND |

PB1–PB5 are on the Dabao header and don't collide with the bootloader's
reservations (PB13/PB14 = console UART, PC13 = USB SE0 / `PROG`). The pins
are constants at the top of `apps-dabao/dabao-gdb/src/main.rs`.

Because the RSP owns the board's only USB serial port, the Xous log console
stays on the physical UART: **PB14 = TX, PB13 = RX, 1 Mbaud** (e.g.
`screen /dev/ttyUSB0 1000000` through an FT232H).

## Building

Requirements: a stable Rust toolchain for which a Xous toolkit release exists
(the build script checks `rustc --version`; 1.97.x and 1.98.0 are known
good), `git`, and network access on the first run (the Xous `xtask`
downloads the `riscv32imac-unknown-xous-elf` standard library).

```shell
# from the rust-dap checkout
./boards/dabao/build.sh            # clones ../xous-core (ciniml, dabao-gdb-server) if missing
./boards/dabao/build.sh --flash    # ... then flash via apps-dabao/dabao-gdb/flash.py (needs FT232H)
```

What it does:

1. Makes sure a xous-core checkout exists **next to** this repository
   (`../xous-core`; override with `XOUS_CORE=/path` or `--xous-core PATH`,
   branch with `XOUS_BRANCH`). The relative path dependencies in
   `apps-dabao/dabao-gdb/Cargo.toml` point at `../../../rust-dap/...`, so the
   two checkouts must be siblings, or you must pass `--rust-dap PATH` and a
   symlink is created.
2. Installs the Xous toolkit if the sysroot lacks the target
   (`cargo xtask install-toolkit --force --no-verify`).
3. Runs `cargo xtask dabao dabao-gdb` in xous-core.
4. Copies `loader.uf2`, `xous.uf2`, `apps.uf2` and the `dabao-gdb` ELF into
   `boards/dabao/dist/` (override with `--out DIR`).

Manual equivalent:

```shell
git clone -b dabao-gdb-server https://github.com/ciniml/xous-core ../xous-core
cd ../xous-core
cargo xtask install-toolkit --force --no-verify     # once per rustc version
cargo xtask dabao dabao-gdb
ls target/riscv32imac-unknown-xous-elf/release/{loader,xous,apps}.uf2
```

## Flashing

All three UF2 images belong together (the fork also patches the USB service
in `xous.uf2`), so flash `loader.uf2`, `xous.uf2`, `apps.uf2` — in that order
when done by hand.

- **By hand**: hold `PROG` while plugging in USB; the `BAOCHIP` volume
  appears; copy the UF2 files; press `PROG` again (or the bootloader's `boot`
  command) to start.
- **Scripted** (`build.sh --flash`, or `apps-dabao/dabao-gdb/flash.py` in
  xous-core): pulses the board's reset line through an FT232H (RTS# / ADBUS2
  wired to the reset button, common GND), waits for the boot1 bootloader's
  USB console (`Baochip-1x`, `1d50:6196`), streams the UF2 over its `uf2`
  command and issues `boot`. Requires `bootwait` to have been enabled once
  in boot1 (enter it with `PROG`, type `bootwait`) so that a plain reset
  parks the board in the bootloader. Python 3 + pyserial only.

## Using

```shell
gdb-multiarch -ex 'set remotetimeout 10' \
              -ex 'target remote /dev/serial/by-id/usb-Baochip_Dabao_*-if02' \
              -ex 'set architecture armv4t' -ex 'info registers'
```

- The probe connects to and halts the target before the first session, so
  GDB attaches to a stopped core. `monitor help` lists the monitor commands.
- Stop ModemManager (or add a udev rule with `ID_MM_DEVICE_IGNORE=1` for
  `1d50:6197`), otherwise it grabs the CDC port right after enumeration.
- `monitor reset` leaves GDB's register cache stale; run
  `maintenance flush register-cache` (or just `stepi`) afterwards.
- RTT: `monitor rtt scan` → `monitor rtt attach <addr>` → `continue`; the
  target's output appears in the GDB console while it runs; `monitor rtt
  dump` drains what accumulated while halted.

## Known limitations

- Single USB CDC port: no RTT terminal port and no UART bridge (the
  endpoint budget is exhausted — see Features). Down-channel RTT is not
  supported.
- RP2350, SWJ/JTAG and the CMSIS-DAP protocol are not implemented for this
  board; it is a GDB server only.
- The Xous-side code is maintained in the `ciniml/xous-core` fork, not
  upstream betrusted-io (the fork carries USB-stack fixes the RSP needs).
