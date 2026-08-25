# Guidance for coding agents

## Keep the READMEs in sync with the code

Documentation is bilingual: every README has an English `README.md` and a
Japanese `README.ja.md` with the same content. When you change one, change
the other.

Update the top-level `README.md` / `README.ja.md` in the same PR when you:

- add or remove a board (`boards/*`), or change what a board supports
  (CMSIS-DAP / SWD / JTAG / UART / GDB server / RTT) — the board table
- change the USB identity (VID:PID, manufacturer/product strings, serial
  number format) — the "USB identification" section
- change how firmware is obtained or built (release artifacts, board
  directories, `cargo build` invocation) — the "Getting the firmware" section

Update the board's `boards/<board>/README.md` / `README.ja.md` when you:

- add, rename or remove a Cargo feature or a binary (`[[bin]]`) of that board
- change pin assignments or default clock settings
- change how the host connects (CDC port layout, GDB `target` command,
  `monitor` commands)

Keep the top-level README short: one table row or one sentence per item.
Details belong in the per-board READMEs.
