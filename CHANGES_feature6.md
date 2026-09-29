# Feature #6: PIO block (firmware-defined protocol emulation)

Adds two RP2040-compatible PIO state machines so UART/SPI/(I2C, JTAG, ...)
are *programs* loaded after fabrication instead of fixed logic.

## RTL
- `src/pio.v`, `src/pio_sm.v`, `src/pio_fifo.v` (new).
- `src/tt_um_agila32.v`: PIO instantiated on the CPU bus; `0xFE/0xFF`
  answered by PIO, everything else by `mem.v`. `uo_out[7:0]` follow PIO when
  the matching `PIN_OWN` bit is set; `uio[4]`/`uio[5]` (previously tied high)
  become PIO pins 8/9 with PINDIR as output-enable when owned. With
  `PIN_OWN = 0` (reset) all pins and `uio_oe = 8'b1111_1011` are identical to
  before, so all earlier tests pass unchanged.

## Tools
- `tools/pioasm.py`: added `origin=` / `--origin` (relocates jump targets,
  wrap); `Program.origin` (required by `pio_host.py`).
- `tools/pio_host.py`, `tools/build_pio_uart.py`, `pio/*.pio` (new).

## Tests
- `tb_pio_isa.v`, `tb_pio_uart.v`, `tb_pio_spi.v`, `tb_pio_cpu_uart.v` in
  `make standalone-tests`. Fixes to the supplied `tb_pio_isa.v`: three WAIT
  encodings had the wrong source field (0x20E5/0x20C6/0x2001 ->
  0x20C5/0x2086/0x2021), and OUT/MOV EXEC cases now briefly enable the SM so
  the pending instruction can run.

## Config
- `info.yaml`: source files, pinout, `tiles: "6x4"` (contest size).

## Synthesis fix: Yosys `share` pass stalled in CI
The LibreLane/Yosys log sat in "Analyzing resource sharing ... activation_patterns"
(the SAT-based `share` pass) on the variable-amount `<<` / `>>` operators in
`pio_sm.v` (pin-mask rotates, OUT/IN shifts). `pio_sm.v` now has **no** variable
`<<`/`>>`: rotates, masks and 32-bit shifts are fixed-stage mux shifters
(`rotl16`, `rotr16`, `cmask*`, `shr5`, `shr32`, `shl32`). Behaviour is unchanged:
all PIO testbenches (ISA, UART, SPI, end-to-end CPU) and the 11 cocotb tests pass.
Locally the `share` pass on the full design dropped from 145 to 5 analyses
(28 s -> 6 s). Only the CPU's own three ALU shifters remain (as in the
pre-PIO design). Rough generic-gate count moved 34.2k -> 35.0k (+2%): this fix
is for synthesis time, not area.
