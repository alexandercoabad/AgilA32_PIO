# Feature 18 -- post-layout (gate-level) PIO smoke test

A short cocotb test that exercises the PIO through the real pads, written to run in the Tiny Tapeout `gl_test` job
(and in the normal RTL job).

## What it does
`test/test_pio_gl.py::test_pio_postlayout_echo_and_pulses`:
1. Boots like the other on-chip tests: boot ROM self-test, GPIO bootloader, 4-byte hand-off stub, then runs a flash image
   (`test/pio_gl_smoke_flash_image.hex`, 22 pages) built by `tools/build_pio_gl_smoke.py`.
2. Two PIO programs share the instruction memory:
   - `pio/gl_echo.pio` (SM0): `pull`, `mov x, ~osr`, `mov isr, ::x` (invert + bit-reverse), `push`, `irq set 0`.
   - `pio/gl_pulses.pio` (SM1): `wait 1 irq 0`, then 4 pulses on pad uo_out[7] (`set pins` with delays, `jmp x--`).
3. The CPU feeds three words (0x55, 0x01, 0xF0), reads each echo from the RX FIFO and shows it on the pads (uo_out[6:0]).
4. The test (a python flash + PSRAM model on the QSPI pins, as in `test.py`) checks:
   the three echoed values in order; exactly 12 pulses, each 4 clocks high with a 10-clock period inside each burst of 4;
   pad low at the end; `uio_oe` as expected.
Covered in the placed netlist: FIFOs, shifter, ISA decode (pull, mov with invert/reverse, push, irq, wait, set, jmp x--),
IRQ flags, clock divider (CLKDIV 1 and 2), pin muxing (`PIN_OWN`), bus handshake.

## Gate-level notes
- `test/Makefile`: `COCOTB_TEST_MODULES = test,test_pio_gl`, so the existing `gl_test` job picks it up with no workflow change.
- In GL the QSPI runs at the slow default (`speed_up_qspi` is a no-op), so the test waits generously (15000 clocks before the
  bootloader, then polls up to 20 M clocks). The gate-level run is much slower than the RTL one (see "Result" below).

## Result (real gate-level run)
The test has run in the Tiny Tapeout `gl_test` job on the real IHP post-layout netlist (PDK cells): **12/12 cocotb tests
passing** (the 11 existing ones plus `test_pio_postlayout_echo_and_pulses`), 0 failures, **2,587 s** wall-clock time
(the `time` of that test in `results.xml`; the same test takes about 10 s on RTL).
Netlist: the layout run this was executed against is the 24 MHz run whose extracted SPICE (`tt_um_agila32.spice`, ihp-sg13g2)
has 3,433 flip-flops, the same count as that run's CI summary.
- CI time: this single test makes `gl_test` take about 50 minutes instead of about 6. That is still well inside GitHub's
  default limit for a hosted job (360 minutes), so no workflow change or `timeout-minutes` is needed.

## Mutation check
`tools/gl_smoke_mutation_sweep.py`: 16 one-line breaks of the two programs and of `src/pio_sm.v` (no invert, no bit-reverse,
no push, no IRQ, wrong IRQ number, 3 pulses, wrong widths, pad left high, no wait, `jmp x--` not decrementing or inverted,
`wait` not clearing the flag, `irq set` doing nothing). Result: **all 16 caught**, each within seconds (the test's polling budget can be lowered with `GL_SMOKE_MAX_CLOCKS`, default 20 M clocks; the sweep uses 150 k, the RTL run needs about 32 k).

## Limits
Developed and mutation-checked here on RTL and on a Yosys-flattened generic-gate netlist (the real IHP post-layout netlist
with PDK cells is not available outside the CI). The first real gate-level run, in GitHub Actions, passed 12/12 in 2,587 s
(see "Result"). Not covered: the other PIO tests (protocol decoders, I2C slave, 1-Wire, VGA, WS2812 ...) have still only been
simulated on RTL; only this smoke test runs on the post-layout netlist.

### What the smoke test touches, and what it does not (derived from `pio/gl_echo.pio`, `pio/gl_pulses.pio`, `tools/build_pio_gl_smoke.py`, `test/test_pio_gl.py`)
- **Exercised post-layout:** `pull block`, `mov x, ~osr`, `mov isr, ::x`, `push block`, `irq set 0`, `wait 1 irq 0`, `set x`, `set pins` with
  delays, `jmp x--`; two state machines (CLKDIV 1 and 2) with the default SHIFTCTRL; `PIN_OWN` on pin 7 only; the CPU-to-PIO bus handshake
  and both FIFO directions.
- **Not exercised post-layout:** `in`; every `out` form; `set pindirs`; side-set (and side-set on pindirs); every `jmp` condition except `x--`;
  `wait` on pins or GPIO; `out exec` / `mov exec`; `mov` from pins or status and `mov pc`; autopush and autopull with their thresholds and
  left-shift direction; the `noblock` / `iffull` / `ifempty` variants; `irq wait`, `irq clear`, relative IRQ numbers; the input synchronisers and
  their bypass; and the tri-state pads: pads 8 and 9 (`uio[4]`, `uio[5]`) are never handed to the PIO, and the test asserts that `uio_oe` never
  changes, so that pad path is untouched.
- **Why this matters for I2C:** the I2C master, multi-master and slave programs rely on exactly these (`out`/`set`/side-set on pindirs, `wait pin`,
  `jmp pin`, `in pins`, `out exec`, autopull/autopush, `irq wait rel`) and drive the open-drain pads 8/9. None of that logic has post-layout
  evidence yet; an I2C gate-level smoke test would have to own pads 8/9 and exercise them from the CPU.
