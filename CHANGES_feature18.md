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
  bootloader, then polls up to 20 M clocks). Expect the gate-level run to be much slower than the RTL one.

## Mutation check
`tools/gl_smoke_mutation_sweep.py`: 16 one-line breaks of the two programs and of `src/pio_sm.v` (no invert, no bit-reverse,
no push, no IRQ, wrong IRQ number, 3 pulses, wrong widths, pad left high, no wait, `jmp x--` not decrementing or inverted,
`wait` not clearing the flag, `irq set` doing nothing). Result: **all 16 caught**, each within seconds (the test's polling budget can be lowered with `GL_SMOKE_MAX_CLOCKS`, default 20 M clocks; the sweep uses 150 k, the RTL run needs about 32 k).

## Limits
Verified here on RTL and on a Yosys-flattened generic-gate netlist (not the real IHP post-layout netlist with PDK cells,
which is not available outside the CI). The first real GL run is the one in your GitHub Actions.
