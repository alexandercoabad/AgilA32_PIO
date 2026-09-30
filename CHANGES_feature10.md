# Feature #10: PS/2 receiver on the PIO (`pio/ps2_rx.pio`) + an honest Step 3

No RTL change. Answers the Step 3 question "how do keyboard polling and display draws share a core
with no interrupts?" -- they don't; the keyboard goes into the PIO.

## Why the polled reader is not enough
`tools/build_ps2_reader.py` / `build_ps2_ascii.py` wait for every CLOCK edge in its own flash page.
One page switch costs about 3000 clocks (docs/info.md; ~165k firmware clocks / 55 pages in
`tb_pio_cpu_spi4.v`). A real keyboard's CLOCK edges are 30-50 us apart: 30-50 core clocks at the 1 MHz
in `info.yaml`, 1500-2500 at 50 MHz. `tb_ps2_reader.v` holds each level for 4000 clocks, so the reader was
only ever tested against a keyboard about 100x slower than a real one (at 1 MHz). README and docs now say so.

## `pio/ps2_rx.pio` (8 instructions, position independent)
Samples DATA on CLOCK's falling edge (CLOCK pin 3, DATA pin 4, `IN_BASE = 4`, `JMP_PIN = 3`, autopush 11,
shift right). One RX word per frame, frame in bits [31:21]. An idle-gap timeout (288 PIO cycles) clears a
partial frame with `mov isr, null`. The 4-deep RX FIFO holds frames while the CPU is busy.

## Tests (`test/test_pio_protocols.py`, +7 -> 26; model `Ps2Keyboard` + `decode_ps2_word` in `pio_tb_lib.py`)
keystrokes incl. extended keys | 10 and 16.7 kHz at 50 MHz | 4-frame burst held in the FIFO | overflow keeps
the oldest 4 and recovers | resync after a partial frame | bad parity / low stop bit reported to the CPU |
SM1 receives PS/2 while SM0 streams 96 SPI bytes (all 1536 SCK edges inside PS/2 frames).
Mutation-checked: no `mov isr, null`, or a timeout that never expires -> the partial-frame test fails;
`set x, 3` (36-cycle timeout) -> all five fast PS/2 tests fail.

## Two things that went wrong on the way
- First version used `jmp 0`: an absolute target, so loading the program at origin 8 next to `spi_master`
  would have jumped into the SPI program. Now a label; verified at origin 8 by the concurrency test.
- A cocotb `COCOTB_TEST_FILTER` with `(a|b)` is expanded by `make`'s shell and runs nothing (silently
  printing no results); use one exact test name per run.

## Not done
CPU-driven top-level demo (flash image + `tt_um_agila32` testbench); host-to-device commands and keyboard
inhibit (need open-drain CLOCK/DATA on `uio[5:4]`); the PIO glyph expander, font, terminal loop; anything on
real hardware. The keyboard model is my reading of the PS/2 timing, not a measured device.
