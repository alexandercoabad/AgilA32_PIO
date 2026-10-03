# Feature #17: WS2812 extras -- repeat-colour program, RGBW and 280 us-reset firmware, PIO workflow

No RTL change.

## `pio/ws2812_repeat.pio` (15 words, one state machine)
One pair of TX words (N-1, pixel) paints a run of N identical pixels, so a long strip does not need a CPU feed per pixel
(the CPU manages one word per ~3000 clocks, a pixel takes 720). Same bit timing as `ws2812.pio` (10 ticks per bit, CLKDIV 3 at
24 MHz). Registers: X = bit, Y = pixels left, ISR = the pixel; the end of a pixel is found with `jmp !osre` (pull threshold 24, or
32 for RGBW). The instructions that reload the pixel stretch the LOW time at the borders (HIGH pulses stay 3 / 7 ticks):
between pixels 4 ticks after a 1 and 8 after a 0, between commands 9 / 12 ticks (1.125 / 1.5 us). That is outside the nominal
T0L / T1L windows (+-150 ns), expected to be harmless because a strip decides on the HIGH width and latches after >= 50 us,
and **not tried on a real strip**.

## Tests
- `test_pio_protocols.py` +5 (cocotb): single run of 60 pixels | N = 1 and all-0 / all-1 pixels | three runs back to back in one frame |
  **300 pixels from one pair of words with the FIFO empty** | RGBW (32-bit) and latch between two commands. Every HIGH and LOW width
  is checked exactly (inside a pixel, at pixel borders, at run borders).
- `tools/build_pio_ws2812.py` +2 options, `test/tb_pio_cpu_ws2812.v` now takes plusargs (`+bits=32`, `+reset_us=N`, `+img=`):
  - `--rgbw` -> `pio_ws2812_rgbw_flash_image.hex`: SK6812 RGBW through the real top level (5 x 32-bit pixels, CPU halted, PIO alone).
  - `--reset-us 280` -> `pio_ws2812_reset280_flash_image.hex`: the CPU waits after `PIN_OWN` so the line is low for 343 us before the
    first bit (the default firmware only gives 126 us, enough for 50 us parts). The testbench checks lead-in >= reset time; the
    default image run with `+reset_us=280` fails (3027 clocks < 6720), as it should.
  - `make standalone-tests` runs the default, RGBW and 280 us variants (three runs of the one bench: default, RGBW, 280 us). The default image is unchanged byte for byte.
- `tools/ws2812_repeat_mutation_sweep.py` (new, shared engine `tools/mutation_common.py`): 18 one-line mutants of the repeat program, all caught (about 6 min).

## CI
`.github/workflows/pio-tests.yaml` (new): on every push runs `make standalone-tests` (all standalone benches + `Makefile.proto`)
and fails on any `FAIL` log or `<failure` in `results.xml`. Started by hand with `mutation_sweeps` ticked it also runs the four mutation
sweeps (about 1 hour more). Before this, `test.yaml` only ran the top-level cocotb test (`make`), not the PIO benches.
Tested here with cocotb 2.1.0 and Icarus 12.0; the workflow itself has not run on GitHub yet.

## Limits
Repeat program: borders not checked against a real strip; one colour per command (no gradient); count is 32-bit but a frame must
still be fed without a gap >= the reset time between commands. 280 us variant: busy-wait timing assumes the 24 MHz core and the fast
QSPI setting the benches use (re-measure with a slower SPI divider). RGBW demo is simulation only.
