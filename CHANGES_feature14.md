# Feature #14: WS2812 / SK6812 LED strip driver (`pio/ws2812.pio`)

No RTL change. A 4-instruction PIO program, 5 cocotb tests, and a CPU-driven top-level demo.

## `pio/ws2812.pio`
Three-phase bit loop on one side-set pin, idle low, MSB first, autopull at 24 (or 32) bits. 10 ticks per bit;
at the 24 MHz in `info.yaml` and CLKDIV 3 the tick is 125 ns: 0 = 375 ns high / 875 ns low, 1 = 875 / 375 ns
(datasheet 400/850 and 800/450, each +-150 ns). The delays are `.define`d `T1`/`T2`/`T3`.

## Tests
- `test/pio_tb_lib.py`: `Ws2812Strip` (decodes bits from pulse widths, latches on a low >= reset time).
  `setup()` in `test_pio_protocols.py` takes an optional clock period (the WS2812 tests run at 41.666 ns; 41.667 is an
  odd number of ps and the simulator refuses it).
- `test_pio_protocols.py` +5 (-> 62): single pixel with exact timing and datasheet windows | 8 pixels back to back |
  two frames each latched | feed gap > reset time splits the frame | 32-bit RGBW.
- `tools/build_pio_ws2812.py` + `test/tb_pio_cpu_ws2812.v` (wired into `make standalone-tests`, 26 -> 27): CPU preloads 4
  pixels, enables, queues a 5th, halts. Strip decode, exact 9/21-clock pulses, 120 pulses and no others, no glitch at
  `PIN_OWN`, 126 us low before the first bit, longest in-frame gap 21 clocks, core halted 2771 clocks before the end.
- Mutation-checked: program mutants (side-set polarity, T2 = 3, 6-tick zero, zero never drops) fail; firmware negatives
  `--naive` (slow feeder, 41,772-clock gap) and `--bad-handover` fail the top-level testbench.

## Finding worth knowing
The CPU feeds ~1 word per 3000 clocks but a pixel is 720 clocks long, so a one-word-at-a-time feeder makes the strip latch
after every pixel. The TX FIFO (4 words) is what makes a short frame possible: preload while disabled, enable, queue one
more. Realistic ceiling for CPU-fed frames: ~5-6 pixels (5 demonstrated).

## Not done
Real strip / level shifting / 280 us-reset parts (the 126 us lead-in is only enough for 50 us parts), a repeat-colour
program for long strips, RGBW top-level demo.
