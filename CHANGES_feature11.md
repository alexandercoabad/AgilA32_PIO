# Feature #11: low-speed USB (1.5 Mb/s) host engine on the PIO, and a 24 MHz clock

Takes "Low-speed USB" off the "Not attempted" list (the brief's stretch goal). **No RTL change.**

## Clock: 1 MHz -> 24 MHz
`info.yaml` `clock_hz` 24000000 and `src/config.json` `CLOCK_PERIOD` 41.667 ns. Low-speed USB needs a
12 MHz PIO tick (8 ticks per 1.5 Mb/s bit) = 24 MHz / CLKDIV 2. The previous GDS run (run #18, 1 MHz
target) reported a register-to-register limit of 21.5 ns / 46.5 MHz in the slow corner, 13.5 ns / 74 MHz
typical, 8.75 ns / 114 MHz fast. **Not yet re-run at the new period** -- that is the first thing to
check. The QSPI divider resets to the slowest setting (clk/128), so flash/PSRAM timing is unaffected.

## `pio/usb_ls.pio` (29 of 32 words, original)
TX: count word + stuffed logical bits; PIO does NRZI, EOP (SE0 x 2, J x 1) and releases the bus.
RX: after TX the same state machine waits for K, samples {D-, D+} mid-bit, NRZI-decodes, pushes bits
(first = MSB) and flushes the partial word at SE0. Pins D+/D- = PIO 8/9 = `uio[4]`/`uio[5]`.
`tools/pio_usb.py` builds tokens/data/handshakes (CRC5, CRC16 verified against the standard vectors
`2D 00 10`, `69 01 E8`, `DD 94`) and decodes RX words. `tools/build_pio_usb.py` + `test/tb_pio_cpu_usb.v`:
CPU queues an IN token and halts; PIO sends it, turns the bus around and captures an 8-byte DATA1.

## Tests: +7 cocotb (26 -> 33 with PS/2), +1 end-to-end bench
Details in `docs/info.md`. Mutation-checked: flipping one bit of the device's reply makes
`tb_pio_cpu_usb.v` fail with the exact differing RX word.

## Things that went wrong on the way
- **Last bit one third too short.** The first waveform test failed: the last data bit lasted 5 ticks
  instead of 8 because EOP started right after `jmp x--`. Fixed with `nop [2]`; the edge-spacing test
  (every edge on a 16-clock boundary, SE0 exactly 32 clocks) would catch a regression.
- **Receiver sample point.** First guess put every sample 3 clocks before the end of the bit; the clock
  sweep was lopsided (only 0 to +0.8 % worked). Moving the sample to ~9 of 16 clocks gave the window in
  the docs (-0.5 % to +0.8 %).
- **A terminator that lost bits.** RX first appended `0 11111111` with `in y, 8` so the host could find
  the packet end. Autopush discards bits that do not fit in the 32-bit ISR, so when the 8 ones straddled
  a word boundary the last word was corrupted (4 of 12 fuzz packets failed). Dropped the terminator; the
  host recovers the length from SYNC + PID check + CRC (3000 random packets decode offline; fuzz passes).
- **Test harness, twice.** A cocotb test that loops must reuse one clock/`World` (starting a second
  clock corrupts timing), and the RX collector's quiet time had to exceed one word time (512 clocks).

## Not done
Per-transition resync (wider RX tolerance), device-side responses, SETUP + DATA0 back to back (needs a
deeper TX FIFO or a faster CPU feed), electrical measurements, 10BASE-T.
