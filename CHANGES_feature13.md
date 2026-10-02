# Feature #13: CPU-driven top-level demo of the PIO PS/2 receiver

Closes the open item left by feature 10 ("wiring the receiver into a CPU-driven top-level demo"). **No RTL
change, no PIO program change.**

## New
- `tools/build_pio_ps2_rx.py` -> `tools/pio_ps2_rx_flash_image.{bin,hex}` (968 bytes, 22 pages):
  load `pio/ps2_rx.pio`, configure (IN_BASE 4, JMP_PIN 3, autopush 11, shift right, CLKDIV 17), enable,
  sleep, then 7 x { wait RX non-empty, pop, `(w >> 22) & 0xFF`, SB to LED_OUT }, EBREAK.
- `test/tb_pio_cpu_ps2.v` (+ `test/pio_ps2_rx_flash_image.hex`): a real-rate keyboard model (10 kHz at 24 MHz,
  odd parity, mid-high DATA changes, 7200-clock frame gaps) typing 1C F0 1C 32 F0 32 21. Checks: CPU asleep
  and `uo_out` untouched while four frames arrive; the RX FIFO holds exactly those four, each with a valid
  start/parity/stop; then `uo_out` shows the seven scancodes in order. Part of `make standalone-tests`.
- `PioHost.ps2_frame_to_byte(rd)` and `PioHost.delay_iterations(n)` in `tools/pio_host.py`.

## Mutation checks (both restored byte-identical)
- CLKDIV 2: the idle timeout (576 clocks) is shorter than CLOCK-high (1200): FIFO holds 0 frames, FAIL.
- CPU shifts by 21 instead of 22: all scancodes wrong, FAIL.

## Things that went wrong on the way
- **The delay helper was 27x off.** The first image slept 4.6 million clocks instead of 170000: the CPU's
  ADDI/BNE loop takes about 274 clocks per pass (measured with throwaway images that sleep 20000 and 40000
  requested clocks: 548683 and 1097231 clocks), not the ~14 the docstring assumed. `delay()` still
  over-waits (harmless, earlier demos only needed protocols to finish); the PS/2 demo uses the exact
  `delay_iterations(560)` (about 153000 clocks). Measured at `qspi_div_sel = 0`; a slower QSPI divider makes
  the loop slower.
- **A duplicated helper in `pio_host.py`.** When the I2C slave demo (feature 12) was merged, it added a
  second `wait_rx_ready` that silently overrode the one the JTAG feature already had (same behaviour, so
  nothing broke). Removed the later copy and made `rx_get` call `rx_pop`. All seven existing flash images
  (uart, i2c, multi, usb, spi4, jtag, i2c_slave) were rebuilt and are **byte-identical** to the repo's.
- A scripted edit of `pio_host.py` replaced an empty string (a slice taken in the wrong order) and
  corrupted the file; caught on the next build and restored from the repo before continuing.

## Not done
Host-to-device commands (LED set, reset: needs CLOCK/DATA open-drain on `uio[5:4]`), scancode-to-ASCII,
a real keyboard, gate-level run of this bench.
