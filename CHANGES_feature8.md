# Feature #8: CPU-driven demo of all four SPI modes (0-3)

Closes the "Not done" item of feature #7: modes 2 and 3 (and 0 and 1) now run from real CPU
firmware through the top level (`tt_um_agila32`), from a flash image, not only at the `pio.v` level.
No RTL change.

## What it does
One flash image (`tools/build_pio_spi4.py` -> `pio_spi4_flash_image.{bin,hex}`, 2420 bytes,
55 pages) reprograms the same PIO state machine for SPI mode 0, 1, 2 and 3 in turn
(`spi_master`, `spi_cpha1`, `spi_cpol1_cpha0`, `spi_cpol1_cpha1`; CLKDIV 8, 8-bit MSB first,
autopull + autopush at 8). Pins: MOSI `uo_out[0]`, SCK `uo_out[1]`, MISO `ui_in[2]`,
CS_n `uo_out[2]` (CPU `GPIO_OUT` bit 2, not a PIO pin).

The CPU is in the data path: it reads each mode's MISO byte from the RX FIFO and sends it as the
first MOSI byte of the next mode (`slave MISO -> input sync -> autopush -> RX FIFO -> CPU LW/SW ->
TX FIFO -> autopull -> MOSI`). After mode 3 it writes the last MISO byte to `GPIO_OUT` and EBREAKs.

## Tools
`tools/pio_host.py` gained firmware primitives (each atomic within a flash page):
`wait_rx_ready`, `rx_pop`, `shift_left`, `shift_right`, `store_data_reg`, `tx_push_reg_paced`,
`write_gpio_out_imm`.

## Test: `test/tb_pio_cpu_spi4.v` (wired into `test/Makefile`)
A slave model decodes the wire from the pins only (per CPOL/CPHA; it never sees the PIO program):
- SCK idles at CPOL when CS falls and rises; 16 rising + 16 falling edges inside each CS window
- both MOSI bytes correct, MSB first; MOSI stable >= 4 clocks before and after every sampling edge
- MOSI byte 0 of mode N+1 == MISO byte 0 of mode N (the CPU really read MISO and re-sent it)
- last MISO byte (0x6D) ends up on `GPIO_OUT`
- no SCK edge while CS is high except the single intentional 0 -> 1 idle-level change before mode 2
- four windows strictly sequential, then the core halts

Mutation-checked (each makes the test fail): relay replaced by a constant; mode 2 running the
mode-0 program; relay byte not shifted into the top byte; mode 3 running the mode-1 program;
CPOL=1 idle preset dropped.

## Pitfall found by this test
The first version passed every functional check but failed "no SCK edges while CS is high": between
mode 2 and mode 3, SCK dipped low for one pulse. Cause: the forced `jmp` that points the SM at its
program is also a side-set instruction. With a plain `.side_set 1` its side-set value is bit 12, so
`0x0000 | addr` drove SCK LOW. On a CPOL=1 bus that is a spurious clock pulse. Fix: force
`(cpol << 12) | addr`, i.e. carry the idle level. (Same root cause as the `set pins` note in
feature #7, but for `jmp`.) A CS-gated slave would ignore it, which is exactly why a functional
check alone did not catch it.

## Not done
- Still one slave and one CS; no multi-slave or CS-per-mode demo.
- Simulation only (RTL). Not re-run through the GDS / gate-level flow (no RTL change, so the
  existing GL result is unaffected, but `tb_pio_cpu_spi4` has not been run on the netlist).
