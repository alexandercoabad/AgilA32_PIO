# Feature #16: VGA 640x480 colour bars on the Tiny VGA Pmod (`pio/vga_frame.pio`, `pio/vga_line.pio`)

No RTL change. Two PIO programs (23 + 9 = all 32 instruction words), two Verilog testbenches, one CPU firmware image.

## Programs
- `pio/vga_line.pio` (SM1, 9 words): free-running 800-tick line, HSYNC (pin 7) low 96 ticks, `irq set 0` at tick 143.
- `pio/vga_frame.pio` (SM0, 23 words): counts line IRQs: VSYNC (pin 3) 2 lines, 33 back porch, 480 picture lines of 8 colour
  bars x 80 ticks on pins 0-2, 9 front porch = 525 lines. The palette (8 x 3 bits, MSB first) sits in the ISR and can be
  rewritten by the CPU (push, force `pull block`, `mov isr, osr`).
- CLKDIV 1: one tick = one pixel clock. 25.175 MHz -> 59.94 Hz; 24 MHz -> 57.1 Hz.

## Tests
- `test/tb_pio_vga.v` (new, in `make standalone-tests`): VGA monitor model on pio.v alone, three frames: HSYNC period 800 and
  low 96 on every line, VSYNC 2 lines, 420000-clock frames with 525 HSYNC pulses, 480 picture lines of 8 exact 80-clock bars,
  same picture start on every line, black everywhere else, unused LSB pins low, palette change applies from the next frame.
- `tools/build_pio_vga.py` + `test/tb_pio_cpu_vga.v` (new, in `make standalone-tests`; 28 -> 30 standalone): the CPU loads both programs,
  pushes the palette, hands uo_out[7:0] to the PIO (idle levels forced first: no glitch), starts both machines with one CTRL write
  and halts; the monitor in the testbench re-checks timing, bars and the halt over three frames.
- Mutation-checked: 9 mutants (see docs/info.md) all fail.

## Limits
8 colours at about 2/3 brightness (pins 4-6 not driven), fixed bar pattern with a programmable palette, no framebuffer (the CPU
manages one word per ~3000 clocks, a line is 800), 57 Hz at 24 MHz, no real monitor tried.

## Addendum: VGA mutation sweep (`tools/vga_mutation_sweep.py`)
Breaks `pio/vga_frame.pio` / `pio/vga_line.pio` one line at a time (23 mutants, instruction count kept at 23 + 9: back porch 32,
one-line VSYNC, VSYNC never low or stuck low, 448 / 479 picture lines, front porch 8, 79- and 81-tick bars, no palette reload,
wrong shift width, line counter removed, HSYNC 95 / 128 ticks or split or never high, 799- and 776-tick lines, IRQ late / missing /
wrong flag) on a temporary copy. Layer 1 = `tb_pio_vga.v`, layer 2 = firmware rebuilt + `tb_pio_cpu_vga.v` for survivors
(`--all-layers` runs both for every mutant). Shared engine: `tools/mutation_common.py`. About 8 minutes for all of layer 1.
Result: all 23 caught by `tb_pio_vga.v`.
One thing went wrong on the first run: **V12 survived** (no black after the last bar). Both palettes of the testbench ended with a
black bar, so blanking the line after the last bar was never visible. The second palette is now
`4 1 5 2 0 3 7 6` (black in the middle, coloured last bar); V12 is caught ("picture does not start at the same clock"), the
unmutated programs still pass. The CPU-level bench still uses the default palette (ends in black) and does not catch V12 on its own.
