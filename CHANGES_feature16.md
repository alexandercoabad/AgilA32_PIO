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
