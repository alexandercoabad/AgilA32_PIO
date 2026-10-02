#!/usr/bin/env python3
"""build_pio_ps2_rx.py -- flash image: the CPU loads the PIO PS/2 receiver (pio/ps2_rx.pio), then
SLEEPS while a keyboard types, then drains the RX FIFO and shows each scancode on uo_out.

  CLOCK = ui_in[3] = PIO pin 3,  DATA = ui_in[4] = PIO pin 4        (the same wiring as build_ps2_reader.py)
  CLKDIV 17  ->  288 PIO cycles x 17 = 4896 clk = 204 us at 24 MHz (the idle-gap timeout; the program's
  header asks for 100..300 us)

Why this exists: the polled reader (build_ps2_reader.py) needs one flash page per CLOCK edge, about 3000
clocks per page, and cannot keep up with a real 10 kHz keyboard (2400 clocks per bit at 24 MHz). Here
PIO watches CLOCK; the CPU is asleep (a delay loop) while the first four frames arrive and are held in
the 4-deep RX FIFO, then wakes up, drains them, and keeps polling for the rest.

Run from tools/:  python3 build_pio_ps2_rx.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

SM = 0
CLKDIV = 17
N_FRAMES = 7                    # 4 are buffered while the CPU sleeps, 3 arrive while it polls
SLEEP_PASSES = 560              # ~274 clocks per pass (measured) = ~153000 clocks > 4 frames x (11 x 2400 + 7200) = 134400

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "ps2_rx.pio")).read(), origin=0)

h = PioHost()
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL), pinctrl(in_base=4, set_count=0))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top, jmp_pin=3))
h.write_reg(sm_reg(SM, SM_SHIFT), shiftctrl(autopush=True, in_right=True, push_thresh=11))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.force(SM, prog.origin + prog.wrap_bottom)            # jmp start
h.write_reg(R_CTRL, 1 << SM)                           # PIO now watches the keyboard on its own
h.delay_iterations(SLEEP_PASSES)                       # CPU asleep: 4 frames pile up in the RX FIFO
for _ in range(N_FRAMES):
    h.wait_rx_ready(SM)
    h.rx_get(SM, 10)
    h.ps2_frame_to_byte(10)
    h.write_gpio_out(10)                               # SB -> LED_OUT = uo_out
h.halt()

image = h.build()
print(f"PIO PS/2 receiver flash image: {len(image)} bytes, {h.pages} pages, "
      f"{len(prog.instrs)}-word program, CLKDIV {CLKDIV}")
write_image(image, os.path.join(here, "pio_ps2_rx_flash_image"))
print("Wrote pio_ps2_rx_flash_image.bin and .hex")
