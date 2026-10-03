#!/usr/bin/env python3
"""build_pio_gl_smoke.py -- flash image for the post-layout (gate-level) PIO smoke test, test/test_pio_gl.py.

The CPU loads two tiny PIO programs, then for each of three words: pushes it to SM0 (pio/gl_echo.pio), waits for the
echo, and shows the result byte on uo_out (bit 7 belongs to the PIO).  Every echo makes SM1 (pio/gl_pulses.pio) put a
four-pulse burst on uo_out[7].

    SM0 gl_echo    result = bit_reverse(~word); raises IRQ 0         words 0-4,  CLKDIV 1
    SM1 gl_pulses  waits IRQ 0, four pulses on pin 7                 words 5-9,  CLKDIV 2
    PIN_OWN = 0x80: only pin 7 is handed to the PIO

Run from tools/:  python3 build_pio_gl_smoke.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

WORDS = [0x00000055, 0x00000001, 0x000000F0]          # keep in sync with test/test_pio_gl.py
R = 10

here = os.path.dirname(os.path.abspath(__file__))
echo = assemble(open(os.path.join(here, "..", "pio", "gl_echo.pio")).read(), origin=0)
pulses = assemble(open(os.path.join(here, "..", "pio", "gl_pulses.pio")).read(), origin=len(echo.instrs))

h = PioHost()
h.load_program(echo)
h.load_program(pulses)
h.write_reg(sm_reg(0, SM_PINCTRL), pinctrl(set_count=0))
h.write_reg(sm_reg(0, SM_EXEC), execctrl(wrap_bottom=echo.wrap_bottom, wrap_top=echo.wrap_top))
h.write_reg(sm_reg(0, SM_SHIFT), shiftctrl())
h.write_reg(sm_reg(0, SM_CLKDIV), clkdiv_reg(1, 0))
h.write_reg(sm_reg(1, SM_PINCTRL), pinctrl(set_base=7, set_count=1))
h.write_reg(sm_reg(1, SM_EXEC), execctrl(wrap_bottom=pulses.wrap_bottom, wrap_top=pulses.wrap_top))
h.write_reg(sm_reg(1, SM_SHIFT), shiftctrl())
h.write_reg(sm_reg(1, SM_CLKDIV), clkdiv_reg(2, 0))
h.force(0, echo.wrap_bottom)                                 # jmp pull   (wrap_bottom is absolute)
h.force(1, assemble("set pins, 0").instrs[0])                # pin 7 idles low before the hand-over
h.force(1, pulses.wrap_bottom)                             # jmp wait
h.write_reg(R_PIN_OWN, 0x80)
h.write_reg(R_CTRL, 0x3)
for w in WORDS:
    h.tx_push_paced(0, w)
    h.wait_rx_ready(0)
    h.rx_pop(0, R)
    h.shift_right(R, 24)                                      # top byte = result
    h.write_gpio_out(R)
h.halt()

image = h.build()
print(f"PIO post-layout smoke image: {len(image)} bytes, {h.pages} pages, "
      f"{len(echo.instrs)} + {len(pulses.instrs)} PIO words")
write_image(image, os.path.join(here, "pio_gl_smoke_flash_image"))
print("Wrote pio_gl_smoke_flash_image.bin and .hex")
