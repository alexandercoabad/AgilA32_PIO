#!/usr/bin/env python3
"""build_pio_vga.py -- flash image: the CPU starts a 640x480 VGA signal generator made of two PIO state machines
and halts.  The PIO keeps drawing 8 colour bars on the Tiny VGA Pmod (uo_out[7:0] = {HS,B0,G0,R0,VS,B1,G1,R1}).

    SM0 = pio/vga_frame.pio  (words 0-22)   colour pins 0-2 + VSYNC (pin 3); counts line IRQs; palette in its ISR
    SM1 = pio/vga_line.pio   (words 23-31)  HSYNC (pin 7) + one IRQ per line
    CLKDIV 1: one PIO tick = one pixel clock.  25.175 MHz -> 59.94 Hz; this chip's 24 MHz -> 57.1 Hz.

Sequence (the same order tb_pio_vga.v uses):  load both programs, configure both machines, hand pins 0-7 to the
PIO, push the palette and force `pull` + `mov isr, osr`, force the idle pin levels (black, both syncs high),
point SM1 at its first word, then ONE write to CTRL starts both machines together.

Run from tools/:  python3 build_pio_vga.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

here = os.path.dirname(os.path.abspath(__file__))
frame = assemble(open(os.path.join(here, "..", "pio", "vga_frame.pio")).read(), origin=0)
line = assemble(open(os.path.join(here, "..", "pio", "vga_line.pio")).read(), origin=len(frame.instrs))
assert len(frame.instrs) + len(line.instrs) == 32

# white yellow cyan green magenta red blue black   (colour = R + 2*G + 4*B)
BARS = [7, 3, 6, 2, 5, 1, 4, 0]


def palette_word(bars):
    """bar k in ISR bits [31-3k : 29-3k], bar 0 leftmost."""
    return sum((c & 7) << (29 - 3 * k) for k, c in enumerate(bars))


PULL_BLOCK = assemble("pull block").instrs[0]          # 0x80A0
MOV_ISR_OSR = assemble("mov isr, osr").instrs[0]       # 0xA0C7
SET_BLACK_VS_HIGH = assemble("set pins, 8").instrs[0]  # 0xE008
SET_HS_HIGH = assemble("set pins, 1").instrs[0]        # 0xE001
assert (PULL_BLOCK, MOV_ISR_OSR) == (0x80A0, 0xA0C7)

h = PioHost()
h.load_program(frame)
h.load_program(line)
# SM0: OUT = pins 0-2, SET = pins 0-3, shift left, pull threshold 24
h.write_reg(sm_reg(0, SM_PINCTRL), pinctrl(out_base=0, out_count=3, set_base=0, set_count=4))
h.write_reg(sm_reg(0, SM_EXEC), execctrl(wrap_bottom=frame.wrap_bottom, wrap_top=frame.wrap_top))
h.write_reg(sm_reg(0, SM_SHIFT), shiftctrl(in_right=True, out_right=False, pull_thresh=24))
h.write_reg(sm_reg(0, SM_CLKDIV), clkdiv_reg(1, 0))
# SM1: SET = pin 7 (HSYNC)
h.write_reg(sm_reg(1, SM_PINCTRL), pinctrl(set_base=7, set_count=1))
h.write_reg(sm_reg(1, SM_EXEC), execctrl(wrap_bottom=line.wrap_bottom, wrap_top=line.wrap_top))
h.write_reg(sm_reg(1, SM_SHIFT), shiftctrl())
h.write_reg(sm_reg(1, SM_CLKDIV), clkdiv_reg(1, 0))
# the palette, then idle levels, then start
h.tx_push(0, palette_word(BARS))
h.force(0, PULL_BLOCK)
h.force(0, MOV_ISR_OSR)
h.force(0, SET_BLACK_VS_HIGH)
h.force(1, SET_HS_HIGH)
h.force(1, line.origin)                                # jmp <first word of vga_line>
h.write_reg(R_PIN_OWN, 0xFF)                           # pins already idle: no glitch at the hand-over
h.write_reg(R_CTRL, 0x3)                               # both machines start on the same clock
h.halt()

image = h.build()
print(f"PIO VGA flash image: {len(image)} bytes, {h.pages} pages, "
      f"{len(frame.instrs)} + {len(line.instrs)} instruction words")
write_image(image, os.path.join(here, "pio_vga_flash_image"))
print("Wrote pio_vga_flash_image.bin and .hex")
