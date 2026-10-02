#!/usr/bin/env python3
"""build_pio_ws2812.py -- flash image: the CPU loads `ws2812.pio`, preloads four pixels into the TX FIFO,
starts SM0, queues a fifth pixel, then HALTS (EBREAK). The state machine keeps clocking the strip's data
line on uo_out[0] with the CPU parked.

  PIO SM0, side-set pin 0 = uo_out[0], CLKDIV = 3.0 at the 24 MHz in info.yaml  ->  PIO tick 8 MHz (125 ns),
  10 ticks per bit = 1.25 us, 24 bits per pixel = 30 us per pixel (720 clocks), idle LOW.

Why four words are preloaded with the state machine still disabled: the CPU needs about 3000 clocks per
queued word (flash paging) but a pixel only takes 720 clocks to send, so a CPU that feeds the FIFO one word
at a time leaves a ~100 us gap between pixels -- longer than the strip's 50 us reset time, so the strip
would latch after every pixel. A full TX FIFO (4 words) lets four pixels go out back to back, and the fifth
word, queued right after the enable, lands before the FIFO runs dry. Longer strips need a faster feed than
this CPU provides (see docs/info.md).

Hand-over: GPIO_OUT[0] is set LOW before PIN_OWN gives the pad to the PIO, so the line does not pulse when
ownership changes. The first pixel is not sent until the CPU has paged in more code after PIN_OWN, which
also leaves the line low for far longer than the reset time (anything the boot LED counter put on the pad
before this point is flushed by the strip's latch).

Run from tools/:  python3 build_pio_ws2812.py   (then copy pio_ws2812_flash_image.hex to test/)

NEGATIVE TESTS (the testbench must FAIL on these; run it with +img=<name>.hex):
  --naive          enable SM0 first, then queue each pixel with a 1500-clock pause (a slow feeder):
                   the strip would latch after every pixel   -> pio_ws2812_naive_flash_image.hex
  --bad-handover   GPIO_OUT[0] is HIGH when PIN_OWN hands the pad over, so the line drops -> pulse at
                   the hand-over                             -> pio_ws2812_badown_flash_image.hex
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

NAIVE = "--naive" in sys.argv
BAD_HANDOVER = "--bad-handover" in sys.argv
SM = 0
CLKDIV = 3
# G R B per pixel (WS2812 wants green first). Red, green, blue, white, an arbitrary mix. Keep in sync
# with the expected values in test/tb_pio_cpu_ws2812.v.
PIXELS_GRB = [0x00FF00, 0xFF0000, 0x0000FF, 0xFFFFFF, 0x12AB7E]

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "ws2812.pio")).read(), origin=0)

h = PioHost()
h.write_gpio_out_imm(1 if BAD_HANDOVER else 0)   # the pad is driven from GPIO_OUT until PIN_OWN: make that LOW
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=0, out_count=0, set_base=0, set_count=0,
                    side_base=0, side_count=prog.side_bits))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                     side_en=prog.side_opt, side_pindir=prog.side_pindirs))
h.write_reg(sm_reg(SM, SM_SHIFT),
            shiftctrl(autopull=True, pull_thresh=24, out_right=False))     # MSB first, 24-bit pixels
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.write_reg(R_PIN_OWN, 0x001)              # uo_out[0] -> PIO (output register resets low: no glitch)
if NAIVE:
    h.write_reg(R_CTRL, 1 << SM)
    for px in PIXELS_GRB:
        h.tx_push(SM, px << 8)
        h.delay(1500)                      # slower than the strip's 50 us reset time (1200 clocks)
else:
    h.tx_push(SM, *[p << 8 for p in PIXELS_GRB[:4]])      # fill the 4-deep TX FIFO while SM0 is disabled
    h.write_reg(R_CTRL, 1 << SM)           # go: four pixels stream out back to back
    h.tx_push(SM, PIXELS_GRB[4] << 8)      # fifth pixel, queued while the first four are on the wire
h.halt()

image = h.build()
print(f"PIO WS2812 flash image: {len(image)} bytes, {h.pages} pages, {len(PIXELS_GRB)} pixels")
stem = "pio_ws2812_naive_flash_image" if NAIVE else "pio_ws2812_badown_flash_image" if BAD_HANDOVER else "pio_ws2812_flash_image"
write_image(image, os.path.join(here, stem))
print("Wrote", stem + ".bin and .hex" + ("   [NEGATIVE TEST IMAGE]" if NAIVE or BAD_HANDOVER else ""))
