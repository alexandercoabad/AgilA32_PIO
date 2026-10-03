#!/usr/bin/env python3
"""build_pio_ws2812_repeat.py -- flash image: the CPU loads `ws2812_repeat.pio`, queues THREE colour runs
(30 green, 20 red, 10 blue = 60 pixels, one frame) with only SIX FIFO words and HALTS (EBREAK). The state machine
keeps clocking the strip's data line on uo_out[0] with the CPU parked.

  PIO SM0, side-set pin 0 = uo_out[0], CLKDIV = 3.0 at the 24 MHz in info.yaml -> PIO tick 125 ns, 10 ticks per bit,
  24 bits per pixel = 30 us per pixel (720 clocks); 60 pixels = 43,200 clocks, far longer than the CPU needs.

A command is two TX words: N-1, then the pixel (GRB << 8). The 4-deep FIFO holds two commands, so the first two are
queued while SM0 is disabled; the third goes in right after the enable (the state machine has already taken the
first command's words by then).  The plain `ws2812.pio` could not do this: 60 pixels would need 60 words.

Hand-over as in build_pio_ws2812.py: GPIO_OUT[0] is set LOW before PIN_OWN gives the pad to the PIO, so the line does
not pulse when ownership changes.

NEGATIVE TESTS (the testbench must FAIL on these; run it with +img=<name>.hex):
  --bad-count      the count words hold N instead of N-1 (one pixel too many per run) -> pio_ws2812_repeat_badcount_flash_image.hex
  --bad-handover   GPIO_OUT[0] HIGH at the hand-over, so the line drops -> pulse     -> pio_ws2812_repeat_badown_flash_image.hex

Run from tools/:  python3 build_pio_ws2812_repeat.py   (then copy pio_ws2812_repeat_flash_image.hex to test/)
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

BAD_COUNT = "--bad-count" in sys.argv
BAD_HANDOVER = "--bad-handover" in sys.argv
SM = 0
CLKDIV = 3
# (pixels, GRB) -- keep in sync with the expected runs in test/tb_pio_cpu_ws2812_repeat.v
RUNS = [(30, 0xFF0000), (20, 0x00FF00), (10, 0x0000FF)]


def words(run):
    n, grb = run
    return [n if BAD_COUNT else n - 1, grb << 8]


here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "ws2812_repeat.pio")).read(), origin=0)

h = PioHost()
h.write_gpio_out_imm(1 if BAD_HANDOVER else 0)       # the pad is driven from GPIO_OUT until PIN_OWN: make that LOW
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=0, out_count=0, set_base=0, set_count=0, side_base=0, side_count=prog.side_bits))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                     side_en=prog.side_opt, side_pindir=prog.side_pindirs))
h.write_reg(sm_reg(SM, SM_SHIFT), shiftctrl(autopull=False, pull_thresh=24, out_right=False))   # MSB first, 24 bit
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.write_reg(R_PIN_OWN, 0x001)                        # uo_out[0] -> PIO (output register resets low: no glitch)
h.tx_push(SM, *(words(RUNS[0]) + words(RUNS[1])))    # two commands fill the 4-deep FIFO while SM0 is disabled
h.write_reg(R_CTRL, 1 << SM)                         # go
h.tx_push(SM, *words(RUNS[2]))                       # third command, queued while the first run is on the wire
h.halt()

image = h.build()
print(f"PIO WS2812 repeat flash image: {len(image)} bytes, {h.pages} pages, runs {RUNS}, "
      f"{sum(n for n, _ in RUNS)} pixels from {2 * len(RUNS)} FIFO words")
stem = ("pio_ws2812_repeat_badcount_flash_image" if BAD_COUNT else
        "pio_ws2812_repeat_badown_flash_image" if BAD_HANDOVER else "pio_ws2812_repeat_flash_image")
write_image(image, os.path.join(here, stem))
print("Wrote", stem + ".bin and .hex" + ("   [NEGATIVE TEST IMAGE]" if BAD_COUNT or BAD_HANDOVER else ""))
