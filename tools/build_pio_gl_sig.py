#!/usr/bin/env python3
"""build_pio_gl_sig.py -- flash image for the post-layout (gate-level) PIO *signature* test, test/test_pio_gl_sig.py.

The smoke test (build_pio_gl_smoke.py) proves the CPU -> PIO -> CPU path on the netlist.  This second test runs a fixed
mix of instruction forms and both open-drain pads, and compares everything it can see on the pins with golden values
that came from the RTL simulation (and were checked by hand, see test/test_pio_gl_sig.py).

    SM0 gl_sig0  (words 0-16)   ALU / shift / jump mix, shows each result on uo_out; eight "exec" words from the CPU
    SM1 gl_sig1  (words 17-30)  side-set PINDIRS on pads 8/9, input sync + bypass, autopull, autopush
    PIN_OWN = 0x3FF, SYNC_BYP = pad 8, ui_in held at 0x6A by the testbench, pads pulled up by the testbench.

CPU sequence: load both programs, configure, push the TX words, start, collect the RX words and the IRQ flags,
release the pins (PIN_OWN = 0) and show the collected words on uo_out, one byte at a time.

Run from tools/:  python3 build_pio_gl_sig.py && cp pio_gl_sig_flash_image.hex ../test/
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, R_SYNC_BYP, R_IRQ, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

here = os.path.dirname(os.path.abspath(__file__))
sig0 = assemble(open(os.path.join(here, "..", "pio", "gl_sig0.pio")).read(), origin=0)
sig1 = assemble(open(os.path.join(here, "..", "pio", "gl_sig1.pio")).read(), origin=len(sig0.instrs))


def ins(text):
    return assemble(text).instrs[0]


# Eight trips round SM0's loop; each TX word carries two instructions that SM0 runs with OUT EXEC (first = upper half).
# State when they run: y = 0xD8, isr = 0x1B53, x = 8 - trip (the loop counter), pin 6 = 1, pin 2 = 0.
EXEC_WORDS = [
    (ins("mov pins, ~y"), ins("set pins, 10")),        # 1: 0x27, then pins 0-4 = 01010 (pins 5-7 keep 001) -> 0x2A
    (ins("mov pins, isr"), ins("in y, 4")),            # 2: 0x53, then isr = (0x1B53 << 4) | 8 = 0x1B538
    (ins("jmp x!=y, 14"), ins("set pins, 30")),        # 3: taken (x = 5): the sentinel is skipped
    (ins("jmp !x, 14"), ins("mov pins, status")),      # 4: not taken (x = 4): RX level 0 < 1 -> status all ones -> 0xFF
    (ins("jmp pin, 14"), ins("set pins, 28")),         # 5: taken (pin 6 = 1): the sentinel is skipped
    (ins("wait 0 gpio 2"), ins("wait 1 pin 4")),       # 6: both already true (pin 2 = 0; IN_BASE 2 + 4 -> pin 6 = 1): no stall
    (ins("jmp y--, 14"), ins("set pins, 27")),         # 7: taken (y = 0xD8 -> 0xD7): the sentinel is skipped
    (ins("mov pins, y"), ins("set pins, 5")),          # 8: 0xD7 (shows the decrement), then pins 0-4 = 00101 -> 0xC5
]
TX0 = [(a << 16) | b for a, b in EXEC_WORDS]
TX1 = [0x00005AC3]                                      # SM1: x = 0xC3, y = 0x5A (autopull, shift right)

h = PioHost()
h.fast_qspi()                    # first thing: run the rest of the image at the fastest QSPI divider
h.load_program(sig0)
h.load_program(sig1)
h.write_reg(sm_reg(0, SM_PINCTRL), pinctrl(out_base=0, out_count=8, set_base=0, set_count=5, in_base=2))
h.write_reg(sm_reg(0, SM_EXEC), execctrl(wrap_bottom=sig0.wrap_bottom, wrap_top=sig0.wrap_top, jmp_pin=6,
                                         status_sel=1, status_n=1))
h.write_reg(sm_reg(0, SM_SHIFT), shiftctrl(in_right=False, out_right=False))
h.write_reg(sm_reg(0, SM_CLKDIV), clkdiv_reg(1, 0))
h.write_reg(sm_reg(1, SM_PINCTRL), pinctrl(side_base=8, side_count=3, in_base=8))
h.write_reg(sm_reg(1, SM_EXEC), execctrl(wrap_bottom=sig1.wrap_bottom, wrap_top=sig1.wrap_top,
                                         side_en=True, side_pindir=True))
h.write_reg(sm_reg(1, SM_SHIFT), shiftctrl(autopush=True, autopull=True, in_right=False, out_right=True,
                                           push_thresh=8, pull_thresh=16))
h.write_reg(sm_reg(1, SM_CLKDIV), clkdiv_reg(2, 128))
h.force(0, sig0.wrap_bottom)                             # jmp 0 (the PC is already 0 after reset; kept for clarity)
h.force(1, sig1.wrap_bottom)                             # jmp 17
h.write_reg(R_SYNC_BYP, 0x100)                           # pad 8 bypasses the synchroniser, pad 9 does not
for w in TX0[:4]:                                        # the FIFO holds four; the rest follow once SM0 has started
    h.tx_push_paced(0, w)
for w in TX1:
    h.tx_push_paced(1, w)
h.write_gpio_out_imm(0)                                  # LED_OUT = 0: what the pads show when the PIO lets go
h.write_reg(R_PIN_OWN, 0x3FF)
h.write_reg(R_CTRL, 0x3)
for w in TX0[4:]:
    h.tx_push_paced(0, w)
h.wait_rx_ready(0)
h.rx_pop(0, 10)                                          # SM0 push: isr
for r in (11, 12, 13):                                   # SM1 autopush words
    h.wait_rx_ready(1)
    h.rx_pop(1, r)
h.write_idx(R_IRQ)
h.read_data(14)                                          # IRQ flags
h.write_reg(R_PIN_OWN, 0)                                # give the pins back to the CPU
for k in range(4):                                       # four bytes of SM0's word, low byte first
    h.write_gpio_out(10)
    if k < 3:
        h.shift_right(10, 8)
for r in (11, 12, 13, 14):
    h.write_gpio_out(r)
h.halt()

image = h.build()
print(f"PIO signature image: {len(image)} bytes, {h.pages} pages, {len(sig0.instrs)} + {len(sig1.instrs)} PIO words")
write_image(image, os.path.join(here, "pio_gl_sig_flash_image"))
print("Wrote pio_gl_sig_flash_image.bin and .hex (copy the .hex to ../test/)")
