#!/usr/bin/env python3
"""build_pio_usb.py -- flash image: the CPU loads the low-speed USB host engine (pio/usb_ls.pio),
queues an IN token, then HALTS (EBREAK). With the CPU parked, PIO sends the token, flips the same
state machine to receive, and captures the device's DATA1 reply into the RX FIFO.

  D+ = PIO pin 8 = uio[4],  D- = PIO pin 9 = uio[5];  CLKDIV 2 -> 16 clk per USB bit
  (24 MHz core clock -> 12 MHz PIO tick -> 1.5 Mb/s)

Why this needs PIO: a low-speed device answers within a few bit times of the token's EOP -- a
few hundred clocks -- while one flash-page switch of the CPU costs about 3000.

Besides the flash image this writes the test vectors for tb_pio_cpu_usb.v:
  pio_usb_token.mem     the token's stuffed logical bits (one per line, for the device to check)
  pio_usb_response.mem  the DATA1 reply's stuffed logical bits (the device's ROM)
  pio_usb_expect.hex    the RX FIFO words PIO must have captured

Run from tools/:  python3 build_pio_usb.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)
import pio_usb

SM = 0
ADDR, ENDP = 5, 1
REPLY = [0xFF, 0x00, 0xFF, 0xFF, 0x3F, 0xFC, 0x81, 0xA5]      # stuffing-heavy 8-byte payload

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "usb_ls.pio")).read(), origin=0)

token = pio_usb.token_bits(pio_usb.IN, ADDR, ENDP)
reply = pio_usb.data_bits(pio_usb.DATA1, REPLY)

h = PioHost()
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=8, out_count=2, set_base=8, set_count=2, in_base=8))
h.write_reg(sm_reg(SM, SM_EXEC), execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top))
h.write_reg(sm_reg(SM, SM_SHIFT),
            shiftctrl(autopush=True, autopull=True, in_right=False, out_right=True))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(2, 0))
h.force(SM, prog.origin + prog.labels["tx_start"])             # jmp tx_start
h.write_reg(R_PIN_OWN, 0x300)          # D+/D- -> PIO, PINDIR = 0 (released, idle J from the pulls)
h.write_reg(R_CTRL, 1 << SM)
h.tx_push_paced(SM, *pio_usb.tx_words(token))
h.halt()

image = h.build()
print(f"PIO USB flash image: {len(image)} bytes, {h.pages} pages; IN addr {ADDR} ep {ENDP}, "
      f"reply DATA1 {[hex(b) for b in REPLY]}")
write_image(image, os.path.join(here, "pio_usb_flash_image"))

# expected RX words: the stuffed bits, MSB first, 32 per word; the last (partial) word is pushed
# at SE0 (an all-zero extra word appears if the packet ends exactly on a word boundary)
words, acc, n = [], 0, 0
for b in reply:
    pass
for b in reply:
    acc = ((acc << 1) | b) & 0xFFFFFFFF
    n += 1
    if n == 32:
        words.append(acc)
        acc, n = 0, 0
words.append(acc)
with open(os.path.join(here, "pio_usb_token.mem"), "w") as f:
    f.write("\n".join(str(b) for b in token) + "\n")
with open(os.path.join(here, "pio_usb_response.mem"), "w") as f:
    f.write("\n".join(str(b) for b in reply) + "\n")
with open(os.path.join(here, "pio_usb_expect.hex"), "w") as f:
    f.write("\n".join("%08x" % w for w in words) + "\n")
print(f"token {len(token)} bits, reply {len(reply)} bits, {len(words)} RX words expected")
print("Wrote pio_usb_flash_image.{bin,hex}, pio_usb_token.mem, pio_usb_response.mem, pio_usb_expect.hex")
