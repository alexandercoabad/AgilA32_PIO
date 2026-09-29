#!/usr/bin/env python3
"""build_pio_i2c.py -- flash image: the CPU loads the Pico-SDK-derived I2C master
(pio/i2c.pio), starts it on PIO SM0, queues a whole I2C write transaction, then HALTS
(EBREAK). The state machine keeps clocking the bus with the CPU parked.

  SDA = PIO pin 8 = uio[4],  SCL = PIO pin 9 = uio[5]  (open drain, external pull-ups)
  CLKDIV = 32.0 -> one SCL period = 32 PIO ticks x 32 = 1024 clk (~50 kHz at 50 MHz).
  The CPU needs ~3000 clk per FIFO word (every word is several flash-paged pushes), so the
  bus must be slower than that per byte (9 SCL periods x 1024 clk) for the CPU to get ahead
  of it and park before the transaction is over.

  Transaction: START, 0x50 << 1 | W, 0xA5, 0x3C, STOP.

The TX FIFO is only 4 deep and the transaction is 11 words, so every push first spins on
FSTAT.TXFULL (PioHost.tx_push_paced). Pins are handed over with PINDIR = 0 (released), so
nothing glitches when PIN_OWN takes uio[4]/uio[5] from the QSPI logic.

Run from tools/:  python3 build_pio_i2c.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)
import pio_i2c

ADDR7 = 0x50
DATA = [0xA5, 0x3C]
SM = 0

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "i2c.pio")).read(), origin=0)
entry = prog.origin + prog.labels["entry_point"]

h = PioHost()
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=8, out_count=1, set_base=8, set_count=1,
                    side_base=9, side_count=prog.side_bits, in_base=8))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                     side_en=prog.side_opt, side_pindir=prog.side_pindirs, jmp_pin=8))
h.write_reg(sm_reg(SM, SM_SHIFT),
            shiftctrl(autopush=True, autopull=True, in_right=False, out_right=False,
                      push_thresh=8, pull_thresh=16))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(32, 0))
h.force(SM, entry)                     # jmp entry_point
h.write_reg(R_PIN_OWN, 0x300)          # uio[4]/uio[5] -> PIO, PINDIR = 0 (bus released)
h.write_reg(R_CTRL, 1 << SM)           # enable SM0
h.tx_push_paced(SM, *[pio_i2c.fifo_word(w) for w in pio_i2c.write_transaction(ADDR7, DATA)])
h.halt()

image = h.build()
print(f"PIO I2C flash image: {len(image)} bytes, {h.pages} pages, "
      f"write {[hex(d) for d in DATA]} to 0x{ADDR7:02x}")
write_image(image, os.path.join(here, "pio_i2c_flash_image"))
print("Wrote pio_i2c_flash_image.bin and .hex")
