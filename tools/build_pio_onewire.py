#!/usr/bin/env python3
"""build_pio_onewire.py -- flash image: the CPU is a 1-Wire bus master through the PIO.

    reset the bus, read the presence pulse               LED_OUT = 0xA0 (device answered) / 0xA1 (nobody)
    send the ROM command READ ROM (0x33)
    read the 8 bytes the device answers with             LED_OUT = each byte in turn (family code first,
                                                          CRC-8 last)
    EBREAK

  DQ = PIO pin 8 = uio[4] (open drain, external ~4.7 kOhm pull-up), CLKDIV 24 -> a 1 us PIO tick at 24 MHz.
  Only pad 8 is handed to the PIO (PIN_OWN bit 8): uo_out is untouched, so the CPU's LED_OUT writes show.

Each command is one FIFO word and produces one RX word (see pio/onewire.pio):
    reset 0;   transfer = 1 | (n-1) << 1 | data << 6;   a read sends ones.

Run from tools/:  python3 build_pio_onewire.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

SM = 0
CLKDIV = 24                  # 1 us tick at 24 MHz
READ_ROM = 0x33
N_ROM_BYTES = 8
R = 10                       # scratch register for RX words

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "onewire.pio")).read(), origin=0)


def ow_xfer(nbits, data):
    assert 1 <= nbits <= 26
    return 1 | ((nbits - 1) << 1) | ((data & ((1 << nbits) - 1)) << 6)


OW_RESET = 0
h = PioHost()
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL), pinctrl(set_base=8, set_count=1, in_base=8))
h.write_reg(sm_reg(SM, SM_EXEC), execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top))
h.write_reg(sm_reg(SM, SM_SHIFT), shiftctrl(autopush=False, autopull=False, in_right=True, out_right=True))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.force(SM, prog.wrap_bottom)                          # jmp cmd
h.write_reg(R_PIN_OWN, 0x100)                          # DQ (pad 8) -> PIO; the line is released (pindir 0)
h.write_reg(R_CTRL, 1 << SM)

# 1. reset + presence
h.tx_push_paced(SM, OW_RESET)
h.wait_rx_ready(SM)
h.rx_pop(SM, R)
h.shift_right(R, 31)                                   # 0 = a device pulled the line low
h.add_imm(R, 0xA0)                                     # 0xA0 = present, 0xA1 = absent
h.write_gpio_out(R)

# 2. READ ROM
h.tx_push_paced(SM, ow_xfer(8, READ_ROM))
h.wait_rx_ready(SM)
h.rx_pop(SM, R)                                        # (the bits sampled while writing: not used)

# 3. the 8 ROM bytes, one 8-bit read each
for _ in range(N_ROM_BYTES):
    h.tx_push_paced(SM, ow_xfer(8, 0xFF))
    h.wait_rx_ready(SM)
    h.rx_pop(SM, R)
    h.shift_right(R, 24)                               # 8 bits arrive in the top byte
    h.write_gpio_out(R)
h.halt()

image = h.build()
print(f"PIO 1-Wire flash image: {len(image)} bytes, {h.pages} pages, "
      f"{len(prog.instrs)}-word program, CLKDIV {CLKDIV}")
write_image(image, os.path.join(here, "pio_onewire_flash_image"))
print("Wrote pio_onewire_flash_image.bin and .hex")
