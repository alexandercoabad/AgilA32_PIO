#!/usr/bin/env python3
"""build_pio_uart.py -- flash image: the CPU loads the Pico-SDK `uart_tx` PIO
program, starts it, queues four bytes, then HALTS (EBREAK). The state machine
keeps transmitting on uo_out[0] with the CPU parked.

  PIO SM0, pin 0 = uo_out[0], CLKDIV = 16.0  ->  8 PIO ticks/bit x 16 = 128 clk/bit
  (1280 clocks per 8N1 frame; the CPU needs far less than that to queue all
  four bytes, so the last bytes go out *after* the core has halted).

Order matters: the pin is driven high (idle) by forced SET instructions BEFORE
PIN_OWN hands the pad to PIO, otherwise the line would glitch low for a moment
(a spurious start bit) because the PIO output register resets to 0.

Run from tools/:  python3 build_pio_uart.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL,
                      write_image)

MESSAGE = b"Agil"
SM = 0

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "uart_tx.pio")).read(), origin=0)

h = PioHost()
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=0, out_count=1, set_base=0, set_count=1,
                    side_base=0, side_count=2))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top, side_en=True))
h.write_reg(sm_reg(SM, SM_SHIFT), shiftctrl(out_right=True))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(16, 0))
h.force(SM, 0xE001)                    # set pins, 1      (line idle high)
h.force(SM, 0xE081)                    # set pindirs, 1
h.write_reg(R_PIN_OWN, 0x001)          # now hand uo_out[0] to PIO (already high: no glitch)
h.write_reg(R_CTRL, 1 << SM)           # enable SM0
h.tx_push(SM, *MESSAGE)
h.halt()

image = h.build()
print(f"PIO UART flash image: {len(image)} bytes, {h.pages} pages, message {MESSAGE!r}")
write_image(image, os.path.join(here, "pio_uart_flash_image"))
print("Wrote pio_uart_flash_image.bin and .hex")
