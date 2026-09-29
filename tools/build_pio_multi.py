#!/usr/bin/env python3
"""build_pio_multi.py -- ONE firmware image, ONE piece of silicon, THREE protocols in a row.

The CPU reprograms the same PIO state machine at run time (that is the whole point of a
protocol-emulator ASIC): load a different 2..18-instruction PIO program, reconfigure pins /
shifting / clock divider, start it, push data.

  phase 1  UART TX  (pio/uart_tx.pio)    pin 0 = uo_out[0]              "Agil", 8N1, 32 clk/bit
  phase 2  SPI mode 0 master (spi_master.pio)  MOSI = uo_out[0], SCK = uo_out[1]  0xA5 0x3C
  phase 3  I2C master (pio/i2c.pio)      SDA = uio[4], SCL = uio[5]     write 0x50: C3 96

Between phases the state machine is disabled, restarted, its FIFOs are cleared and instruction
memory is overwritten. The CPU waits (busy loop) for each protocol to finish before moving on,
then after queueing phase 3 it executes EBREAK: the STOP condition is produced by PIO alone.

Run from tools/:  python3 build_pio_multi.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)
import pio_i2c

SM = 0
UART_MSG = b"Agil"
SPI_MSG = [0xA5, 0x3C]
I2C_ADDR, I2C_DATA = 0x50, [0xC3, 0x96]

UART_CLKDIV, SPI_CLKDIV, I2C_CLKDIV = 4, 8, 32
CTRL_RESET_SM0 = (1 << 4) | (1 << 8) | (1 << 12)      # restart + clkdiv restart + FIFO clear, SM disabled

here = os.path.dirname(os.path.abspath(__file__))


def prog_of(name):
    return assemble(open(os.path.join(here, "..", "pio", name)).read(), origin=0)


def switch_to(h, prog, *, pins, shift, div, entry=None, jmp_pin=0):
    """Stop SM0, wipe its state, load `prog`, configure it. Does not enable it."""
    h.write_reg(R_CTRL, CTRL_RESET_SM0)
    h.load_program(prog)
    h.write_reg(sm_reg(SM, SM_PINCTRL), pins)
    h.write_reg(sm_reg(SM, SM_EXEC),
                execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                         side_en=prog.side_opt, side_pindir=prog.side_pindirs, jmp_pin=jmp_pin))
    h.write_reg(sm_reg(SM, SM_SHIFT), shift)
    h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(div, 0))
    h.force(SM, prog.origin + prog.labels[entry] if entry else prog.wrap_bottom)   # jmp entry


h = PioHost()

# ---------------------------------------------------------------- phase 1: UART TX
uart = prog_of("uart_tx.pio")
switch_to(h, uart, div=UART_CLKDIV,
          pins=pinctrl(out_base=0, out_count=1, set_base=0, set_count=1, side_base=0,
                       side_count=uart.side_bits),
          shift=shiftctrl(out_right=True))
h.force(SM, 0xE001)                    # set pins, 1  (idle high BEFORE PIO owns the pad: no glitch)
h.write_reg(R_PIN_OWN, 0x001)
h.write_reg(R_CTRL, 1 << SM)
h.tx_push_paced(SM, *UART_MSG)
h.delay(3 * 10 * 8 * UART_CLKDIV + 400)         # last frame = 10 bits x 8 ticks x CLKDIV, +margin

# ---------------------------------------------------------------- phase 2: SPI mode 0
spi = prog_of("spi_master.pio")
switch_to(h, spi, div=SPI_CLKDIV,
          pins=pinctrl(out_base=0, out_count=1, set_base=1, set_count=1, side_base=1,
                       side_count=spi.side_bits, in_base=2),
          shift=shiftctrl(autopull=True, pull_thresh=8, in_right=False, out_right=False))
h.force(SM, 0xE000)                    # set pins, 0  -> SCK low before PIO owns pin 1
h.write_reg(R_PIN_OWN, 0x003)
h.write_reg(R_CTRL, 1 << SM)
h.tx_push_paced(SM, *[b << 24 for b in SPI_MSG])
h.delay(len(SPI_MSG) * 8 * 4 * SPI_CLKDIV + 600)

# ---------------------------------------------------------------- phase 3: I2C write
i2c = prog_of("i2c.pio")
switch_to(h, i2c, div=I2C_CLKDIV, entry="entry_point", jmp_pin=8,
          pins=pinctrl(out_base=8, out_count=1, set_base=8, set_count=1, side_base=9,
                       side_count=i2c.side_bits, in_base=8),
          shift=shiftctrl(autopush=True, autopull=True, in_right=False, out_right=False,
                          push_thresh=8, pull_thresh=16))
h.write_reg(R_PIN_OWN, 0x303)          # uio[4]/uio[5] -> PIO, PINDIR = 0 (bus released)
h.write_reg(R_CTRL, 1 << SM)
h.tx_push_paced(SM, *[pio_i2c.fifo_word(w) for w in pio_i2c.write_transaction(I2C_ADDR, I2C_DATA)])
h.halt()

image = h.build()
print(f"PIO multi-protocol flash image: {len(image)} bytes, {h.pages} pages")
write_image(image, os.path.join(here, "pio_multi_flash_image"))
print("Wrote pio_multi_flash_image.bin and .hex")
