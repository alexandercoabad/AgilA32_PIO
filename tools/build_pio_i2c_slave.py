#!/usr/bin/env python3
"""build_pio_i2c_slave.py -- the CPU turns the PIO into an I2C slave, then into a different one.

One flash image, one state machine, an external I2C master (the testbench):

  phase 1  i2c_slave_rx   (address 0x42, 2 data bytes)   the master WRITES  A5 3C   to us
           the CPU polls the RX FIFO and reads both bytes (the PIO did every bit: START detect,
           address compare, ACK, shifting)
  phase 2  the CPU computes byte + 1 for each, swaps in i2c_slave_tx (the two slave programs use
           29 + 32 of the 32 instruction words, so they cannot be loaded together), queues
           A6 3D, and HALTS (EBREAK).
           The master then READS two bytes from 0x42 and gets A6 3D, answered by PIO alone.

This is the pattern of a real register-file device (write the data, read it back) built from the
two direction-specific programs: the CPU does the bookkeeping the PIO cannot, between the
phases. SDA = uio[4], SCL = uio[5]; CLKDIV 1 (the master's SCL period must be >= 40 clocks).

Run from tools/:  python3 build_pio_i2c_slave.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)
from pio_i2c_slave import I2cSlaveRx, I2cSlaveTx, PIN_OWN_MASK, INIT_INSTRS

SM = 0
ADDR7 = 0x42
CTRL_RESET_SM0 = (1 << 4) | (1 << 8) | (1 << 12)       # restart + clkdiv restart + FIFO clear, SM0 disabled
SET_PINDIRS_0 = 0xE080                                  # set pindirs, 0  (release SDA)


def bring_up(h, slave):
    """Wipe SM0, load `slave`, configure it, hand it the pads and its own address. Not enabled."""
    prog, cfg = slave.prog, slave.config
    h.write_reg(R_CTRL, CTRL_RESET_SM0)
    h.load_program(prog)
    h.write_reg(sm_reg(SM, SM_PINCTRL),
                pinctrl(out_base=cfg["out_base"], out_count=cfg["out_count"],
                        set_base=cfg["set_base"], set_count=cfg["set_count"],
                        side_base=cfg.get("side_base", 0), side_count=prog.side_bits,
                        in_base=cfg["in_base"]))
    h.write_reg(sm_reg(SM, SM_EXEC),
                execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                         side_en=prog.side_opt, side_pindir=prog.side_pindirs,
                         jmp_pin=cfg["jmp_pin"]))
    h.write_reg(sm_reg(SM, SM_SHIFT),
                shiftctrl(autopush=cfg["autopush"], autopull=cfg["autopull"],
                          in_right=cfg["in_right"], out_right=cfg["out_right"],
                          push_thresh=cfg["push_thresh"], pull_thresh=cfg["pull_thresh"]))
    h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(1, 0))
    h.force(SM, SET_PINDIRS_0)             # SDA released before the pads change hands
    h.write_reg(R_PIN_OWN, PIN_OWN_MASK)
    h.tx_push(SM, slave.addr7)             # own address -> Y (forced pull + mov y, osr)
    for ins in INIT_INSTRS:
        h.force(SM, ins)
    h.force(SM, slave.idle)                # jmp idle


rx = I2cSlaveRx(ADDR7, write_bytes=2)
tx = I2cSlaveTx(ADDR7)

h = PioHost()

# ---------------------------------------------------------------- phase 1: write slave
bring_up(h, rx)
h.write_reg(R_CTRL, 1 << SM)               # enable: from here PIO answers the master on its own
h.wait_rx_ready(SM)
h.rx_get(SM, 10)                           # x10 <- first data byte
h.wait_rx_ready(SM)
h.rx_get(SM, 11)                           # x11 <- second data byte
h.byte_plus1_to_i2c_slave_word(10)         # CPU work: (byte + 1) as a TX word
h.byte_plus1_to_i2c_slave_word(11)
h.delay(6000)                              # let the master finish the last ACK and the STOP

# ---------------------------------------------------------------- phase 2: read slave
bring_up(h, tx)
h.write_idx(sm_reg(SM, 6))                 # SM_TXF
h.write_data_reg(10)
h.write_data_reg(11)
h.write_reg(R_CTRL, 1 << SM)
h.halt()                                   # CPU parked; PIO serves the master's read

image = h.build()
print(f"PIO I2C slave flash image: {len(image)} bytes, {h.pages} pages; "
      f"RX slave {len(rx.prog.instrs)} words, TX slave {len(tx.prog.instrs)} words")
write_image(image, os.path.join(here := os.path.dirname(os.path.abspath(__file__)), "pio_i2c_slave_flash_image"))
print("Wrote pio_i2c_slave_flash_image.bin and .hex")
