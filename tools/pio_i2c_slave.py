#!/usr/bin/env python3
"""pio_i2c_slave.py -- host-side helpers for the two I2C *slave* programs.

    pio/i2c_slave_rx.pio   master WRITES to us: address match, ACK, exactly `write_bytes` data bytes
    pio/i2c_slave_tx.pio   master READS from us: address match, ACK, bytes from the TX FIFO, stretches
                           SCL while the FIFO is empty, stops at the master's NAK

Why two programs: a combined slave does not fit.  The state machines share ONE 32-word instruction
memory and each program here already uses 29 and 32 words.  Load whichever direction the device needs
(or reload at run time, as the CPU demos do).  The per-direction register-file pattern of a real
sensor/EEPROM (write pointer, then repeated START + read) needs both directions; see CHANGES_feature11.md.

Both programs: SDA = pad 8 (IN/OUT/SET base), SCL = pad 9 (JMP_PIN; SIDE-SET base in the TX program);
IN/OUT shift left; no autopush/autopull; PULL threshold 8 (the OSR is the hardware bit counter).

Init (before enabling the SM):   push the 7-bit address to the TX FIFO, force `pull block` and
`mov y, osr` (INIT_INSTRS), then force `jmp idle`.  RX words pushed by the RX program carry the byte in
bits [7:0]; mask with 0xFF (older bits of the ISR are not cleared).
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble  # noqa: E402

_PIO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pio")

SDA_PIN, SCL_PIN = 8, 9
PIN_OWN_MASK = (1 << SDA_PIN) | (1 << SCL_PIN)
PULL_BLOCK = assemble("pull block").instrs[0]
MOV_Y_OSR = assemble("mov y, osr").instrs[0]
INIT_INSTRS = [PULL_BLOCK, MOV_Y_OSR]          # forced, in this order, after pushing the address

_COMMON = dict(out_base=SDA_PIN, out_count=1, set_base=SDA_PIN, set_count=1, in_base=SDA_PIN,
               jmp_pin=SCL_PIN, autopush=False, autopull=False, in_right=False, out_right=False,
               push_thresh=8, pull_thresh=8)
RX_CONFIG = dict(_COMMON)
TX_CONFIG = dict(_COMMON, side_base=SCL_PIN)


def tx_word(byte):
    """TX FIFO word for one byte the slave sends: PINDIR polarity (1 = pull low), byte in the top 8 bits."""
    return ((~byte) & 0xFF) << 24


class I2cSlaveRx:
    """Write-direction slave.  `write_bytes` (1..29) data bytes are accepted per transaction."""

    def __init__(self, addr7, write_bytes=2, origin=0):
        assert 1 <= write_bytes <= 29
        text = open(os.path.join(_PIO, "i2c_slave_rx.pio")).read()
        text = re.sub(r"(\.define public WRITE_BYTES )\d+", r"\g<1>%d" % write_bytes, text)
        self.prog = assemble(text, origin=origin)
        self.addr7, self.write_bytes = addr7 & 0x7F, write_bytes
        self.idle = self.prog.labels["idle"] + origin
        self.config = RX_CONFIG


class I2cSlaveTx:
    """Read-direction slave; data comes from the TX FIFO (use tx_word())."""

    def __init__(self, addr7, origin=0):
        self.prog = assemble(open(os.path.join(_PIO, "i2c_slave_tx.pio")).read(), origin=origin)
        self.addr7 = addr7 & 0x7F
        self.idle = self.prog.labels["idle"] + origin
        self.config = TX_CONFIG


if __name__ == "__main__":
    for s in (I2cSlaveRx(0x50), I2cSlaveTx(0x50)):
        print(type(s).__name__, len(s.prog.instrs), "words, idle =", s.idle)
