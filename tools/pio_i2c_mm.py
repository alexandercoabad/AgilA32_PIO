#!/usr/bin/env python3
"""pio_i2c_mm.py -- FIFO word builders for pio/i2c_mm.pio (multi-master I2C master).

Same bus conditions as pio_i2c.py, but every word is a full 32-bit record (autopull 24).
Data record layout (see the header of i2c_mm.pio):
    [31:26] 0      [25:10] 8 x {drive, tolerate-low}, bit 7 first      [9:8] {drive, NAK-ok}
Exec record:  first word n<<26 (n+1 instructions follow), then one word per instruction, instr<<16.

drive = 1 pulls the line low (hardware polarity).  Writing bit L: drive = 1-L, tolerate-low = drive.
Reading: drive = 0 (release) and tolerate-low = 1, because the slave legitimately pulls SDA low.
So `jmp pin`/`jmp x--` in the program flag arbitration lost exactly when a released write bit reads 0.

SM configuration (same pins as i2c.pio): out/set/in/jmp base = 8 (SDA), side-set base = 9 (SCL),
side_count 2 (opt + pindirs), OUT/IN shift left, autopull threshold 24, autopush threshold 8.
IRQ 0 rel = unexpected NAK, IRQ 1 rel = arbitration lost.
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble  # noqa: E402
from pio_host import shiftctrl  # noqa: E402
import pio_i2c as _base  # noqa: E402

_PIO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pio", "i2c_mm.pio")

SDA_PIN, SCL_PIN = 8, 9
PIN_OWN_MASK = (1 << SDA_PIN) | (1 << SCL_PIN)
IRQ_NAK, IRQ_LOST = 0, 1
MOV_ISR_NULL = assemble("mov isr, null").instrs[0]


def shift_ctrl(rx_enable):
    return shiftctrl(autopush=rx_enable, autopull=True, in_right=False, out_right=False,
                     push_thresh=8, pull_thresh=24)


SM_CONFIG = dict(out_base=SDA_PIN, out_count=1, set_base=SDA_PIN, set_count=1,
                 side_base=SCL_PIN, in_base=SDA_PIN, jmp_pin=SDA_PIN,
                 autopush=False, autopull=True, in_right=False, out_right=False,
                 push_thresh=8, pull_thresh=24)


def _exec_seq(seq16):
    """[n<<10, instr, instr, ...] from pio_i2c  ->  32-bit records for this program."""
    return [((seq16[0] >> 10) << 26)] + [(w & 0xFFFF) << 16 for w in seq16[1:]]


def _data(pairs, ack_drive, nak_ok):
    v = 0
    for d, t in pairs:
        v = (v << 2) | (d << 1) | t
    return [(v << 10) | (ack_drive << 9) | (nak_ok << 8)]


class I2cMM:
    def __init__(self, origin=0):
        self.origin = origin
        self.prog = assemble(open(_PIO).read(), origin=origin)
        self.entry = self.prog.labels["entry_point"] + origin

    # bus conditions -------------------------------------------------------------------
    @staticmethod
    def start():
        return _exec_seq(_base.start())

    @staticmethod
    def repstart():
        return _exec_seq(_base.repeated_start())

    @staticmethod
    def stop():
        return _exec_seq(_base.stop())

    # bytes -----------------------------------------------------------------------------
    @staticmethod
    def write_byte(b, nak_ok=False):
        """Send `b` with arbitration; release SDA in the ACK slot (NAK -> IRQ 0 unless nak_ok)."""
        pairs = []
        for i in range(7, -1, -1):
            d = 1 - ((b >> i) & 1)
            pairs.append((d, d))
        return _data(pairs, 0, 1 if nak_ok else 0)

    @staticmethod
    def read_byte(last=False):
        """Release SDA for 8 bits (slave drives), then ACK -- or NAK when `last`."""
        return _data([(0, 1)] * 8, 0 if last else 1, 1)

    def address(self, addr7, read=False):
        return self.write_byte(((addr7 & 0x7F) << 1) | (1 if read else 0))

    def write_transaction(self, addr7, data, stop=True):
        w = self.start() + self.address(addr7)
        for b in data:
            w += self.write_byte(b)
        return w + (self.stop() if stop else [])


if __name__ == "__main__":
    m = I2cMM()
    print("entry_point =", m.entry)
    for w in m.write_transaction(0x50, [0x12]):
        print("%08x" % w)
