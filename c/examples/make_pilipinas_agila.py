#!/usr/bin/env python3
"""Builds the PILIPINAS AGILA32 7-segment examples (44 bytes each, the size of the program window).

  pilipinas_agila.hex / pilipinas_agila_hardware.hex   for the GPIO bootloader (shift 12 for the simulator, 18 for hardware)
  pilipinas_agila_flash.bin / _flash_shift12.bin       for a flash chip on the QSPI Pmod: the boot ROM runs it by itself (shift 14: every instruction is fetched from flash, about 900 clocks per loop)
                                                       after its boot timeout, with no bootload (see c/README.md)

Run:  python3 make_pilipinas_agila.py        (no toolchain needed; the encodings are checked against the committed pilipinas.hex)
"""
import struct

ZERO, GP, T0, T1 = 0, 3, 5, 6
def addi(rd, rs1, imm): return ((imm & 0xFFF) << 20) | (rs1 << 15) | (rd << 7) | 0x13
def srli(rd, rs1, sh):  return (sh << 20) | (rs1 << 15) | (5 << 12) | (rd << 7) | 0x13
def andi(rd, rs1, imm): return ((imm & 0xFFF) << 20) | (rs1 << 15) | (7 << 12) | (rd << 7) | 0x13
def lbu(rd, imm, rs1):  return ((imm & 0xFFF) << 20) | (rs1 << 15) | (4 << 12) | (rd << 7) | 0x03
def sb(rs2, imm, rs1):  return (((imm >> 5) & 0x7F) << 25) | (rs2 << 20) | (rs1 << 15) | ((imm & 0x1F) << 7) | 0x23
def jal(rd, off):
    o = off & 0x1FFFFF
    return (((o >> 20) & 1) << 31) | (((o >> 1) & 0x3FF) << 21) | (((o >> 11) & 1) << 20) | (((o >> 12) & 0xFF) << 12) | (rd << 7) | 0x6F

# P I L I P I N A S A G I L A 3 2; bit 7 (decimal point) set on every second character
TABLE = bytes([0x73, 0xB0, 0x38, 0xB0, 0x73, 0xB0, 0x37, 0xF7, 0x6D, 0xF7, 0x3D, 0xB0, 0x38, 0xF7, 0x4F, 0xDB])

def program(shift, first):
    """first = the instruction at byte 0.  The loop starts at byte 4; the table is at byte 28 (address 0xB4 + 28 = 0xD0)."""
    words = [first,
             addi(GP, GP, 1),            # counter
             srli(T0, GP, shift),        # character number
             andi(T0, T0, 15),           # 16 characters
             lbu(T1, 0xD0, T0),          # pattern = table[t0]
             sb(T1, 0xF0, ZERO),         # LED_OUT = pattern
             jal(ZERO, -20)]             # back to byte 4
    b = b"".join(struct.pack("<I", w) for w in words) + TABLE
    assert len(b) == 44
    return b

MV_GP_ZERO = addi(GP, ZERO, 0)           # bootloaded version: start the counter at 0
SET_FAST_QSPI = sb(ZERO, 0xFB, ZERO)     # flash version: QSPI_CTRL <- 0 (reset value is sys_clk/128, far too slow to fetch from)

def hexs(b): return " ".join("%02x" % x for x in b) + "\n"

known = open("pilipinas.hex").read().split()
assert program(12, MV_GP_ZERO)[:28] == bytes.fromhex("".join(known[:28])), "encoders disagree with pilipinas.hex"

open("pilipinas_agila.hex", "w").write(hexs(program(12, MV_GP_ZERO)))
open("pilipinas_agila_hardware.hex", "w").write(hexs(program(18, MV_GP_ZERO)))
# Flash version: at the boot timeout the boot ROM leaves x3 (gp) = 1 and jumps to flash byte 0, so the counter does not need
# its own initialisation and byte 0 can set the QSPI clock instead.
open("pilipinas_agila_flash.bin", "wb").write(program(14, SET_FAST_QSPI))
open("pilipinas_agila_flash_shift12.bin", "wb").write(program(12, SET_FAST_QSPI))
print("ok")
