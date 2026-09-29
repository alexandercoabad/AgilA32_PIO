#!/usr/bin/env python3
"""
Standalone verification program for every newly-added asm_pineapple.py
instruction (SUB, SLL/SRL/SRA, SLT/SLTU, SLTI/SLTIU, XORI, SRAI,
LB/LH/LHU, SH, BGE/BLTU/BGEU, LUI/AUIPC) -- not trusted on paper, run
through the real rv32i_core in test/tb_alu_test.v and checked against
hand-computed expected values via direct register-file access.

Test values are deliberately chosen so signed and unsigned
interpretations of the SAME bit pattern disagree (-1 == 0xFFFFFFFF is
used throughout for exactly this reason) -- a wrapper with the wrong
funct3/funct7, or an ALU op the RTL implements differently than
expected, is far more likely to be caught by a value that flips sign
than by small positive numbers where signed and unsigned agree.

RAM_BASE (0xC0) is this test's own on-chip scratch address, placed
right after the 192-byte ROM range test/alu_test_mem.v gives this
program -- unrelated to mem.v's actual RAM_BASE (0xB0) or address map;
test/alu_test_mem.v is a minimal flat ROM+RAM harness, not mem.v, so
there's no collision with anything this project's real address map
uses.
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import Asm, words_to_bytes

RAM_BASE = 0xC0

a = Asm()

# --- SUB ---
a.ADDI(1, 0, 10)
a.ADDI(2, 0, 3)
a.SUB(3, 1, 2)                  # x3 = 7

# --- SLL ---
a.ADDI(1, 0, 1)
a.ADDI(2, 0, 4)
a.SLL(4, 1, 2)                  # x4 = 16

# --- SLT / SLTU (signed vs unsigned disagree on -1 vs 1) ---
a.ADDI(1, 0, -1)
a.ADDI(2, 0, 1)
a.SLT(5, 1, 2)                  # x5 = 1  (signed -1 < 1)
a.SLTU(6, 1, 2)                 # x6 = 0  (unsigned 0xFFFFFFFF < 1 is false)

# --- SRL / SRA (logical vs arithmetic disagree on a negative value) ---
a.LUI(1, 0x80000)               # x1 = 0x80000000
a.ADDI(2, 0, 4)
a.SRL(7, 1, 2)                  # x7 = 0x08000000 (zero-filled)
a.SRA(8, 1, 2)                  # x8 = 0xF8000000 (sign-filled)

# --- SLTI / SLTIU ---
a.ADDI(1, 0, -5)
a.SLTI(9, 1, 0)                 # x9  = 1 (signed -5 < 0)
a.SLTIU(10, 1, 1)                # x10 = 0 (unsigned huge < 1 is false)

# --- XORI ---
a.ADDI(1, 0, 0xFF)
a.XORI(11, 1, 0x0F)             # x11 = 0xF0

# --- SRAI (immediate-form arithmetic shift, same value SRA used) ---
a.LUI(1, 0x80000)               # x1 = 0x80000000
a.SRAI(12, 1, 4)                 # x12 = 0xF8000000

# --- SH / LH / LHU: store 0xFFFF, read back sign- and zero-extended ---
a.ADDI(1, 0, RAM_BASE)
a.ADDI(2, 0, -1)
a.SH(2, 1, 0)
a.LH(13, 1, 0)                   # x13 = 0xFFFFFFFF (sign-extended)
a.LHU(14, 1, 0)                  # x14 = 0x0000FFFF (zero-extended)

# --- SB / LB / LBU: store 0xFF, read back sign- and zero-extended ---
a.ADDI(2, 0, -1)
a.SB(2, 1, 4)
a.LB(15, 1, 4)                    # x15 = 0xFFFFFFFF (sign-extended)
a.LBU(16, 1, 4)                   # x16 = 0x000000FF (zero-extended)

# --- BGE: signed -1 >= 1 is false -> branch NOT taken ---
a.ADDI(1, 0, -1)
a.ADDI(2, 0, 1)
a.ADDI(17, 0, 0)
a.BGE(1, 2, "SKIP1")
a.ADDI(17, 0, 111)               # runs iff BGE correctly did NOT branch
a.label("SKIP1")

# --- BLTU: unsigned 0xFFFFFFFF < 1 is false -> branch NOT taken ---
a.ADDI(18, 0, 0)
a.BLTU(1, 2, "SKIP2")
a.ADDI(18, 0, 222)                # runs iff BLTU correctly did NOT branch
a.label("SKIP2")

# --- BGEU: unsigned 0xFFFFFFFF >= 1 is true -> branch IS taken (skips
#     the next instruction; landing value proves it was actually
#     skipped, not just coincidentally equal) ---
a.ADDI(19, 0, 0)
a.BGEU(1, 2, "SKIP3")
a.ADDI(19, 0, 999)                 # must NOT run if BGEU branches correctly
a.label("SKIP3")
a.ADDI(19, 19, 333)                 # x19 = 333 if BGEU worked, 1332 if it didn't

# --- AUIPC ---
a.label("AUIPC_TEST")
a.AUIPC(20, 0)                       # x20 = this instruction's own address

a.label("SPIN")
a.JAL(0, "SPIN")

words = a.finalize()
prog_bytes = words_to_bytes(words)
print(f"ALU test program: {len(words)} instructions, {len(prog_bytes)} bytes")
print(f"  AUIPC_TEST address: 0x{a.labels['AUIPC_TEST']:02x}")

ROM_BYTES = 192  # must match test/alu_test_mem.v's ROM_BYTES
assert len(prog_bytes) <= ROM_BYTES
padded = prog_bytes + bytes(ROM_BYTES - len(prog_bytes))  # pad so
# $readmemh's target array size and this file's word count match
# exactly -- the padding bytes are never fetched (the program parks in
# an infinite loop well before reaching them), this is purely to avoid
# a benign-but-noisy "not enough words in the file" simulator warning.

with open('alu_test.bin', 'wb') as f:
    f.write(prog_bytes)
with open('alu_test.hex', 'w') as f:
    for b in padded:
        f.write(f"{b:02x}\n")
print("Wrote alu_test.bin and alu_test.hex")
