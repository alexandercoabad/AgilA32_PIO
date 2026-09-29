#!/usr/bin/env python3
"""Tiny synthetic program for test/tb_flash_handoff.v -- NOT meant to be
flashed onto real hardware. Deliberately small (4 instructions) so the
testbench that preloads it into the flash slave model can simulate in
a reasonable amount of time, unlike the real st7789_flash_image.py
(90+ instructions, most of it a 240*240-iteration pixel-fill loop).

Byte 0 of this image is a NOP and never actually executes -- see
build_flash_handoff_stub.py's "Why flash byte 0 is dead" for why: the
one-instruction handoff stub's own PC+4 lands on flash byte 4, not
byte 0, so byte 0 is unreachable by construction, not an oversight.

The instructions from byte 4 on are written so that whether execution
starts there correctly is externally, unambiguously observable on
uo_out:

  +0  (dead): ADDI x0, x0, 0    ; NOP -- see above
  +4        : ADDI x5, x0, 0x2A  ; x5 = 0x2A -- if flash execution
                somehow started one word further in than expected
                (repeating the exact bug this stub already had once),
                x5 would instead be whatever the bootloader's own
                RECV_BYTE left in a register (0, per its own exit
                condition -- see build_boot_rom.py), so the SW below
                would write 0x00 instead.
  +8        : SW   x5, 0(x0+0xF0) ; uo_out = x5
  +12       : JAL  x0, +12        ; spin forever (self-relative)

If the flash handoff is correct, uo_out ends up 0x2A. If it's
misaligned by one more word than expected, uo_out ends up 0x00.
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import Asm, words_to_bytes

GPIO_OUT = 0xF0
CANARY_VALUE = 0x2A

a = Asm()
a.ADDI(0, 0, 0)              # dead slot -- see module docstring
a.ADDI(5, 0, CANARY_VALUE)
a.SW(5, 0, GPIO_OUT)
a.label("SPIN")
a.JAL(0, "SPIN")

words = a.finalize()
prog_bytes = words_to_bytes(words)
print(f"Flash canary: {len(words)} instructions, {len(prog_bytes)} bytes, expect uo_out=0x{CANARY_VALUE:02x}")

with open('flash_canary.bin', 'wb') as f:
    f.write(prog_bytes)
with open('flash_canary.hex', 'w') as f:
    for b in prog_bytes:
        f.write(f"{b:02x}\n")
