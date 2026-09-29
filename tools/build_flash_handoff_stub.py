#!/usr/bin/env python3
"""
Bootload-over-GPIO stub that hands off execution to external flash.

AgilA32's standard bootload path (see tools/build_boot_rom.py) writes
into the 0xB4-0xDF on-chip RAM window -- only 44 bytes (11 RV32I
instructions), nowhere near enough for a real LCD driver (even a
minimal bit-banged SPI byte-send loop alone needs more than that, see
build_st7789_flash_image.py's SEND_BYTE). FLASH_MODE (0xF8, see
mem.v) is the intended way around this: it redirects that SAME address
range to external flash (CS0) instead, which isn't limited by an
on-chip RAM budget -- so the full driver lives on flash (pre-
programmed there by some means outside this bootloader -- a standalone
SPI flash programmer, or whatever your Tiny Tapeout demo board
provides for flashing its QSPI Pmod), and this stub is the tiny thing
that's actually sent over the wire: it just flips FLASH_MODE and lets
the CPU's own natural next fetch pick up execution from flash.

This is deliberately the entire program -- one instruction, 4 bytes.
It MUST stay exactly one instruction; see "Why flash byte 0 is dead"
below for why a second bootloaded instruction wouldn't do what it
looks like it should.

FLASH_MODE takes effect the instant the write commits (a synchronous
register, visible starting the very next cycle) -- the CPU's normal
PC+4 lands on LOAD_BASE+4, and THAT fetch is already redirected to
external flash. Confirmed in simulation, see test/tb_flash_handoff.v.

### Why flash byte 0 is dead (don't try to "fix" this with a 2nd
    instruction -- it was tried, see below)

You'd expect the flash image's own byte 0 to map to chip address
LOAD_BASE (the same "byte 0 of the image is chip address LOAD_BASE"
framing build_st7789_flash_image.py's header uses) and for THIS stub
to jump there explicitly after setting FLASH_MODE, landing cleanly on
flash byte 0. That was tried -- a 2-instruction version (SW
FLASH_MODE; JAL LOAD_BASE) -- and it does NOT reach flash byte 0
either. FLASH_MODE's redirect isn't scoped to "future bootloads" or
"addresses beyond this stub" -- it's a flat address-range redirect
that takes effect for literally the next fetch, and LOAD_BASE+4 (where
the JAL itself was bootloaded, still within LOAD_BASE-0xDF) IS that
next fetch. So the JAL is never actually executed from where it was
loaded -- the fetch that should retrieve it instead comes back with
flash byte 4 instead (misinterpreted as the JAL), producing whatever
garbage that byte happens to decode as. Confirmed in simulation with a
canary program (test/tb_flash_handoff.v + tools/build_flash_canary.py)
whose byte 0 has an obviously-wrong effect if skipped: with the
2-instruction stub, execution landed on the canary's *second*
instruction, using a leftover bootloader register value instead of the
one the (never-executed) first instruction would have set -- same
symptom, different register, as this file's very first version being
JUST the FLASH_MODE write with no jump at all.

The one-instruction stub sidesteps this entirely: there's no second
bootloaded instruction to preempt, so PC+4 (LOAD_BASE+4) is simply
where flash execution starts. The tradeoff is that flash byte 0 is
*never* fetched by any instruction stream -- see
build_st7789_flash_image.py and build_flash_canary.py, which both
reserve their own first word as an explicit NOP (ADDI x0,x0,0) rather
than leave a real instruction sitting somewhere it can't be reached.
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import Asm, words_to_bytes

FLASH_MODE = 0xF8

a = Asm()
a.SW(0, 0, FLASH_MODE)   # write-any-value-to-set; x0 is fine as the value

words = a.finalize()
prog_bytes = words_to_bytes(words)
print(f"Flash-handoff stub: {len(words)} instruction, {len(prog_bytes)} bytes "
      f"(bootload budget: 44 bytes / 11 instructions)")
assert len(prog_bytes) <= 44

with open('flash_handoff_stub.bin', 'wb') as f:
    f.write(prog_bytes)
with open('flash_handoff_stub.hex', 'w') as f:
    for b in prog_bytes:
        f.write(f"{b:02x}\n")
print("Wrote flash_handoff_stub.bin and .hex")
