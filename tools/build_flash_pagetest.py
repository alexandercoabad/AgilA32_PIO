#!/usr/bin/env python3
"""
Builds the 4-page bank-switching test image test/tb_flash_paging.v
expects (flash_pagetest.hex): page 0 writes 0x11 then switches to
page 1; page 1 writes 0x22, then uses a register-held counter and
switch_to_computed() to loop back to ITSELF once (writing 0x22 a
second time) before moving on to page 2; page 2 writes 0x33 then
switches to page 3; page 3 writes 0x44 and spins forever.

Expected GPIO_OUT (0xF0) write sequence, in order: 0x11, 0x22, 0x22,
0x33, 0x44 -- exercises both switch_to() (pages 0 and 2, a compile-
time-constant target) and switch_to_computed() (page 1's self-loop,
a runtime-decided target -- see asm_pineapple.py's PagedAsm docstring
for why that needs a convergent branch rather than an ordinary one).

Page 3 ends with its own explicit `JAL x0, SPIN` well within the
usable-code budget, rather than relying on finalize_last_page()'s
"falls back to restarting this page" default -- PagedAsm's own
docstring warns against relying on that fallback as the real terminal
behavior. Without the explicit spin loop, execution would fall through
into the reserved switch+trampoline area, write FLASH_PAGE=3 (itself),
and jump back to LOAD_BASE -- silently RE-EXECUTING page 3's own
ADDI+SW every pass, forever. That wouldn't just be redundant: each
re-execution is another observed GPIO_OUT write, so
test/tb_flash_paging.v's `seq_len == 5` check (it tracks every write,
not just distinct values, specifically so a second visit to page 1
looks different from a single one) would fail with far more than 5
writes recorded well before the test's cycle budget runs out.
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import PagedAsm

GPIO_OUT = 0xF0
FLASH_PAGE = 0xFC

p = PagedAsm(load_base=0xB4, window_bytes=44, flash_page_addr=FLASH_PAGE)

# --- Page 0: write 0x11, switch to page 1 ---
# x5 (page 1's self-loop counter) is explicitly zeroed here rather than
# assumed to already be 0 -- it IS the CPU's power-on-reset value, but
# by the time flash execution actually starts, the bootloader that got
# us here has already run and used x5 itself (as its RAM write pointer,
# see tools/build_boot_rom.py), leaving it nonzero. Found by running
# this: without the explicit zero, page 1's BNE took the "already
# looped" branch on its very first visit, producing only 4 GPIO_OUT
# writes (0x11,0x22,0x33,0x44) instead of the intended 5.
p.page.ADDI(5, 0, 0)
p.page.ADDI(2, 0, 0x11)
p.page.SW(2, 0, GPIO_OUT)
p.switch_to(1)

# --- Page 1: write 0x22, self-loop once via switch_to_computed(),
#     then move on to page 2 on the second visit. x5 (reset to 0,
#     like every register) is 0 on the first visit and set to 1
#     before looping back, so the second visit takes the other arm.
#     Both arms converge at the SAME instruction count before the
#     switch, per switch_to_computed()'s documented requirement. ---
p.page.ADDI(2, 0, 0x22)
p.page.SW(2, 0, GPIO_OUT)
p.page.BNE(5, 0, "GOTO_PAGE2")
p.page.ADDI(5, 0, 1)            # mark "already looped once"
p.page.ADDI(30, 0, 1)           # target = page 1 (self)
p.page.JAL(0, "CONVERGE")
p.page.label("GOTO_PAGE2")
p.page.ADDI(30, 0, 2)           # target = page 2
p.page.label("CONVERGE")
p.switch_to_computed()

# --- Page 2: write 0x33, switch to page 3 ---
p.page.ADDI(2, 0, 0x33)
p.page.SW(2, 0, GPIO_OUT)
p.switch_to(3)

# --- Page 3: write 0x44, spin forever (own explicit loop, see
#     docstring above for why this can't just fall through) ---
p.page.ADDI(2, 0, 0x44)
p.page.SW(2, 0, GPIO_OUT)
p.page.label("SPIN")
p.page.JAL(0, "SPIN")
p.finalize_last_page()

image = p.finalize()
assert len(image) == 4 * 44, f"expected exactly 4 pages (176 bytes), got {len(image)} bytes"
print(f"Flash pagetest image: {len(image)} bytes, {len(image)//44} pages")

with open('flash_pagetest.bin', 'wb') as f:
    f.write(image)
with open('flash_pagetest.hex', 'w') as f:
    for b in image:
        f.write(f"{b:02x}\n")
print("Wrote flash_pagetest.bin and flash_pagetest.hex")
