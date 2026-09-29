#!/usr/bin/env python3
"""
ST7789 driver for AgilA32, meant to be pre-programmed onto the
external flash chip on the QSPI Pmod (CS0) and reached via
build_flash_handoff_stub.py's tiny bootloaded stub -- see that file's
docstring for the handoff mechanism, and mem.v's "Reprogrammability" /
"Bank-switched flash execution" sections for FLASH_MODE/FLASH_PAGE.

REWRITTEN to actually use tools/asm_pineapple.py's PagedAsm -- the
previous version of this file used plain Asm() (a flat, single-page
program), which silently does NOT work: this driver (SEND_BYTE alone
is 14 instructions/56 bytes) is far bigger than the single 44-byte
LOAD_BASE-0xDF window FLASH_MODE exposes. Once PC walked past 0xDF it
would start fetching from the PSRAM window (0xE0-0xEF) instead of the
rest of this program -- wrong instructions, not a clean failure. This
was never caught because nothing had exercised bank-switching with a
program anywhere near this size before. See PagedAsm's own docstring
in asm_pineapple.py for the page-layout mechanics this now relies on.

Two page-friendly patterns replace the old single flat program:

1. SEND_BYTE became per-BIT pages, not a callable subroutine. A page
   switch can only ever land at LOAD_BASE of the target page (never
   at an arbitrary offset within it), so a subroutine living in one
   page can't be JAL'd-into from a DIFFERENT page and JALR-returned-
   from the way the old flat version's SEND_BYTE was called from a
   dozen different call sites -- there's no "return to caller's page"
   concept FLASH_PAGE supports. Every byte sent by this driver is a
   compile-time Python constant (command opcodes, fixed init args,
   and the single FILL_COLOR), so emit_byte_pages() below just
   precomputes the exact GPIO_OUT value for each of the 8 bits at
   BUILD time and emits a trivial ADDI+SW/ADDI+SW (4 instructions)
   per bit-page -- no runtime shift-register or bit-counter needed at
   all, which is what makes each bit fit comfortably inside one
   page's 8-instruction usable budget (with headroom to spare).

2. The pixel fill can't afford one page per bit at real panel sizes
   (240x240x16bpp bit-banged one bit per page-switch would be roughly
   3.7 million pages, i.e. tens of MB of flash for one color) -- but
   it doesn't need to, because every pixel sends the IDENTICAL 16
   bits (FILL_COLOR never changes). So there's a fixed ~18-page RING
   built ONCE (16 bit-send pages + 1 counter/control page using the
   NEW switch_to_computed(), see below), and the control page loops
   back to the ring's first page or falls through to whatever's next
   based on a runtime pixel counter in a register -- reused
   PANEL_WIDTH*PANEL_HEIGHT times via ordinary FLASH_PAGE cycling,
   not re-emitted per pixel. Flash size for the fill is therefore
   fixed (~18 pages) regardless of panel size; only the counter's
   initial value (a single ADDI-loadable-ish 32-bit constant, see
   load_const32) scales with pixel count.

   This is exactly the gap tools/asm_pineapple.py's switch_to()
   couldn't cover on its own: it only ever emits a compile-time-
   constant page target. A real bulk-transfer loop's exit condition
   is a runtime value (the pixel counter reaching 0), so the control
   page computes the next-page number itself (a two-armed branch,
   both arms loading the SAME scratch register, converging before the
   switch) and hands it to the new switch_to_computed(), which emits
   just the SW + trampoline, trusting the caller already loaded the
   register. See that method's own docstring in asm_pineapple.py for
   the convergent-branch requirement this depends on.

Verified in test/tb_st7789_driver.v against a small scaled-down panel
(see SIM_PANEL_WIDTH/HEIGHT below) by capturing every bit-banged SCK
rising edge and reconstructing the actual command/data byte stream,
not just checking a final register value -- confirms the full init
sequence AND at least one full trip around the fill ring (including
the loop-back decision) produce the exact expected byte sequence, in
the correct DC (command/data) state, in order. Scaling PANEL_WIDTH/
PANEL_HEIGHT back up to a real panel's actual size for hardware use
changes nothing about the ring itself, only the counter's starting
value -- see USE_SIM_PANEL_SIZE below to switch between the two.

Hardware assumptions (confirm before flashing) -- unchanged from the
previous version:
  - Panel wiring, bit-banged over GPIO_OUT (0xF0) since AgilA32 has
    no dedicated SPI peripheral (unlike AgilA8's CS2 engine):
      GPIO_OUT[0] = SCK
      GPIO_OUT[1] = MOSI
      GPIO_OUT[2] = DC
      GPIO_OUT[3] = CS -- set once at the very start and held low for
                   the program's entire lifetime (single-device bus,
                   no other peripheral to arbitrate with), NOT pulsed
                   per byte or per transaction.
  - RST tied high externally -- this program never drives a reset
    pin, relying on SWRESET over SPI instead.
  - Flash itself is read-only from this design's memory port, so this
    program can never modify itself; any scratch state it needs lives
    in registers only (no on-chip RAM is reachable once FLASH_MODE has
    redirected LOAD_BASE-0xDF away from it).
  - SPI mode 0 (CPOL=0, CPHA=0), sample on SCK's rising edge. Each
    bit's SW that sets up the NEXT bit's MOSI/DC also explicitly sets
    SCK=0 -- this implicitly lowers SCK back down after the previous
    bit's sample pulse without a dedicated "settle" write in between,
    which is functionally identical for a mode-0 slave (setup/hold
    around the rising edge is unaffected by exactly how long the
    PRIOR high phase lasted) and is what keeps each bit-page down to
    4 instructions. The very last bit of the very last byte leaves
    SCK high until DELAY's own first write (or, for the last thing
    this program ever does, forever, since there's nothing after the
    fill loop but SPIN) -- harmless since nothing else shares this
    bus and no further transaction is expected once the panel is lit.
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import PagedAsm

GPIO_OUT = 0xF0
FLASH_PAGE = 0xFC

# Real hardware target.
PANEL_WIDTH = 240
PANEL_HEIGHT = 240
MADCTL_VALUE = 0x00
FILL_COLOR = 0xF800   # RGB565; e.g. 0xF800=red, 0x07E0=green, 0x001F=blue
DELAY_COUNT = 2_000_000  # tune by eye; see DELAY_PAGE below

# Simulation target -- tb_st7789_driver.v uses THIS size (a full
# 240x240 fill is 57,600 pixels, not something worth spending real
# simulation wall-clock time on bit by bit). The ring/control-page
# logic is identical either way; only this constant changes.
USE_SIM_PANEL_SIZE = True
SIM_PIXEL_COUNT = 3       # small enough to see init + a full loop-back
SIM_DELAY_COUNT = 20

pixel_count = SIM_PIXEL_COUNT if USE_SIM_PANEL_SIZE else (PANEL_WIDTH * PANEL_HEIGHT)
delay_count = SIM_DELAY_COUNT if USE_SIM_PANEL_SIZE else DELAY_COUNT

DC_CMD = 0
DC_DATA = 1 << 2

p = PagedAsm(load_base=0xB4, window_bytes=44, flash_page_addr=FLASH_PAGE)


def emit_byte_pages(byte_val, dc_bit, next_page_after):
    """Appends 8 pages (one per bit, MSB first) bit-banging the
    compile-time-constant `byte_val` over GPIO_OUT with DC held at
    `dc_bit` throughout. The 8th bit's page switches to
    `next_page_after` (a plain page-number constant -- every call site
    in this file knows its target at build time, so plain switch_to()
    is enough here; only the fill ring's control page needs
    switch_to_computed())."""
    for i in range(8):
        bit = (byte_val >> (7 - i)) & 1
        mosi = bit << 1
        setup_val = dc_bit | mosi        # SCK=0
        pulse_val = setup_val | 1         # SCK=1
        p.page.ADDI(2, 0, setup_val)
        p.page.SW(2, 0, GPIO_OUT)
        p.page.ADDI(2, 0, pulse_val)
        p.page.SW(2, 0, GPIO_OUT)
        if i == 7:
            p.switch_to(next_page_after)
        else:
            p.switch_to(p._cur_page_num + 1)


def emit_cmd(byte_val, next_page_after):
    emit_byte_pages(byte_val, DC_CMD, next_page_after)


def emit_data(byte_val, next_page_after):
    emit_byte_pages(byte_val, DC_DATA, next_page_after)


def reserve_page(want):
    """Asserts the next page built will be page `want` -- a cheap
    sanity check that this script's own page-numbering bookkeeping
    (every emit_*/switch_to call site below hardcodes its target page
    number) hasn't drifted out of sync with actual page allocation."""
    assert p._cur_page_num == want, (
        f"page numbering drifted: expected to be building page {want}, "
        f"actually about to build page {p._cur_page_num} -- a call site "
        f"above disagrees with actual page count, fix the hardcoded "
        f"target page numbers to match"
    )


def load_const32(rd, value, scratch):
    """Loads an arbitrary (<2**24) 32-bit constant into rd using
    scratch as a temporary: ADDI rd,x0,low12 ; ADDI scratch,x0,hi ;
    SLLI scratch,scratch,12 ; ADD rd,rd,scratch. Both DELAY_COUNT and
    a real panel's pixel count (max 57,600 for 240x240) fit under
    2**24 with room to spare."""
    assert 0 <= value < (1 << 24), f"load_const32 only handles <2**24 here, got {value}"
    lo = value & 0xFFF
    hi = value >> 12
    p.page.ADDI(rd, 0, lo)
    if hi:
        p.page.ADDI(scratch, 0, hi)
        p.page.SLLI(scratch, scratch, 12)
        p.page.ADD(rd, rd, scratch)


# --- Page 0: CS low (held for the rest of the program), pixel counter init ---
reserve_page(0)
p.page.ADDI(2, 0, 0)      # x2 = 0 (SCK=0,MOSI=0,DC=0,CS=0 all at once)
p.page.SW(2, 0, GPIO_OUT)
load_const32(5, pixel_count, scratch=8)   # x5 = pixel count (fill ring's counter)
p.switch_to(1)

# --- SWRESET (0x01) ---
reserve_page(1)
emit_cmd(0x01, next_page_after=9)   # 8 bit-pages: 1..8, then -> 9

# --- DELAY page (self-contained in-page loop, no page-switch needed
# to run it -- only to enter/leave it) ---
reserve_page(9)
p.page.ADDI(2, 0, delay_count)
p.page.label("DELAY_LOOP")
p.page.BEQ(2, 0, "DELAY_DONE")
p.page.ADDI(2, 2, -1)
p.page.JAL(0, "DELAY_LOOP")
p.page.label("DELAY_DONE")
p.switch_to(10)

# --- SLPOUT (0x11) ---
reserve_page(10)
emit_cmd(0x11, next_page_after=18)

# --- 2nd DELAY ---
reserve_page(18)
p.page.ADDI(2, 0, delay_count)
p.page.label("DELAY_LOOP")
p.page.BEQ(2, 0, "DELAY_DONE")
p.page.ADDI(2, 2, -1)
p.page.JAL(0, "DELAY_LOOP")
p.page.label("DELAY_DONE")
p.switch_to(19)

# --- COLMOD (0x3A) = 0x05 ---
reserve_page(19)
emit_cmd(0x3A, next_page_after=27)
reserve_page(27)
emit_data(0x05, next_page_after=35)

# --- MADCTL (0x36) = MADCTL_VALUE ---
reserve_page(35)
emit_cmd(0x36, next_page_after=43)
reserve_page(43)
emit_data(MADCTL_VALUE, next_page_after=51)

# --- CASET (0x2A): 0..(PANEL_WIDTH-1) ---
reserve_page(51)
emit_cmd(0x2A, next_page_after=59)
reserve_page(59)
emit_data(0, next_page_after=67)                          # xs hi
reserve_page(67)
emit_data(0, next_page_after=75)                          # xs lo
reserve_page(75)
emit_data((PANEL_WIDTH - 1) >> 8, next_page_after=83)      # xe hi
reserve_page(83)
emit_data((PANEL_WIDTH - 1) & 0xFF, next_page_after=91)    # xe lo

# --- RASET (0x2B): 0..(PANEL_HEIGHT-1) ---
reserve_page(91)
emit_cmd(0x2B, next_page_after=99)
reserve_page(99)
emit_data(0, next_page_after=107)                          # ys hi
reserve_page(107)
emit_data(0, next_page_after=115)                          # ys lo
reserve_page(115)
emit_data((PANEL_HEIGHT - 1) >> 8, next_page_after=123)     # ye hi
reserve_page(123)
emit_data((PANEL_HEIGHT - 1) & 0xFF, next_page_after=131)   # ye lo

# --- DISPON (0x29) ---
reserve_page(131)
emit_cmd(0x29, next_page_after=139)

# --- RAMWR (0x2C) ---
reserve_page(139)
emit_cmd(0x2C, next_page_after=147)

# --- Fill ring: 16 bit-send pages (color hi byte then lo byte, both
# compile-time constants) + 1 control page, reused for every pixel via
# switch_to_computed(). Page 147 = ring start. ---
reserve_page(147)
color_bytes = [(FILL_COLOR >> 8) & 0xFF, FILL_COLOR & 0xFF]
RING_START = 147
RING_LEN = 16       # 8 bits * 2 bytes
CTRL_PAGE = RING_START + RING_LEN     # 163
DONE_PAGE = CTRL_PAGE + 1              # 164

bit_num = 0
for byte_val in color_bytes:
    for i in range(8):
        bit = (byte_val >> (7 - i)) & 1
        mosi = bit << 1
        setup_val = DC_DATA | mosi
        pulse_val = setup_val | 1
        p.page.ADDI(2, 0, setup_val)
        p.page.SW(2, 0, GPIO_OUT)
        p.page.ADDI(2, 0, pulse_val)
        p.page.SW(2, 0, GPIO_OUT)
        bit_num += 1
        if bit_num == RING_LEN:
            p.switch_to(CTRL_PAGE)
        else:
            p.switch_to(p._cur_page_num + 1)

# --- Control page: runtime-conditional loop-back vs done ---
reserve_page(CTRL_PAGE)
p.page.ADDI(5, 5, -1)
p.page.BNE(5, 0, "NOT_DONE")
p.page.ADDI(30, 0, DONE_PAGE)
p.page.JAL(0, "CONVERGE")
p.page.label("NOT_DONE")
p.page.ADDI(30, 0, RING_START)
p.page.label("CONVERGE")
p.switch_to_computed()

# --- Done: nothing left to fall through to (no HALT in this ISA) ---
reserve_page(DONE_PAGE)
p.page.label("SPIN")
p.page.JAL(0, "SPIN")
p.finalize_last_page()

words_note = None  # (this file builds pages directly, not via a flat Asm())
image = p.finalize()
print(f"ST7789 flash image: {len(image)} bytes, {len(image)//44} pages "
      f"(pixel_count={pixel_count}, delay_count={delay_count}, "
      f"{'SIMULATION' if USE_SIM_PANEL_SIZE else 'REAL PANEL'} size)")
print(f"  fill ring: pages {RING_START}-{RING_START+RING_LEN-1}, "
      f"control page {CTRL_PAGE}, done page {DONE_PAGE}")

with open('st7789_flash_image.bin', 'wb') as f:
    f.write(image)
with open('st7789_flash_image.hex', 'w') as f:
    for b in image:
        f.write(f"{b:02x}\n")
print("Wrote st7789_flash_image.bin and .hex")
