#!/usr/bin/env python3
"""
PS/2 keyboard reader, Step 2: scancode -> ASCII translation.

Builds on Step 1 (tools/build_ps2_reader.py) unchanged for the actual
bit-level PS/2 frame receive -- pages 0-22 below are byte-for-byte the
same wait_clock()/wait_clock_low_and_sample() state machine, same
register conventions (x1=GPIO_IN snapshot, x2=CLOCK bit, x3=DATA bit,
x4=accumulating scancode). Duplicated rather than imported: PagedAsm
builds one flat page sequence per script run, there's no mechanism to
splice two scripts' pages together, so every *_flash_image.py file in
this project is self-contained the same way.

What's new is everything AFTER a full scancode byte lands in x4,
replacing Step 1's RESULT_PAGE (which just did SW x4,GPIO_OUT
unconditionally, every frame, including break-code and prefix bytes)
with real PS/2 framing awareness and a scancode -> ASCII table.

### PS/2 framing this now handles

Scan Code Set 2 (what essentially every PS/2 keyboard speaks) frames
key EVENTS, not just key IDENTITIES, as a short byte sequence:
  - plain make code (key pressed): the scancode byte itself, e.g. 0x1C
  - break code (key released): 0xF0 followed by the SAME scancode byte
  - extended make code (e.g. right-ctrl, arrow keys): 0xE0 followed by
    the scancode byte
  - extended break code: 0xE0 0xF0 followed by the scancode byte

Step 1 treated every received byte as a standalone scancode and just
echoed it -- so releasing a key produced TWO GPIO_OUT writes (0xF0,
then that key's own code again), and extended keys produced garbage
translations. Step 2 tracks this with two persistent flag registers
(x5=break_pending, x6=ext_pending -- both survive FLASH_PAGE switches
exactly like x4 already does, see mem.v's "Bank-switched flash
execution") and only ever emits a translated ASCII byte for a genuine
plain make code:

  byte received (in x4), current (break_pending, ext_pending):
    break_pending==1  -> this byte is the released key's own code
                          (base OR extended, don't care which for
                          Step 2) -- clear BOTH flags, no emit
    byte==0xF0        -> set break_pending=1 (leave ext_pending as-is,
                          so E0 F0 xx is tracked correctly as an
                          extended break, not confused with a plain
                          one), no emit
    byte==0xE0        -> set ext_pending=1, no emit
    ext_pending==1     -> this byte is an extended make code -- Step 2
                          has no ASCII mapping for any of them (arrow
                          keys, right-ctrl, etc. aren't ASCII), clear
                          ext_pending, no emit
    else               -> plain make code: look up ASCII, emit it
                          (0x00 for anything not in the table below --
                          function keys, Escape, modifier keys like
                          Shift/Ctrl/Alt/CapsLock themselves, etc.)

Deliberately NOT handled (later-step scope, same "prove the mechanism
first" philosophy Step 1 used for parity/stop-bit values): Shift or
CapsLock state -- every letter maps to its lowercase form and every
shifted-symbol key (e.g. the digit row's !@#$...) maps to its
UNSHIFTED character only, regardless of what's actually held down.
Modifier keys themselves (Shift/Ctrl/Alt/CapsLock/the Windows key)
have no table entry and translate to 0x00, same as any other unmapped
key -- this program doesn't track them as state at all yet.

### How the table lookup actually works: FLASH_PAGE-indexed dispatch,
    not a data table in memory

There's no runtime-addressable byte array this design can put a
256-entry lookup table into -- on-chip RAM is 4 bytes once flash
execution owns LOAD_BASE-0xDF (see mem.v's header), and indexing
directly into flash with an arbitrary computed address isn't a thing
FLASH_PAGE supports (it selects which WINDOW_BYTES-sized SLICE shows
up at the fixed chip-address window, not a byte offset within a flat
address space).

What IS directly available: switch_to_computed() (see
asm_pineapple.py) can jump to a page number computed at runtime. So
this uses TABLE_START + the scancode value as the page number:
PROCESS_DISPATCH below bounds-checks that against FLASH_PAGE's 8-bit
range (see "Page budget notes" below -- this bounds check is NOT
optional, an earlier version without it had a real wraparound bug)
and then switch_to_computed()s there -- landing on one of
MAX_SAFE_SCANCODE+1 tiny generated pages (one per byte value from
0x00 up to however high TABLE_START+scancode can go without exceeding
255, including 0xE0 and 0xF0 since both sit well within that range
even though dispatch can never actually hand either of them to it --
see "Page budget notes" below for why they're still built). Each page
does exactly one thing: ADDI x8,x0,<ascii-value-or-0>, then a plain
switch_to() to the shared EMIT page. No runtime table lookup logic at
all -- the "lookup" IS which page execution lands on, which is
exactly what build_st7789_flash_image.py's fill-ring control page
already proved works for a runtime-computed page target; this reuses
the identical mechanism for keyboard input instead of pixel-loop
control flow.

Flash cost: MAX_SAFE_SCANCODE+1 one-instruction pages (227 with the
current page layout), roughly 10KB. Negligible next to any SPI flash
chip likely to be on this Pmod.

### Page budget notes

Every conditional-branch page below funnels through
switch_to_computed() with a two-armed CONVERGE pattern (both arms
loading x30 with a different target page, converging at the same
offset before the switch) -- see asm_pineapple.py's own docstring for
why both arms must land at the identical byte offset. Splitting the
four framing checks (break_pending? / ==0xF0? / ==0xE0? / ext_pending?)
across FOUR separate small pages instead of cramming them into fewer,
denser pages was a deliberate choice after the first attempt at a
single combined page didn't fit (needed roughly 12 instructions,
window budget is 8) -- each of the four fits comfortably (6-7
instructions each), with a page-switch's inherent latency being
completely irrelevant here (this is a human typing, not anything
remotely timing-sensitive).

PROCESS_DISPATCH itself is also a two-armed CONVERGE page, for a
reason that has nothing to do with framing: FLASH_PAGE (mem.v) is 8
bits, so TABLE_START+scancode overflows for scancode values above
MAX_SAFE_SCANCODE (255-TABLE_START) -- and the write silently WRAPS
rather than saturating or faulting, landing on whatever unrelated page
that wrapped number happens to name. This was an actual bug caught in
simulation: scancode 0xFF wrapped to EMIT_PAGE's own page number,
which blindly re-emitted whatever stale ASCII value was left in x8
from the last REAL translation instead of anything defensible.
PROCESS_DISPATCH now checks the scancode against MAX_SAFE_SCANCODE
first and routes anything over it to TABLE_START's own page (scancode
0's translation -- always 0x00, since scancode 0x00 is never a real
key and so has no SCANCODE_TO_ASCII entry) instead of letting the add
run unchecked.

The translation table only builds pages for scancode values that
survive that bounds check (0x00 through MAX_SAFE_SCANCODE), including
0xE0 and 0xF0 -- both comfortably within the safe range -- even though
PROCESS_DISPATCH can never actually hand either of them to it (both
are always intercepted by PROCESS_F0/PROCESS_E0 first). Skipping
THEIR two page slots was tried first (before the bounds check existed)
and is wrong: PagedAsm's page counter only advances when a page
actually finishes, so skipping a loop iteration leaves a gap in the
physical page sequence, and every scancode value ABOVE the skipped one
would then land on the wrong page (off by however many slots were
skipped) via the scancode==page-number arithmetic PROCESS_DISPATCH
relies on. Building harmless never-reached pages for 0xE0/0xF0 (they
just fall out of SCANCODE_TO_ASCII.get(scancode, 0) as ascii_val=0,
same as any other unmapped key) keeps the indexing exact with no
special-casing at the dispatch site. Scancode values ABOVE
MAX_SAFE_SCANCODE get no page at all, unlike 0xE0/0xF0 -- there's no
indexing reason to build one, since the bounds check already
intercepts every one of them before this arithmetic would apply.

Verified in test/tb_ps2_ascii.v: sends real bit-banged PS/2 frames
covering a plain make code, a break sequence (make + break of the
SAME key, confirming only one GPIO_OUT write happens and it's not
disturbed by the break), an extended make+break pair (confirming
BOTH are fully consumed with no emit), an unmapped key (confirming
0x00), several more plain make codes back-to-back (confirming the
flags don't get stuck set and the reader keeps working indefinitely),
and a scancode above MAX_SAFE_SCANCODE (confirming the FLASH_PAGE
bounds check above actually prevents the wraparound bug it exists
for, rather than just being untested code).
"""
import sys
sys.path.insert(0, '.')
from asm_pineapple import PagedAsm

GPIO_OUT = 0xF0
GPIO_IN = 0xF4
FLASH_PAGE = 0xFC

CLOCK_BIT = 3
DATA_BIT = 4
CLOCK_MASK = 1 << CLOCK_BIT
DATA_MASK = 1 << DATA_BIT

# Registers (Step 1's x1-x4 unchanged; x5/x6 are new):
#   x1 = latest GPIO_IN snapshot (re-read every poll iteration)
#   x2 = CLOCK bit extracted from x1
#   x3 = DATA bit extracted from x1 (sampling pages only)
#   x4 = accumulating scancode / general scratch in the process pages
#   x5 = break_pending flag (0/1)
#   x6 = ext_pending flag (0/1)
#   x7 = scratch (comparison constants)
#   x8 = translated ASCII result, valid only on the path into EMIT_PAGE
#   x30 = switch_to_computed()'s scratch register (PagedAsm default)
ACCUM = 4
BREAK_PENDING = 5
EXT_PENDING = 6

# Scan Code Set 2 make codes -> unshifted ASCII. Anything not listed
# here (function keys, Escape, modifiers, arrows, numpad, etc.)
# translates to 0x00 -- see module docstring for why that's Step 2's
# deliberate scope, not an oversight.
SCANCODE_TO_ASCII = {
    0x1C: ord('a'), 0x32: ord('b'), 0x21: ord('c'), 0x23: ord('d'),
    0x24: ord('e'), 0x2B: ord('f'), 0x34: ord('g'), 0x33: ord('h'),
    0x43: ord('i'), 0x3B: ord('j'), 0x42: ord('k'), 0x4B: ord('l'),
    0x3A: ord('m'), 0x31: ord('n'), 0x44: ord('o'), 0x4D: ord('p'),
    0x15: ord('q'), 0x2D: ord('r'), 0x1B: ord('s'), 0x2C: ord('t'),
    0x3C: ord('u'), 0x2A: ord('v'), 0x1D: ord('w'), 0x22: ord('x'),
    0x35: ord('y'), 0x1A: ord('z'),

    0x45: ord('0'), 0x16: ord('1'), 0x1E: ord('2'), 0x26: ord('3'),
    0x25: ord('4'), 0x2E: ord('5'), 0x36: ord('6'), 0x3D: ord('7'),
    0x3E: ord('8'), 0x46: ord('9'),

    0x29: ord(' '),   # space
    0x5A: 0x0D,       # enter -> CR
    0x66: 0x08,       # backspace -> BS
    0x0D: 0x09,       # tab

    0x4E: ord('-'), 0x55: ord('='), 0x54: ord('['), 0x5B: ord(']'),
    0x5D: ord('\\'), 0x4C: ord(';'), 0x52: ord("'"), 0x41: ord(','),
    0x49: ord('.'), 0x4A: ord('/'), 0x0E: ord('`'),
}

p = PagedAsm(load_base=0xB4, window_bytes=44, flash_page_addr=FLASH_PAGE)


def reserve_page(want):
    assert p._cur_page_num == want, (
        f"page numbering drifted: expected to be building page {want}, "
        f"actually about to build page {p._cur_page_num}"
    )


def wait_clock(level, next_page):
    """Unchanged from Step 1 -- see build_ps2_reader.py."""
    p.page.label("POLL")
    p.page.LBU(1, 0, GPIO_IN)
    p.page.ANDI(2, 1, CLOCK_MASK)
    if level == 0:
        p.page.BNE(2, 0, "POLL")
    else:
        p.page.BEQ(2, 0, "POLL")
    p.switch_to(next_page)


def wait_clock_low_and_sample(bit_value, next_page):
    """Unchanged from Step 1 -- see build_ps2_reader.py."""
    p.page.label("POLL")
    p.page.LBU(1, 0, GPIO_IN)
    p.page.ANDI(2, 1, CLOCK_MASK)
    p.page.BNE(2, 0, "POLL")
    p.page.ANDI(3, 1, DATA_MASK)
    p.page.BEQ(3, 0, "SKIP_SET")
    p.page.ORI(ACCUM, ACCUM, bit_value)
    p.page.label("SKIP_SET")
    p.switch_to(next_page)


# =====================================================================
# Pages 0-22: byte receive state machine -- identical to Step 1.
# =====================================================================

reserve_page(0)
p.page.ADDI(ACCUM, 0, 0)
p.page.ADDI(BREAK_PENDING, 0, 0)
p.page.ADDI(EXT_PENDING, 0, 0)
p.switch_to(1)

reserve_page(1)
wait_clock(level=0, next_page=2)   # START_LOW
reserve_page(2)
wait_clock(level=1, next_page=3)   # START_HIGH

BIT_PAGES_START = 3
page_num = BIT_PAGES_START
for bit_i in range(8):
    bit_value = 1 << bit_i
    reserve_page(page_num)
    wait_clock_low_and_sample(bit_value, next_page=page_num + 1)
    page_num += 1
    reserve_page(page_num)
    wait_clock(level=1, next_page=page_num + 1)
    page_num += 1

PARITY_LOW, PARITY_HIGH = page_num, page_num + 1
reserve_page(PARITY_LOW)
wait_clock(level=0, next_page=PARITY_HIGH)
reserve_page(PARITY_HIGH)
wait_clock(level=1, next_page=PARITY_HIGH + 1)

STOP_LOW, STOP_HIGH = PARITY_HIGH + 1, PARITY_HIGH + 2
reserve_page(STOP_LOW)
wait_clock(level=0, next_page=STOP_HIGH)
reserve_page(STOP_HIGH)
wait_clock(level=1, next_page=STOP_HIGH + 1)   # -> PROCESS_BP (new, replaces Step 1's RESULT_PAGE)

# =====================================================================
# New in Step 2: framing-aware processing + ASCII dispatch
# =====================================================================

PROCESS_BP = STOP_HIGH + 1
PROCESS_F0 = PROCESS_BP + 1
PROCESS_E0 = PROCESS_F0 + 1
PROCESS_EXTCHECK = PROCESS_E0 + 1
PROCESS_DISPATCH = PROCESS_EXTCHECK + 1
EMIT_PAGE = PROCESS_DISPATCH + 1
TABLE_START = EMIT_PAGE + 1          # page numbers 0x00-0xFF map directly onto
                                       # scancode values from here -- see below

# --- PROCESS_BP: is a break sequence already in progress? ---
reserve_page(PROCESS_BP)
p.page.BEQ(BREAK_PENDING, 0, "NOT_BP")
# break_pending was set: this byte is the released key's own code
# (don't care which key, or whether it was an extended one) -- clear
# both flags, no emit, straight back to listening.
p.page.ADDI(BREAK_PENDING, 0, 0)
p.page.ADDI(EXT_PENDING, 0, 0)
p.page.ADDI(ACCUM, 0, 0)
p.page.ADDI(30, 0, 1)          # -> START_LOW
p.page.JAL(0, "CONVERGE")
p.page.label("NOT_BP")
p.page.ADDI(30, 0, PROCESS_F0)
p.page.label("CONVERGE")
p.switch_to_computed()

# --- PROCESS_F0: is this byte the break-code prefix (0xF0)? ---
reserve_page(PROCESS_F0)
p.page.ADDI(7, 0, 0xF0)
p.page.BEQ(ACCUM, 7, "IS_F0")
p.page.ADDI(30, 0, PROCESS_E0)
p.page.JAL(0, "CONVERGE")
p.page.label("IS_F0")
p.page.ADDI(BREAK_PENDING, 0, 1)   # leave ext_pending as-is -- E0 F0 xx
                                     # needs to stay "extended" through
                                     # the break too
p.page.ADDI(ACCUM, 0, 0)
p.page.ADDI(30, 0, 1)               # -> START_LOW
p.page.label("CONVERGE")
p.switch_to_computed()

# --- PROCESS_E0: is this byte the extended-code prefix (0xE0)? ---
reserve_page(PROCESS_E0)
p.page.ADDI(7, 0, 0xE0)
p.page.BEQ(ACCUM, 7, "IS_E0")
p.page.ADDI(30, 0, PROCESS_EXTCHECK)
p.page.JAL(0, "CONVERGE")
p.page.label("IS_E0")
p.page.ADDI(EXT_PENDING, 0, 1)
p.page.ADDI(ACCUM, 0, 0)
p.page.ADDI(30, 0, 1)               # -> START_LOW
p.page.label("CONVERGE")
p.switch_to_computed()

# --- PROCESS_EXTCHECK: is an extended make code in progress? ---
reserve_page(PROCESS_EXTCHECK)
p.page.BEQ(EXT_PENDING, 0, "NOT_EP")
# ext_pending was set: this byte is the extended key's own code --
# no ASCII mapping for any extended key in Step 2, consume it.
p.page.ADDI(EXT_PENDING, 0, 0)
p.page.ADDI(ACCUM, 0, 0)
p.page.ADDI(30, 0, 1)          # -> START_LOW
p.page.JAL(0, "CONVERGE")
p.page.label("NOT_EP")
p.page.ADDI(30, 0, PROCESS_DISPATCH)
p.page.label("CONVERGE")
p.switch_to_computed()

# --- PROCESS_DISPATCH: plain make code -- jump to its translation
# page. Translation pages start at TABLE_START, not page 0, so the
# target is TABLE_START + scancode, not the scancode value alone --
# missing the + TABLE_START here was an actual bug caught in
# simulation (scancode 0x1C landed on page 28 -- coincidentally a
# valid page, EMIT_PAGE itself, since 0x1C==28 decimal -- instead of
# page 57 (TABLE_START+0x1C), silently reading x8 before it was ever
# set rather than crashing outright, which is what made it easy to
# miss without actually running this).
#
# A second, equally silent bug from the same family, ALSO only found
# by actually running a case for it: FLASH_PAGE (mem.v) is 8 bits, so
# any TABLE_START+scancode above 255 wraps, not saturates -- e.g.
# TABLE_START+0xFF landed exactly on EMIT_PAGE's own page number,
# which blindly re-emitted whatever stale ASCII value was left in x8
# from the last REAL translation instead of a defensible 0x00. Fixed
# by bounds-checking BEFORE the add: only scancode values up to
# MAX_SAFE_SCANCODE (255-TABLE_START) can safely map to
# TABLE_START+scancode without leaving the 0-255 page-number range, so
# anything above that is routed to TABLE_START's own page instead
# (scancode 0's translation, which is always 0x00 -- scancode 0x00
# itself is never a real key, so SCANCODE_TO_ASCII has no entry for
# it) rather than letting the add wrap into an arbitrary unrelated
# page. This is also why the table below only builds
# MAX_SAFE_SCANCODE+1 entries instead of the full 256: nothing can
# ever dispatch past that point anymore, so there's no indexing
# reason left to build pages that can never be reached (unlike 0xE0/
# 0xF0, which sit WITHIN the safe range and still need real, if
# unreachable, slots to keep the rest of the table's arithmetic
# exact). ---
MAX_SAFE_SCANCODE = 255 - TABLE_START   # highest scancode TABLE_START+scancode can safely address

reserve_page(PROCESS_DISPATCH)
p.page.ADDI(7, 0, MAX_SAFE_SCANCODE)
p.page.BLT(7, ACCUM, "OUT_OF_RANGE")     # MAX_SAFE_SCANCODE < scancode -> would overflow FLASH_PAGE
p.page.ADDI(7, 0, TABLE_START)
p.page.ADD(30, ACCUM, 7)
p.page.JAL(0, "CONVERGE")
p.page.label("OUT_OF_RANGE")
p.page.ADDI(30, 0, TABLE_START)          # scancode 0's page -- always ascii=0, same as any other unmapped key
p.page.label("CONVERGE")
p.switch_to_computed()

# --- EMIT_PAGE: shared by every translation page below. ---
reserve_page(EMIT_PAGE)
p.page.SW(8, 0, GPIO_OUT)
p.page.ADDI(ACCUM, 0, 0)
p.switch_to(1)   # -> START_LOW

# --- Translation table: one page per scancode value from 0x00 up to
# MAX_SAFE_SCANCODE, landing at page TABLE_START+scancode -- including
# 0xE0 and 0xF0 (both comfortably within the safe range), even though
# PROCESS_DISPATCH can never actually reach those two (intercepted
# earlier as protocol prefixes): skipping their page slots would leave
# a GAP in the page sequence, breaking the "page number == scancode
# value" arithmetic for every scancode above them (PagedAsm's own page
# counter only advances when a page is actually finished, not per loop
# iteration) -- building them anyway, as normal-but-unreachable pages,
# keeps the indexing exact with zero special-casing. Scancodes ABOVE
# MAX_SAFE_SCANCODE get no page at all -- PROCESS_DISPATCH's bounds
# check above already redirects every one of them to TABLE_START
# before this arithmetic would ever apply. ---
assert p._cur_page_num == TABLE_START
for scancode in range(MAX_SAFE_SCANCODE + 1):
    reserve_page(TABLE_START + scancode)
    ascii_val = SCANCODE_TO_ASCII.get(scancode, 0)
    p.page.ADDI(8, 0, ascii_val)
    p.switch_to(EMIT_PAGE)

TOTAL_PAGES = TABLE_START + MAX_SAFE_SCANCODE + 1

# Every page built above (including the very last table entry,
# scancode 0xFF) already ends in its own switch_to()/switch_to_computed()
# call, which is what actually finalizes a page into self.pages -- so
# finalize() alone is correct here; there's no "terminal" page that
# needs finalize_last_page()'s special no-further-transition handling
# (unlike e.g. build_st7789_flash_image.py's SPIN page).
image = p.finalize()
n_pages = len(image) // 44
print(f"PS/2 ASCII reader flash image: {len(image)} bytes, {n_pages} pages")
print(f"  CLOCK=ui_in[{CLOCK_BIT}] DATA=ui_in[{DATA_BIT}], ASCII -> GPIO_OUT (uo_out)")
print(f"  translation table: pages {TABLE_START}-{TABLE_START + MAX_SAFE_SCANCODE} "
      f"(scancode == page number - {TABLE_START}; 0xE0/0xF0 slots unused; "
      f"scancodes above 0x{MAX_SAFE_SCANCODE:02x} have no page -- bounds-checked "
      f"in PROCESS_DISPATCH instead)")

with open('ps2_ascii_flash_image.bin', 'wb') as f:
    f.write(image)
with open('ps2_ascii_flash_image.hex', 'w') as f:
    for b in image:
        f.write(f"{b:02x}\n")
print("Wrote ps2_ascii_flash_image.bin and .hex")
