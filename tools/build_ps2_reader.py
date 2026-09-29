#!/usr/bin/env python3
"""
PS/2 keyboard reader for AgilA32 -- Step 1 of the keyboard option
(see project notes): a bit-banged PS/2 receiver that decodes one
11-bit frame (start + 8 data bits + parity + stop) at a time and
reflects the last scancode byte on GPIO_OUT, forever. Meant to be
pre-programmed onto the external flash chip on the QSPI Pmod (CS0)
and reached via build_flash_handoff_stub.py's tiny bootloaded stub,
exactly like build_st7789_flash_image.py -- see that file and
mem.v's "Reprogrammability" / "Bank-switched flash execution" for the
handoff/paging mechanics this depends on.

Why this needs FLASH_PAGE at all (unlike a plain bootloaded program):
a real frame-sync state machine -- wait for a clock edge, sample data,
loop back and wait for the NEXT edge, eleven times per frame, forever
-- is comfortably bigger than the 44-byte/11-instruction on-chip RAM
window the normal bootloader targets. Same headroom problem the LCD
driver had, same fix.

Pin usage (GPIO_IN = ui_in, 0xF4; see mem.v):
  ui_in[3] = PS/2 CLOCK (keyboard-driven; idle high)
  ui_in[4] = PS/2 DATA  (keyboard-driven; idle high)
  ui_in[7:5], ui_in[2:0] are ignored by this program (ui_in[2:0] are
  only meaningful during the earlier bootload-over-GPIO phase that
  gets this program's handoff stub loaded in the first place; once
  running, this program never reads them for anything other than as
  don't-care bits ANDed away by CLOCK_MASK/DATA_MASK below).
GPIO_OUT (uo_out, 0xF0): overwritten with the last fully-received
  scancode byte every frame -- same "once a bootloaded program takes
  over GPIO_OUT it no longer reflects the demo counter" convention the
  LCD driver already established.

No interrupts on this core (see docs/info.md) -- this is a plain
polling loop, structured as one page per protocol PHASE rather than
one page per bit like the LCD driver's SEND_BYTE:

  page 0            : init (clear accumulator), -> START_LOW
  page 1  START_LOW : wait for CLOCK to go low (start bit) -> START_HIGH
  page 2  START_HIGH: wait for CLOCK to go high again        -> BIT0_LOW
  page 3  BIT0_LOW  : wait CLOCK low, sample DATA into bit 0  -> BIT0_HIGH
  page 4  BIT0_HIGH : wait CLOCK high                          -> BIT1_LOW
  ... (repeats for bits 1..7, 2 pages each) ...
  page 19 PARITY_LOW / 20 PARITY_HIGH : waited through, value ignored
  page 21 STOP_LOW   / 22 STOP_HIGH   : waited through, value ignored
  page 23 RESULT    : GPIO_OUT = accumulator; accumulator = 0; -> START_LOW

Each data-bit's LOW-phase page ORs a *compile-time-known* constant
(1<<n) into the accumulator register when DATA samples high --
exactly the same "every bit is a compile-time constant, no runtime
shift register needed" trick the LCD driver's per-bit pages use for
SEND_BYTE, just OR-ing into a fixed bit position on the way IN instead
of shifting a fixed byte out. This is also why bit position is fixed
per page rather than looping over a runtime bit-index: a loop would
need a per-iteration variable shift amount, which is exactly the extra
complexity this sidesteps.

Every phase gets its own low-and-high pair of pages (rather than e.g.
folding a "wait low, sample, wait high" sequence into one page) so
each page comfortably fits the 7-8 usable-instruction budget a 44-byte
window leaves after PagedAsm's switch+trampoline reservation -- see
asm_pineapple.py's PagedAsm docstring. The low-phase pages that DO
sample (the 8 data bits) are the tightest: LBU + AND(clock) + BNE-loop
+ AND(data) + BEQ + ORI + the switch's own ADDI = 7 words, still one
spare inside the 8-word non-page-0 budget.

Framing note: this program does not check the parity or stop bit's
VALUE, only that a clock edge occurred where each is expected -- good
enough to stay in sync with a well-behaved PS/2 device (this is Step
1, "prove the mechanism", not a fully spec-compliant PS/2 host with
parity-error/inhibit handling). It also never asserts CLOCK or DATA
itself (both are inputs here, and PS/2 is host-inhibit / device-clocks
in the common keyboard-to-host direction this only implements) -- so
there's no host-to-device command support, just passive listening.

Verified in test/tb_ps2_reader.v against a Verilog task that
bit-bangs real 11-bit PS/2 frames (start/8 data bits LSB-first/odd
parity/stop) onto ui_in[3]/ui_in[4], the same way a real keyboard
would, and checks GPIO_OUT lands on the exact scancode sent -- across
several distinct scancodes sent back-to-back, to prove the state
machine re-arms correctly rather than only working once.
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

# Registers:
#   x1 = latest GPIO_IN snapshot (re-read every poll iteration)
#   x2 = CLOCK bit extracted from x1
#   x3 = DATA bit extracted from x1 (only used in sampling pages)
#   x4 = accumulating scancode (persists across FLASH_PAGE switches --
#        switching pages only changes what the LOAD_BASE window
#        fetches from, it's not a core reset, so plain register state
#        survives exactly like the LCD driver's pixel counter does)
ACCUM = 4

p = PagedAsm(load_base=0xB4, window_bytes=44, flash_page_addr=FLASH_PAGE)


def reserve_page(want):
    """Same sanity check build_st7789_flash_image.py uses: asserts this
    script's hardcoded page numbers haven't drifted from actual page
    allocation order."""
    assert p._cur_page_num == want, (
        f"page numbering drifted: expected to be building page {want}, "
        f"actually about to build page {p._cur_page_num}"
    )


def wait_clock(level, next_page):
    """Builds a page that polls GPIO_IN in a tight loop until CLOCK
    reads `level` (0 = wait for the falling edge, i.e. loop while
    high; 1 = wait for the rising edge, i.e. loop while low), then
    switches to `next_page`. No sampling -- used for the start bit,
    parity bit, and stop bit phases, and for every bit's HIGH
    (re-arm) phase."""
    p.page.label("POLL")
    p.page.LBU(1, 0, GPIO_IN)
    p.page.ANDI(2, 1, CLOCK_MASK)
    if level == 0:
        p.page.BNE(2, 0, "POLL")   # loop while CLOCK bit is set (high)
    else:
        p.page.BEQ(2, 0, "POLL")   # loop while CLOCK bit is clear (low)
    p.switch_to(next_page)


def wait_clock_low_and_sample(bit_value, next_page):
    """Like wait_clock(0, ...), but on the exact GPIO_IN read that saw
    CLOCK low (the same byte read, so DATA is sampled at the identical
    instant, not a stale one), also checks DATA and ORs the
    compile-time constant `bit_value` into the accumulator if it was
    set. Used for the 8 data-bit LOW phases only."""
    p.page.label("POLL")
    p.page.LBU(1, 0, GPIO_IN)
    p.page.ANDI(2, 1, CLOCK_MASK)
    p.page.BNE(2, 0, "POLL")
    p.page.ANDI(3, 1, DATA_MASK)
    p.page.BEQ(3, 0, "SKIP_SET")
    p.page.ORI(ACCUM, ACCUM, bit_value)
    p.page.label("SKIP_SET")
    p.switch_to(next_page)


# --- Page 0: clear the accumulator, then start listening ---
reserve_page(0)
p.page.ADDI(ACCUM, 0, 0)
p.switch_to(1)

# --- Pages 1-2: start bit (value not checked, just synced on) ---
reserve_page(1)
wait_clock(level=0, next_page=2)   # START_LOW
reserve_page(2)
wait_clock(level=1, next_page=3)   # START_HIGH

# --- Pages 3-18: the 8 data bits, LSB first, 2 pages each ---
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

# --- Pages 19-20: parity bit (value not checked) ---
PARITY_LOW, PARITY_HIGH = page_num, page_num + 1
reserve_page(PARITY_LOW)
wait_clock(level=0, next_page=PARITY_HIGH)
reserve_page(PARITY_HIGH)
wait_clock(level=1, next_page=PARITY_HIGH + 1)

# --- Pages 21-22: stop bit (value not checked) ---
STOP_LOW, STOP_HIGH = PARITY_HIGH + 1, PARITY_HIGH + 2
reserve_page(STOP_LOW)
wait_clock(level=0, next_page=STOP_HIGH)
reserve_page(STOP_HIGH)
wait_clock(level=1, next_page=STOP_HIGH + 1)

# --- Final page: publish the scancode, reset, go listen for the next frame ---
RESULT_PAGE = STOP_HIGH + 1
reserve_page(RESULT_PAGE)
p.page.SW(ACCUM, 0, GPIO_OUT)
p.page.ADDI(ACCUM, 0, 0)
p.switch_to(1)   # back to START_LOW -- this program never actually stops listening

TOTAL_PAGES = RESULT_PAGE + 1

image = p.finalize()
assert len(image) == TOTAL_PAGES * 44
print(f"PS/2 reader flash image: {len(image)} bytes, {TOTAL_PAGES} pages")
print(f"  CLOCK=ui_in[{CLOCK_BIT}] DATA=ui_in[{DATA_BIT}], scancode -> GPIO_OUT (uo_out)")

with open('ps2_reader_flash_image.bin', 'wb') as f:
    f.write(image)
with open('ps2_reader_flash_image.hex', 'w') as f:
    for b in image:
        f.write(f"{b:02x}\n")
print("Wrote ps2_reader_flash_image.bin and .hex")
