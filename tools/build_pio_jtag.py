#!/usr/bin/env python3
"""build_pio_jtag.py -- the CPU walks a JTAG TAP through the PIO and relays data through itself.

One PIO state machine runs pio/jtag.pio (TCK generator + TMS/TDI driver + TDO sampler); the CPU is
the JTAG "probe software": it decides every TAP move by pushing header words, and it is in the
data path exactly like build_pio_spi4.py:

    reset TAP (TMS=1 x5, then 0)                                   6 TCK
    RTI -> Shift-DR, read the 32-bit IDCODE (default instruction)  3 + 32 + 2 = 37
    CPU pops the IDCODE word from the RX FIFO  (x8)
    RTI -> Shift-IR, load the 4-bit instruction USER (0x2)         4 + 4 + 2 = 10
    RTI -> Shift-DR, WRITE the 16-bit USER register with x8        3 + 16 + 2 = 21
        (a relay: the TX FIFO word is a CPU register holding what the target just returned; the
         ISR shifts right, so after 32 clocks it IS the IDCODE, and the low 16 bits leave first)
    RTI -> Shift-DR, READ the USER register back (shifting x8 in again, so the read does not
        erase the register: every scan also writes)               3 + 16 + 2 = 21
    total 95 TCK pulses
    CPU shows the low byte of what it read back on GPIO_OUT, waits for the SM to finish its trailing
    TMS clocks, then EBREAKs.

Pins (existing pads, no RTL change):   TDI = uo_out[0]   TMS = uo_out[1]   TCK = uo_out[2]
                                       TDO = ui_in[3]    (PIO pins 0, 1, 2, 3)
CLKDIV 4 -> a data bit is 7 ticks = 28 core clocks per TCK period.

Run from tools/:  python3 build_pio_jtag.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

SM = 0
JTAG_CLKDIV = 4
CTRL_RESET_SM0 = (1 << 4) | (1 << 8) | (1 << 12)      # restart + clkdiv restart + FIFO clear
INSTR_USER = 0x2
R_RELAY, R_RESULT = 8, 9                              # x8 = IDCODE from the target, x9 = USER read back

here = os.path.dirname(os.path.abspath(__file__))
prog = assemble(open(os.path.join(here, "..", "pio", "jtag.pio")).read(), origin=0)


# ---- header words for pio/jtag.pio ------------------------------------------------------------
def tms_seq(*bits):
    """TMS-only sequence: one TCK pulse per bit, first bit first (max 10)."""
    assert 1 <= len(bits) <= 10
    return (len(bits) - 1) | sum(b << (6 + i) for i, b in enumerate(bits))


def shift_hdr(nbits):
    """Data shift of `nbits` pulses; TMS=1 on the last one; the next word is the TDI data."""
    assert 1 <= nbits <= 32
    return (nbits - 1) | (1 << 5)


TAP_RESET = tms_seq(1, 1, 1, 1, 1, 0)                 # -> Test-Logic-Reset -> Run-Test/Idle
RTI_TO_SHIFT_DR = tms_seq(1, 0, 0)                    # RTI -> Select-DR -> Capture-DR -> Shift-DR
RTI_TO_SHIFT_IR = tms_seq(1, 1, 0, 0)                 # RTI -> Select-DR -> Select-IR -> Capture-IR -> Shift-IR
EXIT_TO_RTI = tms_seq(1, 0)                           # Exit1 -> Update -> RTI

h = PioHost()

# TCK/TMS/TDI idle low on the pads BEFORE the PIO owns them, so the hand-over is glitch-free
h.write_gpio_out_imm(0x00)

h.write_reg(R_CTRL, CTRL_RESET_SM0)
h.load_program(prog)
h.write_reg(sm_reg(SM, SM_PINCTRL),
            pinctrl(out_base=0, out_count=1, set_base=1, set_count=1, side_base=2,
                    side_count=prog.side_bits, in_base=3))
h.write_reg(sm_reg(SM, SM_EXEC),
            execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                     side_en=prog.side_opt, side_pindir=prog.side_pindirs))
h.write_reg(sm_reg(SM, SM_SHIFT),
            shiftctrl(autopush=False, autopull=False, in_right=True, out_right=True))
h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(JTAG_CLKDIV, 0))
# forced jmp to the program start; its side-set bit (12) is the TCK idle level = 0 (low)
h.force(SM, (0 << 12) | prog.wrap_bottom)
h.write_reg(R_PIN_OWN, 0x007)                          # TDI, TMS, TCK -> PIO
h.write_reg(R_CTRL, 1 << SM)                           # enable

# ---- 1. reset the TAP, read the IDCODE (DR32 with the default IDCODE instruction)
h.tx_push_paced(SM, TAP_RESET, RTI_TO_SHIFT_DR, shift_hdr(32), 0, EXIT_TO_RTI)
h.wait_rx_ready(SM)
h.rx_pop(SM, R_RELAY)                                  # x8 = IDCODE

# ---- 2. load IR = USER
h.tx_push_paced(SM, RTI_TO_SHIFT_IR, shift_hdr(4), INSTR_USER, EXIT_TO_RTI)

# ---- 3. write USER with the low half of the IDCODE the CPU just read (relay through a register)
h.tx_push_paced(SM, RTI_TO_SHIFT_DR, shift_hdr(16))
h.tx_push_reg_paced(SM, R_RELAY)
h.tx_push_paced(SM, EXIT_TO_RTI)

# ---- 4. read USER back.  A JTAG scan is always also a write: whatever is shifted in is latched by
#         Update-DR.  Shifting zeros would erase the register we just wrote, so the read-back scan
#         shifts the same value in again (non-destructive read of a writable register).
h.tx_push_paced(SM, RTI_TO_SHIFT_DR, shift_hdr(16))
h.tx_push_reg_paced(SM, R_RELAY)
h.tx_push_paced(SM, EXIT_TO_RTI)

# ---- collect: RX words are IDCODE (already popped), IR scan, USER-write scan, USER-read scan
h.wait_rx_ready(SM); h.rx_pop(SM, R_RESULT)            # IR scan (discarded)
h.wait_rx_ready(SM); h.rx_pop(SM, R_RESULT)            # USER capture during the write (discarded)
h.wait_rx_ready(SM); h.rx_pop(SM, R_RESULT)            # USER read back: 16 bits, in the TOP half
h.shift_right(R_RESULT, 16)                            # -> USER value in bits 15:0

# the RX word arrives BEFORE the trailing Exit1 -> Update -> RTI clocks: wait for the SM to finish
h.wait_sm_idle(SM, prog.wrap_bottom)

h.write_gpio_out(R_RESULT)                             # low byte of what came back
h.halt()

image = h.build()
print(f"PIO JTAG flash image: {len(image)} bytes, {h.pages} pages")
print("header words: reset=%#x to_dr=%#x to_ir=%#x exit=%#x dr32=%#x ir4=%#x dr16=%#x" % (
    TAP_RESET, RTI_TO_SHIFT_DR, RTI_TO_SHIFT_IR, EXIT_TO_RTI, shift_hdr(32), shift_hdr(4), shift_hdr(16)))
write_image(image, os.path.join(here, "pio_jtag_flash_image"))
print("Wrote pio_jtag_flash_image.bin and .hex")
