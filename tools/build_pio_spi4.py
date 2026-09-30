#!/usr/bin/env python3
"""build_pio_spi4.py -- ONE firmware image, ONE PIO state machine, FOUR SPI modes in a row,
with the CPU in the data path.

The CPU reprograms the same PIO state machine for SPI mode 0, 1, 2 and 3 (the four CPOL/CPHA
combinations), drives the chip-select itself, and *relays data*: the byte the slave returns on
MISO in mode N is read out of the RX FIFO by the CPU and sent back out on MOSI as the first
byte of mode N+1.  A slave model that checks MOSI therefore proves the whole chain

    slave MISO -> PIO input sync -> ISR autopush -> RX FIFO -> CPU load -> CPU store
               -> TX FIFO -> autopull -> OSR -> MOSI

worked in every mode, not just that the pins wiggled.

Pins (all on the existing pads, no RTL change):
    MOSI = uo_out[0]   PIO pin 0 (OUT)          SCK = uo_out[1]  PIO pin 1 (side-set)
    MISO = ui_in[2]    PIO pin 2 (IN)           CS_n = uo_out[2] CPU GPIO_OUT bit 2 (not PIO)

Per phase m (SCK = clk / (4 * CLKDIV), CLKDIV = 8, 8-bit MSB-first, autopull/autopush at 8):
    T[m] , F[m]   MOSI bytes      (T[0] constant, T[m>0] = MISO byte 0 of phase m-1, relayed)
    M[m] , N[m]   MISO bytes returned by the slave model

The last phase's MISO byte is left in a register and written to GPIO_OUT (0xF0) as an epilogue,
so the testbench can check the CPU really received it, then the core executes EBREAK.

Run from tools/:  python3 build_pio_spi4.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)

SM = 0
SPI_CLKDIV = 8
CTRL_RESET_SM0 = (1 << 4) | (1 << 8) | (1 << 12)       # restart + clkdiv restart + FIFO clear

PROGRAMS = ["spi_master.pio", "spi_cpha1.pio", "spi_cpol1_cpha0.pio", "spi_cpol1_cpha1.pio"]

# ---- test vectors (shared with test/tb_pio_cpu_spi4.v) ----
T0 = 0xA5                                   # only constant first MOSI byte; the rest are relayed
F = [0x3C, 0x81, 0xE7, 0x18]                # second MOSI byte of each phase
M = [0xC3, 0x5A, 0x96, 0x6D]                # first MISO byte of each phase  (M[3] has bit 2 set)
N = [0xF0, 0x0F, 0xAA, 0x55]                # second MISO byte of each phase

R_RELAY, R_DISCARD = 8, 9                   # x8 = relayed byte, x9 = second RX word (dropped)
CS_HIGH_BIT = 0x04                          # GPIO_OUT bit 2 = CS_n (uo_out[2])
SCK_BIT = 0x02                              # GPIO_OUT bit 1 = SCK pad level before PIO owns it

here = os.path.dirname(os.path.abspath(__file__))


def prog_of(name):
    return assemble(open(os.path.join(here, "..", "pio", name)).read(), origin=0)


def expected_mosi():
    t = [T0] + M[:3]
    return [(t[m], F[m]) for m in range(4)]


h = PioHost()

# CS_n idles high before anything else touches the bus (SCK/MOSI low, SCK level per mode below)
h.write_gpio_out_imm(CS_HIGH_BIT)

for mode, name in enumerate(PROGRAMS):
    cpol = mode >> 1
    prog = prog_of(name)

    # ---- stop SM0, wipe its state, load this mode's program, configure it (not enabled yet)
    h.write_reg(R_CTRL, CTRL_RESET_SM0)
    h.load_program(prog)
    h.write_reg(sm_reg(SM, SM_PINCTRL),
                pinctrl(out_base=0, out_count=1, set_base=1, set_count=1, side_base=1,
                        side_count=prog.side_bits, in_base=2))
    h.write_reg(sm_reg(SM, SM_EXEC),
                execctrl(wrap_bottom=prog.wrap_bottom, wrap_top=prog.wrap_top,
                         side_en=prog.side_opt, side_pindir=prog.side_pindirs))
    h.write_reg(sm_reg(SM, SM_SHIFT),
                shiftctrl(autopush=True, autopull=True, in_right=False, out_right=False,
                          push_thresh=8, pull_thresh=8))
    h.write_reg(sm_reg(SM, SM_CLKDIV), clkdiv_reg(SPI_CLKDIV, 0))
    # jmp to the program start.  A forced instruction also carries a side-set value in bit 12
    # (these programs use a plain `.side_set 1`), so it must be the IDLE level of this mode:
    # a plain 0x0000|addr would drive SCK low for a moment, glitching a CPOL=1 clock line.
    h.force(SM, (cpol << 12) | prog.wrap_bottom)

    # ---- SCK idle level for this mode, BEFORE the pad is (still/again) PIO's: no glitch
    h.write_gpio_out_imm(CS_HIGH_BIT | (SCK_BIT if cpol else 0))    # first hand-over uses GPIO_OUT
    h.force(SM, 0xF001 if cpol else 0xE000)                         # set pins,x + side-set = idle
    h.write_reg(R_PIN_OWN, 0x003)                                   # MOSI + SCK -> PIO
    h.write_reg(R_CTRL, 1 << SM)                                    # enable

    # ---- CS low (CPU), then two bytes: first is relayed from the previous mode's MISO
    h.write_gpio_out_imm(SCK_BIT if cpol else 0x00)
    if mode == 0:
        h.tx_push_paced(SM, T0 << 24)
    else:
        h.tx_push_reg_paced(SM, R_RELAY)                            # x8 = previous MISO << 24
    h.tx_push_paced(SM, F[mode] << 24)

    # ---- CPU reads both received bytes (also proves the transfer finished)
    h.wait_rx_ready(SM)
    h.rx_pop(SM, R_RELAY)                                           # x8 = MISO byte 0
    h.wait_rx_ready(SM)
    h.rx_pop(SM, R_DISCARD)                                         # x9 = MISO byte 1 (dropped)
    h.delay(16 * SPI_CLKDIV)                                        # let SCK return to idle
    h.write_gpio_out_imm(CS_HIGH_BIT | (SCK_BIT if cpol else 0))    # CS high
    if mode < 3:
        h.shift_left(R_RELAY, 24)                                   # ready to send in next mode

# ---- epilogue: the CPU shows the last MISO byte it received on the LED pads, then parks
h.write_gpio_out(R_RELAY)
h.halt()

image = h.build()
print(f"PIO four-mode SPI flash image: {len(image)} bytes, {h.pages} pages")
print("expected MOSI per phase:", [(hex(a), hex(b)) for a, b in expected_mosi()])
write_image(image, os.path.join(here, "pio_spi4_flash_image"))
print("Wrote pio_spi4_flash_image.bin and .hex")
