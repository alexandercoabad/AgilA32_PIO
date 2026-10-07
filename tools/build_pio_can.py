#!/usr/bin/env python3
"""build_pio_can.py -- flash image: the CPU is a CAN node through the PIO (pio/can_tx.pio + pio/can_rx.pio).

    set up SM0 (transmitter) and SM1 (raw bit capture), CLKDIV 4  ->  64 clocks per bit
    send the standard frame  ID 0x123, data DE AD   (stuffed bit stream and CRC computed here, at build time)
        LED_OUT = (result >> 19) & 0xFF   (0xFC: ACK slot read dominant, the 11 bits after it recessive)
    drain the RX capture of that frame (the chip hears its own transmission), LED_OUT = its words, 4 bytes each
    drain the RX capture of the next frame on the bus (sent by the testbench), LED_OUT = its words, 4 bytes each
    EBREAK

Pins: TXD = uo_out[0] (PIO pin 0, PIN_OWN bit 0), RXD = ui_in[1] (PIO pin 1).  Because uo_out[0] belongs to the PIO,
the testbench records the LED_OUT writes themselves (all 8 bits), not the pad levels.

Also writes test/pio_can_expect.vh: the parameters, the expected wire bits and the expected LED_OUT bytes that
test/tb_pio_cpu_can.v checks.  Run from tools/:  python3 build_pio_can.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pioasm import assemble
from pio_host import (PioHost, pinctrl, execctrl, shiftctrl, clkdiv_reg, sm_reg,
                      R_CTRL, R_PIN_OWN, SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL, write_image)
import can_frame as F

TX_SM, RX_SM = 0, 1
CLKDIV = 4                                   # 16 ticks per bit * 4 = 64 clocks per bit
FRAME_A = dict(ident=0x123, data=b"\xde\xad")                 # sent by the chip
FRAME_B = dict(ident=0x2A5, data=b"\x01\x02\x03")             # sent by the testbench, nobody acknowledges it
R = 10

here = os.path.dirname(os.path.abspath(__file__))
tx = assemble(open(os.path.join(here, "..", "pio", "can_tx.pio")).read(), origin=0)
rx = assemble(open(os.path.join(here, "..", "pio", "can_rx.pio")).read(), origin=len(tx.instrs))

strict_a = F.tx_stream(**FRAME_A)
cmd_a = F.tx_command(strict_a)
strict_b = F.tx_stream(**FRAME_B)
wire_a = F.wire_bits(strict_a, acked=True)                    # strict + ACK slot + 11 recessive
words_a = F.capture_words(wire_a + [1] * 20)
words_b = F.capture_words(strict_b + [1] * 40)                # nobody acknowledges: ACK slot recessive
for words, fr in ((words_a, FRAME_A), (words_b, FRAME_B)):
    d = F.decode_capture(words)
    assert d["error"] is None and d["ident"] == fr["ident"] and d["data"] == fr["data"], d
assert F.decode_capture(words_a)["ack"] and not F.decode_capture(words_b)["ack"]
assert len(words_a) <= 4 and len(words_b) <= 4, "a capture must fit the 4 deep RX FIFO"
RESULT_BYTE = 0xFC                                            # (0xFFE00000 >> 19) & 0xFF

h = PioHost()
h.load_program(tx)
h.load_program(rx)
h.write_reg(sm_reg(TX_SM, SM_PINCTRL), pinctrl(out_base=0, out_count=1, set_base=0, set_count=1, in_base=1))
h.write_reg(sm_reg(TX_SM, SM_EXEC), execctrl(wrap_bottom=tx.wrap_bottom, wrap_top=tx.wrap_top, jmp_pin=1))
h.write_reg(sm_reg(TX_SM, SM_SHIFT), shiftctrl(autopush=False, autopull=False, in_right=True, out_right=True))
h.write_reg(sm_reg(TX_SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.force(TX_SM, F.SET_PINS_1)                               # TXD recessive before the PIO owns the pin
h.force(TX_SM, tx.wrap_bottom)                             # jmp cmd
h.write_reg(sm_reg(RX_SM, SM_PINCTRL), pinctrl(in_base=1, set_count=0))
h.write_reg(sm_reg(RX_SM, SM_EXEC), execctrl(wrap_bottom=rx.wrap_bottom, wrap_top=rx.wrap_top, jmp_pin=1))
h.write_reg(sm_reg(RX_SM, SM_SHIFT), shiftctrl(autopush=True, autopull=False, in_right=True, out_right=True,
                                               push_thresh=0))
h.write_reg(sm_reg(RX_SM, SM_CLKDIV), clkdiv_reg(CLKDIV, 0))
h.force(RX_SM, rx.wrap_top)                                # enter at `mov x, ~null`
h.write_reg(R_PIN_OWN, 0x001)                              # TXD (pad 0) -> PIO
# the whole command (header + 2 data words) fits the 4 deep TX FIFO: queue it BEFORE the state machine starts, so the
# slow CPU (flash paging) can never run the FIFO empty in the middle of the frame
assert len(cmd_a) <= 4
h.tx_push(TX_SM, *cmd_a)
h.write_reg(R_CTRL, (1 << TX_SM) | (1 << RX_SM))

# 1. the frame goes out; report the transmit result
h.wait_rx_ready(TX_SM)
h.rx_pop(TX_SM, R)
h.shift_right(R, 19)
h.write_gpio_out(R)


def report_words(n):
    for _ in range(n):
        h.wait_rx_ready(RX_SM)
        h.rx_pop(RX_SM, R)
        for k in range(4):
            h.write_gpio_out(R)
            if k < 3:
                h.shift_right(R, 8)


report_words(len(words_a))          # 2. the capture of our own frame
report_words(len(words_b))          # 3. the capture of the testbench's frame
h.halt()

image = h.build()
print(f"PIO CAN flash image: {len(image)} bytes, {h.pages} pages, TX {len(tx.instrs)} + RX {len(rx.instrs)} words, "
      f"CLKDIV {CLKDIV}")
write_image(image, os.path.join(here, "pio_can_flash_image"))
print("Wrote pio_can_flash_image.bin and .hex")

# ---- the checks of test/tb_pio_cpu_can.v
expect = [RESULT_BYTE]
for w in words_a + words_b:
    expect += [(w >> (8 * k)) & 0xFF for k in range(4)]


def bits_const(bits):
    v = 0
    for i, b in enumerate(bits):
        v |= b << i
    return "%d'h%x" % (256, v)


vh = ["// generated by tools/build_pio_can.py -- do not edit",
      "localparam integer CAN_CLKDIV  = %d;" % CLKDIV,
      "localparam integer CAN_IMG_LEN = %d;" % len(image),
      "localparam integer CAN_NA      = %d;   // strict bits of frame A (SOF .. CRC delimiter)" % len(strict_a),
      "localparam integer CAN_NB      = %d;   // bits of frame B the testbench sends (SOF .. CRC delimiter)" % len(strict_b),
      "localparam integer CAN_NEXP    = %d;   // LED_OUT writes the CPU makes" % len(expect),
      "localparam [255:0] CAN_A_WIRE  = %s;   // bit i = level of wire bit i of frame A, ACK slot and tail" % bits_const(wire_a),
      "localparam [255:0] CAN_B_BITS  = %s;   // bit i = bit i of frame B" % bits_const(strict_b),
      "reg [7:0] can_exp [0:%d];" % (len(expect) - 1),
      "task load_can_expect;",
      "    begin"]
vh += ["        can_exp[%d] = 8'h%02x;" % (i, b) for i, b in enumerate(expect)]
vh += ["    end", "endtask", ""]
open(os.path.join(here, "..", "test", "pio_can_expect.vh"), "w").write("\n".join(vh))
open(os.path.join(here, "..", "test", "pio_can_flash_image.hex"), "w").write(open(os.path.join(here, "pio_can_flash_image.hex")).read())
print("Wrote test/pio_can_expect.vh (%d expected LED_OUT bytes) and test/pio_can_flash_image.hex" % len(expect))
