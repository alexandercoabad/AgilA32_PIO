#!/usr/bin/env python3
"""gl_sig_mutation_sweep.py -- does the post-layout signature test (test/test_pio_gl_sig.py) notice a broken PIO?

Applies ONE one-line break at a time to pio/gl_sig0.pio, pio/gl_sig1.pio or src/pio_sm.v / src/pio.v, rebuilds the
flash image and runs `make COCOTB_TEST_FILTER=test_pio_postlayout_signature` (RTL, about 15 s each).
Works on a temp copy of the repo.

    python3 tools/gl_sig_mutation_sweep.py            # all mutants
    python3 tools/gl_sig_mutation_sweep.py R3 B2      # just these
    python3 tools/gl_sig_mutation_sweep.py --list
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, N, copy_repo, sh, sweep

T = copy_repo("gl_sig_mut_")
A, B, SM, PIO = T + "/pio/gl_sig0.pio", T + "/pio/gl_sig1.pio", T + "/src/pio_sm.v", T + "/src/pio.v"
TEST = T + "/test"

M = [
 ("A1", A, "set pins, 21 -> 20",                                [S("set pins, 21", "21", "20")]),
 ("A2", A, "mov pins, ~x -> x (no invert)",                     [S("mov pins, ~x", "~x", "x")]),
 ("A3", A, "in pins, 8 -> 7",                                   [S("in pins, 8", "8", "7")]),
 ("A4", A, "in x, 5 -> 4",                                      [S("in x, 5", "5", "4")]),
 ("A5", A, "mov osr, ::isr -> isr (no bit reverse)",            [S("mov osr, ::isr", "::isr", "isr")]),
 ("A6", A, "out pins, 8 -> 7",                                  [S("out pins, 8", "8", "7")]),
 ("A7", A, "out y, 8 -> out x, 8",                              [S("out y, 8", "out y", "out x")]),
 ("A8", A, "mov pins, y -> ~y",                                 [S("mov pins, y", "y", "~y")]),
 ("A9", A, "seven trips instead of eight (set x, 6)",           [S("set x, 7", "7", "6")]),
 ("A10", A, "out exec, 16 -> 15 (first one)",                   [S("out exec, 16", "16", "15")]),
 ("A11", A, "jmp x-- loop -> jmp !x loop",                      [S("jmp x-- loop", "x--", "!x")]),
 ("B1", B, "pad 8 side value 1 -> 2",                           [S("nop side 1 [3]", "side 1", "side 2")]),
 ("B2", B, "both pads side 3 -> 1",                             [S("nop side 3 [3]", "side 3", "side 1")]),
 ("B3", B, "first in pins, 2 -> 1",                             [S("in pins, 2", "2", "1")]),
 ("B4", B, "out x, 8 -> 7 (autopull)",                          [S("out x, 8", "8", "7")]),
 ("B5", B, "in x, 8 -> 7 (autopush)",                           [S("in x, 8", "8", "7")]),
 ("B6", B, "irq set 0 rel -> irq set 0 (flag 0, not 1)",        [S("irq set 0 rel", " rel", "")]),
 ("R1", SM, "RTL: mov invert does nothing",                     [S("2'd1:    mv = ~srcv;", "~srcv", "srcv")]),
 ("R2", SM, "RTL: mov bit-reverse does nothing",                [S("2'd2:    mv = bitrev32(srcv);", "bitrev32(srcv)", "srcv")]),
 ("R3", SM, "RTL: in shifts the wrong way",                     [S("n_isr = in_shiftdir ?", "in_shiftdir ?", "!in_shiftdir ?")]),
 ("R4", SM, "RTL: autopush threshold off by one",               [S("if (autopush && n_isr_cnt >= push_thr)", ">=", ">")]),
 ("R5", SM, "RTL: out shifts the wrong way",                    [S("dat = out_shiftdir ?", "out_shiftdir ?", "!out_shiftdir ?")]),
 ("R6", SM, "RTL: autopull threshold off by one",               [S("if (autopull && osr_cnt >= pull_thr) begin", ">=", ">")]),
 ("R7", SM, "RTL: out y writes x",                              [S("3'd2: n_y = dat;", "n_y", "n_x")]),
 ("R8", SM, "RTL: out exec does nothing",                       [S("3'd7: begin set_exec = 1'b1; set_exec_instr = dat[15:0]; end", "1'b1", "1'b0")]),
 ("R9", SM, "RTL: jmp x!=y inverted",                           [S("3'd5: take = (x != y);", "x != y", "x == y")]),
 ("R10", SM, "RTL: jmp !x inverted",                            [S("3'd1: take = (x == 32'd0);", "==", "!=")]),
 ("R11", SM, "RTL: jmp pin inverted",                           [S("3'd6: take = pins_in[jmp_pin];", "pins_in[jmp_pin]", "!pins_in[jmp_pin]")]),
 ("R12", SM, "RTL: jmp y-- never decrements",                   [S("3'd4: begin take = (y != 32'd0)", "n_y = y - 32'd1;", "")]),
 ("R13", SM, "RTL: wait gpio compares the wrong polarity",      [S("met   = (pin_b == arg[7]);", "==", "!=")]),
 ("R14", SM, "RTL: mov status always zero",                     [S("wire [31:0] status_v", "32'hFFFF_FFFF", "32'h0")]),
 ("R15", SM, "RTL: status uses the TX level instead of RX",     [S("wire        status_hit = status_sel", "status_sel ?", "!status_sel ?")]),
 ("R16", SM, "RTL: side-set drives pin values, not PINDIRS",    [S("wire s_d_en = side_on &&  side_pindir;", "side_pindir", "!side_pindir")]),
 ("R17", SM, "RTL: side-set enable bit ignored",                [S("wire       side_on", "side_fld[val_bits]", "1'b1")]),
 ("R18", SM, "RTL: autopush sends the old ISR",                 [S("c_rx_push = 1'b1; c_rx_data = n_isr;", "n_isr;", "isr;")]),
 ("R19", SM, "RTL: relative IRQ ignores the SM number",         [S("wire [2:0]  irq_idx", "(arg[1:0] + SM_ID)", "arg[1:0]")]),
 ("R20", SM, "RTL: wait pin ignores IN_BASE",                   [S("pin_b = pins_in[in_base + arg[3:0]];", "in_base + ", "")]),
 ("R21", SM, "RTL: set pins ignores the 5-bit mask (writes 8)", [S("wire [15:0] setm", "{1'b0, setc}", "4'd8")]),
 ("R22", PIO, "RTL: pad 8 not bypassing the synchroniser",      [S("wire [9:0]  pins_eff", "(pins_raw & bypass_q)", "(10'h0)")]),
 ("R23", SM, "RTL: IN pins ignores IN_BASE rotation",           [S("wire [15:0] pins_rot", "in_base", "4'd0")]),
]


def layer(T_):
    sh("rm -rf sim_build results.xml", TEST, 30)
    rc, out = sh("python3 build_pio_gl_sig.py && cp pio_gl_sig_flash_image.hex ../test/", T + "/tools", 60)
    if rc:
        return "error", "build: " + out[-160:].replace("\n", " ")
    rc, out = sh("GL_SIG_MAX_CLOCKS=200000 make COCOTB_TEST_FILTER=test_pio_postlayout_signature", TEST, 400)
    m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
    if rc == 124:
        return "hang", "[hang] test did not finish in 400 s"
    if not m:
        return "error", "no result line: " + out[-160:].replace("\n", " ")
    if m.group(3) == "0":
        return "pass", "%s test" % m.group(1)
    a = re.findall(r"(AssertionError[^\n]{0,70})", out)
    return "fail", (a[0] if a else "failed")


if __name__ == "__main__":
    sys.exit(sweep(T, M, [("cocotb test_pio_postlayout_signature", layer)], [A, B, SM, PIO], sys.argv[1:],
                   "Post-layout signature test mutation sweep (%d mutants)" % len(M)))
