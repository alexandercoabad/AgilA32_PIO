#!/usr/bin/env python3
"""can_mutation_sweep.py -- do the CAN tests (test/test_pio_can.py, 25 tests) notice a broken pio/can_tx.pio,
pio/can_rx.pio or tools/can_frame.py?

Applies ONE deliberate one-line break at a time (on a temp copy of the repo) and runs
`make -f Makefile.proto COCOTB_TEST_MODULES=test_pio_can` (~35 s per mutant).

    python3 tools/can_mutation_sweep.py            # all mutants
    python3 tools/can_mutation_sweep.py T3 R7      # just these
    python3 tools/can_mutation_sweep.py --list
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, N, D, copy_repo, sh, sweep

T = copy_repo("can_mut_")
TX = T + "/pio/can_tx.pio"
RX = T + "/pio/can_rx.pio"
FR = T + "/tools/can_frame.py"
TEST = T + "/test"

M = [
 # ---- pio/can_tx.pio
 ("T1",  TX, "bit loop one tick short (mov pins [9])",                [S("mov pins, y", "[10]", "[9]")]),
 ("T2",  TX, "bit loop one tick long (mov pins [11])",                [S("mov pins, y", "[10]", "[11]")]),
 ("T3",  TX, "compare: dominant test on x instead of y",              [S("jmp !y dom", "!y", "!x")]),
 ("T4",  TX, "recessive sent: readback never checked",                [N("jmp pin ok")]),
 ("T5",  TX, "dominant sent: readback never checked",                 [N("jmp pin abort")]),
 ("T6",  TX, "abort: TXD driven dominant instead of released",        [S("set pins, 1", "1", "0")]),
 ("T7",  TX, "abort: TXD not released",                               [N("set pins, 1")]),
 ("T8",  TX, "abort word not inverted (mov isr, x)",                  [S("mov isr, ~x", "~x", "x")]),
 ("T9",  TX, "abort: no result word",                                 [N("push block")]),
 ("T10", TX, "abort does not park (falls into the next check)",       [N("jmp park")]),
 ("T11", TX, "strict loop ends on !x instead of x--",                 [S("jmp x-- bit", "x--", "!x")]),
 ("T12", TX, "pull ifempty -> pull (a word per bit)",                 [S("pull ifempty block", "ifempty ", "")]),
 ("T13", TX, "header: only 8 bits of the count",                      [S("out x, 32", "32", "8")]),
 ("T14", TX, "header pull missing",                                   [N("pull block")]),
 ("T15", TX, "two data bits per bit time",                            [S("out y, 1", "1", "2")]),
 ("T16", TX, "TXD never driven",                                      [N("mov pins, y")]),
 ("T17", TX, "ACK slot one tick short",                               [S("set pins, 1          [10]", "[10]", "[9]")]),
 ("T18", TX, "tail one bit short (set x, 9)",                         [S("set x, 10", "10", "9")]),
 ("T19", TX, "tail one bit long (set x, 11)",                         [S("set x, 10", "10", "11")]),
 ("T20", TX, "ACK slot sampled two ticks late",                       [S("in pins, 1           [1]", "[1]", "[2]")]),
 ("T21", TX, "tail bit one tick short",                               [S("nop                  [13]", "[13]", "[12]")]),
 ("T22", TX, "ACK slot not recorded",                                 [N("in pins, 1", 0)]),
 ("T23", TX, "tail bits not recorded",                                [N("in pins, 1", 1)]),
 ("T24", TX, "no completion word",                                    [N("push block", 1)]),
 ("T25", TX, "tail samples two pins",                                 [S("in pins, 1", "1", "2", 1)]),
 ("T26", TX, "ACK slot driven dominant by the transmitter",           [S("set pins, 1          [10]", "set pins, 1", "set pins, 0")]),
 ("T27", TX, "tail loop runs forever (jmp tail)",                     [S("jmp x-- tail", "x-- tail", "tail")]),
 # ---- pio/can_rx.pio
 ("R1",  RX, "first sample one tick early",                           [S("wait 0 pin 0", "[9]", "[8]")]),
 ("R2",  RX, "first sample two ticks late (87 %+)",                   [S("wait 0 pin 0", "[9]", "[11]")]),
 ("R3",  RX, "waits for a rising edge instead of SOF",                [S("wait 0 pin 0", "0 pin", "1 pin")]),
 ("R4",  RX, "recessive test removed",                                [N("jmp pin rec")]),
 ("R5",  RX, "end after 10 recessive bits",                           [S("set y, 10", "10", "9")]),
 ("R6",  RX, "end after 12 recessive bits",                           [S("set y, 10", "10", "11")]),
 ("R7",  RX, "bit time 15 ticks",                                     [S("jmp x-- rbit", "[12]", "[11]")]),
 ("R8",  RX, "bit time 17 ticks",                                     [S("jmp x-- rbit", "[12]", "[13]")]),
 ("R9",  RX, "no partial word",                                       [N("push block", 0)]),
 ("R10", RX, "no tag word",                                           [N("push block", 1)]),
 ("R11", RX, "tag word is ~x",                                        [S("mov isr, x", "isr, x", "isr, ~x")]),
 ("R12", RX, "x not re-initialised after a frame",                    [N("mov x, ~null")]),
 ("R13", RX, "x initialised to 0",                                    [S("mov x, ~null", "~null", "null")]),
 ("R14", RX, "samples two pins",                                      [S("in pins, 1", "1", "2")]),
 ("R15", RX, "recessive counter tests !y",                            [S("jmp y-- back", "y--", "!y")]),
 ("R16", RX, "recessive path skips the bit counter",                  [S("jmp y-- back", "back", "rbit")]),
 ("R17", RX, "dominant sample does not reload the counter",           [N("set y, 10")]),
 # ---- tools/can_frame.py (host side)
 ("F1",  FR, "CRC polynomial 0x4598",                                 [S("CRC15_POLY = 0x4599", "0x4599", "0x4598")]),
 ("F2",  FR, "CRC register 14 bits",                                  [S("crc = (crc << 1) & 0x7FFF", "0x7FFF", "0x3FFF")]),
 ("F3",  FR, "stuffing after 4 equal bits",                           [S("if run == 5:", "5", "4")]),
 ("F4",  FR, "TX header counts n instead of n-1",                     [S("return [len(bits) - 1]", "len(bits) - 1", "len(bits)")]),
 ("F5",  FR, "partial capture word read one bit off",                 [S("(32 - m + k)", "32 - m", "31 - m")]),
 ("F6",  FR, "abort position off by one",                             [S("return n - 1 - r", "n - 1", "n")]),
 ("F7",  FR, "ACK bit inverted in the result",                        [S('"ack": bits[0] == 0', "== 0", "== 1")]),
 ("F8",  FR, "extended identifier low part misplaced",                [S("bits += _field(ident & 0x3FFFF, 18)", "0x3FFFF", "0x1FFFF")]),
]


def layer(T_):
    sh("rm -rf sim_build results.xml", TEST, 30)
    rc, out = sh("make -f Makefile.proto COCOTB_TEST_MODULES=test_pio_can", TEST, 400)
    m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
    if rc == 124:
        return "hang", "[hang] tests did not finish in 400 s"
    if not m:
        return "error", "no result line: " + out[-160:].replace("\n", " ")
    if m.group(3) == "0":
        return "pass", "%s tests" % m.group(1)
    failed = re.findall(r"\*\* test_pio_can\.test_can_(\S+)\s+FAIL", out)
    return "fail", ", ".join(failed)[:90] if failed else "failed"


if __name__ == "__main__":
    sys.exit(sweep(T, M, [("cocotb test_pio_can", layer)], [TX, RX, FR], sys.argv[1:],
                   "CAN mutation sweep (%d mutants)" % len(M)))
