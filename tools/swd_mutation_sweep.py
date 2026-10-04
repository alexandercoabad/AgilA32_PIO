#!/usr/bin/env python3
"""swd_mutation_sweep.py -- do the SWD tests (test/test_pio_swd.py, 18 tests) notice a broken pio/swd.pio?

Applies ONE deliberate one-line break at a time to pio/swd.pio (on a temp copy of the repo) and runs
`make -f Makefile.proto COCOTB_TEST_MODULES=test_pio_swd` (~25 s per mutant).

    python3 tools/swd_mutation_sweep.py            # all mutants
    python3 tools/swd_mutation_sweep.py S3 S7      # just these
    python3 tools/swd_mutation_sweep.py --list
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, N, copy_repo, sh, sweep

T = copy_repo("swd_mut_")
P = T + "/pio/swd.pio"
TEST = T + "/test"

M = [
 ("S1",  P, "read: sample one tick early (nop [1]) -> 7-tick bit",      [S("nop                  side 0 [2]", "[2]", "[1]")]),
 ("S2",  P, "read: sample one tick late (nop [3]) -> 9-tick bit",       [S("nop                  side 0 [2]", "[2]", "[3]")]),
 ("S3",  P, "read: SWCLK high only 3 ticks",                            [S("jmp x-- rbit", "[3]", "[2]")]),
 ("S4",  P, "read: SWCLK high 5 ticks",                                 [S("jmp x-- rbit", "[3]", "[4]")]),
 ("S5",  P, "write: SWCLK low only 3 ticks (no setup margin)",          [S("out pins, 1", "[3]", "[2]")]),
 ("S6",  P, "write: SWCLK high only 3 ticks",                           [S("jmp x-- wbit", "[3]", "[2]")]),
 ("S7",  P, "read: SWDIO stays an output (contention with target)",     [S("set pindirs, 0", "pindirs, 0", "pindirs, 1")]),
 ("S8",  P, "write: SWDIO never driven",                                [S("set pindirs, 1", "pindirs, 1", "pindirs, 0")]),
 ("S9",  P, "write: SWCLK idles high after the data",                   [S("push block", "side 0", "side 1", 1)]),
 ("S10", P, "header: read flag inverted",                               [S("jmp !y wr", "!y", "y--")]),
 ("S11", P, "header: pulse count only 4 bits (n <= 16)",                [S("out x, 5", "5", "4")]),
 ("S12", P, "read: nothing pushed",                                     [N("push block", 0)]),
 ("S13", P, "write: no completion word",                                [N("push block", 1)]),
 ("S14", P, "read: samples two pins (in pins, 2)",                      [S("in pins, 1", "1", "2")]),
 ("S15", P, "read: nothing sampled (in -> nop)",                        [N("in pins, 1")]),
 ("S16", P, "write: data shifted out two bits per clock",               [S("out pins, 1", "1", "2")]),
 ("S17", P, "header: SWCLK high while waiting for a command",           [S("pull block", "side 0", "side 1", 0)]),
 ("S18", P, "write: second pull missing (data = header leftovers)",     [N("pull block", 1)]),
 ("S19", P, "read: jmp back one instruction too far (jmp rbit -> cmd)", [S("jmp x-- rbit", "rbit", "cmd")]),
 ("S20", P, "write loop never ends early (jmp x-- -> jmp wbit)",        [S("jmp x-- wbit", "x-- wbit", "wbit")]),
]


def layer(T_):
    sh("rm -rf sim_build results.xml", TEST, 30)
    rc, out = sh("make -f Makefile.proto COCOTB_TEST_MODULES=test_pio_swd", TEST, 500)
    m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
    if rc == 124:
        return "hang", "[hang] tests did not finish in 500 s"
    if not m:
        return "error", "no result line: " + out[-160:].replace("\n", " ")
    if m.group(3) == "0":
        return "pass", "%s tests" % m.group(1)
    failed = re.findall(r"\*\* test_pio_swd\.test_swd_(\S+)\s+FAIL", out)
    return "fail", ", ".join(failed)[:90] if failed else "failed"


if __name__ == "__main__":
    sys.exit(sweep(T, M, [("cocotb test_pio_swd", layer)], [P], sys.argv[1:],
                   "SWD mutation sweep (%d mutants)" % len(M)))
