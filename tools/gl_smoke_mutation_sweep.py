#!/usr/bin/env python3
"""gl_smoke_mutation_sweep.py -- does the post-layout smoke test (test/test_pio_gl.py) notice a broken PIO?

Applies ONE one-line break at a time to pio/gl_echo.pio, pio/gl_pulses.pio or src/pio_sm.v, rebuilds the flash image
from the mutated programs and runs `make COCOTB_TEST_FILTER=test_pio_postlayout` (RTL, ~1-2 min each).
Works on a temp copy of the repo.

    python3 tools/gl_smoke_mutation_sweep.py            # all mutants
    python3 tools/gl_smoke_mutation_sweep.py R3 P2      # just these
    python3 tools/gl_smoke_mutation_sweep.py --list
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, N, copy_repo, sh, sweep

T = copy_repo("gl_mut_")
E, P, SM = T + "/pio/gl_echo.pio", T + "/pio/gl_pulses.pio", T + "/src/pio_sm.v"
TEST = T + "/test"

M = [
 ("E1", E, "echo does not invert (mov x, ~osr -> osr)",         [S("mov x, ~osr", "~osr", "osr")]),
 ("E2", E, "echo does not bit-reverse (mov isr, ::x -> x)",     [S("mov isr, ::x", "::x", "x")]),
 ("E3", E, "echo never pushes the result",                      [N("push block")]),
 ("E4", E, "echo never raises IRQ 0 (burst never starts)",      [N("irq set 0")]),
 ("E5", E, "echo raises IRQ 1 instead of 0",                    [S("irq set 0", "irq set 0", "irq set 1")]),
 ("P1", P, "burst of 3 pulses (set x, 2)",                      [S("set x, 3", "3", "2")]),
 ("P2", P, "high 3 clocks-units (set pins, 1 [2])",             [S("set pins, 1 [1]", "[1]", "[2]")]),
 ("P3", P, "low 1 unit (set pins, 0 [0])",                      [S("set pins, 0 [1]", " [1]", "")]),
 ("P4", P, "pad left high at the end (last set pins, 0 -> 1)",  [S("set pins, 0 [1]", "set pins, 0", "set pins, 1")]),
 ("P5", P, "no wait for IRQ 0",                                 [N("wait 1 irq 0")]),
 ("R1", SM, "RTL: mov invert does nothing",                     [S("2'd1:    mv = ~srcv;", "~srcv", "srcv")]),
 ("R2", SM, "RTL: mov bit-reverse does nothing",                [S("2'd2:    mv = bitrev32(srcv);", "bitrev32(srcv)", "srcv")]),
 ("R3", SM, "RTL: jmp x-- never decrements",                    [S("3'd2: begin take = (x != 32'd0)", "n_x = x - 32'd1;", "")]),
 ("R4", SM, "RTL: jmp x-- inverted condition",                  [S("3'd2: begin take = (x != 32'd0)", "x != 32'd0", "x == 32'd0")]),
 ("R5", SM, "RTL: wait irq does not clear the flag",            [S("if (met && arg[7]) c_irq_clr", "met && arg[7]", "1'b0")]),
 ("R6", SM, "RTL: irq set does nothing",                        [S("c_irq_set[irq_idx] = 1'b1;", "1'b1", "1'b0")]),
]


def layer(T_):
    sh("rm -rf sim_build results.xml", TEST, 30)
    rc, out = sh("python3 build_pio_gl_smoke.py && cp pio_gl_smoke_flash_image.hex ../test/", T + "/tools", 60)
    if rc:
        return "error", "build: " + out[-160:].replace("\n", " ")
    rc, out = sh("GL_SMOKE_MAX_CLOCKS=150000 make COCOTB_TEST_FILTER=test_pio_postlayout", TEST, 400)
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
    sys.exit(sweep(T, M, [("cocotb test_pio_postlayout", layer)], [E, P, SM], sys.argv[1:],
                   "Post-layout smoke test mutation sweep (%d mutants)" % len(M)))
