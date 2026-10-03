#!/usr/bin/env python3
"""ws2812_repeat_mutation_sweep.py -- do the tests notice a broken pio/ws2812_repeat.pio?

Applies ONE deliberate single-line break at a time and runs
  layer 1  the five cocotb tests test_ws2812_repeat_* (exact HIGH and LOW widths at every bit, pixel and run border,
           run lengths, colours, RGBW, latch between frames, the 300-pixel run with an empty FIFO)
  layer 2  test/tb_pio_cpu_ws2812_repeat.v (real CPU + top level, firmware rebuilt from the mutated program) -- only for
           mutants layer 1 missed, or for all with --all-layers
Exit status 1 if a mutant survived or hung.  Works on a COPY of the project.

    python3 tools/ws2812_repeat_mutation_sweep.py          # all mutants (~6 min)
    python3 tools/ws2812_repeat_mutation_sweep.py W5 W9
    python3 tools/ws2812_repeat_mutation_sweep.py --list
"""
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, D, N, copy_repo, sh, sweep

T = copy_repo("wsr_mut_")
P = T + "/pio/ws2812_repeat.pio"
TEST = T + "/test"

M = [  # (id, file, description, ops)
 ("W1",  P, "0-bit high pulse 2 ticks (T1 = 2)",                   [S("define public T1", "3", "2")]),
 ("W2",  P, "1-bit high pulse 6 ticks (T2 = 3)",                   [S("define public T2", "4", "3")]),
 ("W3",  P, "low after a 1 inside a pixel 2 ticks (T3 = 2)",       [S("define public T3", "3", "2")]),
 ("W4",  P, "0-bit low one tick short (nop [T2 - 3])",             [S("nop", "T2 - 2", "T2 - 3")]),
 ("W5",  P, "0-bit never ends the pixel (jmp !osre -> jmp)",       [S("jmp !osre bitloop", "jmp !osre bitloop", "jmp bitloop", 1)]),
 ("W6",  P, "1-bit never ends the pixel (jmp !osre -> jmp)",       [S("jmp !osre bitloop", "jmp !osre bitloop", "jmp bitloop", 0)]),
 ("W7",  P, "run never ends after a 1 (jmp y-- -> jmp)",           [S("jmp y-- pixel", "jmp y-- pixel", "jmp pixel", 0)]),
 ("W8",  P, "run never ends after a 0 (jmp y-- -> jmp)",           [S("jmp y-- pixel", "jmp y-- pixel", "jmp pixel", 1)]),
 ("W9",  P, "count word ignored (mov y, osr -> nop)",              [N("mov y, osr")]),
 ("W10", P, "pixel not stored (mov isr, osr -> nop)",              [N("mov isr, osr")]),
 ("W11", P, "pixel not reloaded (mov osr, isr -> nop)",            [N("mov osr, isr")]),
 ("W12", P, "first bit of a pixel skips one (out x, 2)",           [S("out x, 1", "out x, 1", "out x, 2", 0)]),
 ("W13", P, "border without the short jump (jmp dispatch -> bitloop)", [S("jmp dispatch", "jmp dispatch", "jmp bitloop")]),
 ("W14", P, "line left HIGH while idle (jmp cmd side 1)",          [S("jmp cmd", "side 0", "side 1")]),
 ("W15", P, "line HIGH while waiting for a command",               [S("pull block", "side 0", "side 1", 0)]),
 ("W16", P, "bit loop low one tick short (out [T3 - 2])",          [S("out x, 1", "T3 - 1", "T3 - 2", 1)]),
 ("W17", P, "wrong side-set on a 1's high (side 0)",               [S("jmp !osre bitloop", "side 1", "side 0", 0)]),
 ("W18", P, "pixel word taken from the count (second pull -> nop)", [N("pull block", 1)]),
]


def layer(T_):
    sh("rm -rf sim_build results.xml", TEST, 30)
    rc, out = sh("make -f Makefile.proto COCOTB_TEST_FILTER=test_ws2812_repeat", TEST, 300)
    m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
    if rc == 124:
        return "hang", "[hang] tests did not finish in 300 s"
    if not m:
        return "error", "no result line: " + out[-160:].replace("\n", " ")
    if m.group(3) == "0":
        return "pass", "%s tests" % m.group(1)
    failed = re.findall(r"\*\* test_pio_protocols\.test_ws2812_repeat_(\S+)\s+FAIL", out)
    return "fail", ", ".join(failed)[:90]


cache = {}


def layer_cpu(T_):
    rc, out = sh("python3 build_pio_ws2812_repeat.py", T + "/tools", 120)
    if rc:
        return "error", "build_pio_ws2812_repeat.py: " + out[-200:]
    shutil.copy(T + "/tools/pio_ws2812_repeat_flash_image.hex", TEST + "/pio_ws2812_repeat_flash_image.hex")
    if "cpu" not in cache:
        src = T + "/src"
        files = " ".join("%s/%s" % (src, f) for f in ("tt_um_agila32.v", "pio.v", "pio_sm.v", "pio_fifo.v", "rv32i_core.v",
                                                      "mem.v", "qspi_shared_engine.v")) + " spi_ram_model.v"
        rc, cout = sh("iverilog -g2012 -I %s -o %s/tb_ws2812_repeat.vvp %s tb_pio_cpu_ws2812_repeat.v" % (src, T, files), TEST, 300)
        cache["cpu"] = (rc == 0, cout)
    if not cache["cpu"][0]:
        return "error", "compile: " + cache["cpu"][1][-200:]
    rc, out = sh("vvp %s/tb_ws2812_repeat.vvp" % T, TEST, 600)
    if rc == 124:
        return "hang", "[hang] tb_pio_cpu_ws2812_repeat did not finish in 600 s"
    if "PASS tb_pio_cpu_ws2812_repeat" in out and "FAIL" not in out:
        return "pass", ""
    msg = [l.strip() for l in out.splitlines() if l.strip().startswith(("FAIL", "TIMEOUT"))]
    return "fail", (msg[0] if msg else "no PASS line")[:70]


if __name__ == "__main__":
    sys.exit(sweep(T, M, [("cocotb test_ws2812_repeat_*", layer), ("tb_pio_cpu_ws2812_repeat", layer_cpu)], [P], sys.argv[1:],
                   "WS2812 repeat-colour mutation sweep (%d mutants)" % len(M)))
