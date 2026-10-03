#!/usr/bin/env python3
"""onewire_mutation_sweep.py -- do the 1-Wire tests notice a broken 1-Wire master program?

Applies ONE deliberate single-line break at a time to pio/onewire.pio and runs
  layer 1  the seven cocotb tests test_onewire_* (test/test_pio_protocols.py), fastest first, stopping at the first
           failing test (use --full to run all seven for every mutant and see which ones notice it)
  layer 2  test/tb_pio_cpu_onewire.v (real CPU + top level, firmware rebuilt from the mutated program) -- only for
           mutants layer 1 missed, or for all with --all-layers
and reports which mutants were caught.  Exit status 1 if anything survived or hung.  A SURVIVED mutant is either a
hole in the tests or an *equivalent* mutant (the change is still inside the 1-Wire timing windows); decide which,
and write it down in CHANGES_feature15.md / docs/info.md.

It works on a COPY of the project in a temp directory -- the working tree is never modified.

    python3 tools/onewire_mutation_sweep.py              # all mutants (a surviving mutant costs ~2.5 min)
    python3 tools/onewire_mutation_sweep.py O6 O12       # just these
    python3 tools/onewire_mutation_sweep.py --list
    python3 tools/onewire_mutation_sweep.py --full --all-layers
"""
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, D, N, copy_repo, sh, sweep

T = copy_repo("ow_mut_")
OW = T + "/pio/onewire.pio"
TEST, SRC = T + "/test", T + "/src"
PIO = "%s/pio.v %s/pio_sm.v %s/pio_fifo.v" % (SRC, SRC, SRC)
TOP = "%s/tt_um_agila32.v %s %s/rv32i_core.v %s/mem.v %s/qspi_shared_engine.v spi_ram_model.v" % (SRC, PIO, SRC, SRC, SRC)

# Not testable with this model (so not in the list): sampling a read EARLY, e.g. 7 us after the fall instead of 13
# (set pindirs, 0 [3]).  The spec allows any sample point from the line's rise time to 15 us, and the model's pull-up
# is ideal (the line is high the same clock the master releases it), so such a master is indistinguishable from the
# real one here; on hardware the rise time (1-5 us) sets the earliest safe point.  It stays a documented limit.
# fastest first (baseline seconds): absent 3, write-slot 3, partial 3, reset 12, read-rom 22, hold 54, tolerance 57
TESTS = ["test_onewire_absent_device", "test_onewire_write_slot_timing", "test_onewire_partial_bit_counts",
         "test_onewire_reset_and_presence", "test_onewire_read_rom", "test_onewire_read_hold_margin",
         "test_onewire_clock_tolerance"]

M = [  # (id, file, description, ops)       tick = 1 us
 ("O1",  OW, "reset low 464 us (x 13), below the 480 us minimum",    [S("set x, 15", "15", "13")]),
 ("O2",  OW, "reset never releases DQ",                             [S("set pindirs, 0 [31]", "set pindirs, 0", "nop")]),
 ("O3",  OW, "presence sampled 37 us after release (was 68)",       [S("nop [31]", "[31]", "[0]", 3)]),
 ("O4",  OW, "presence sampled too late (nop [31])",                [S("nop [3]", "[3]", "[31]")]),
 ("O5",  OW, "presence never sampled (in null)",                    [S("sample presence", "in pins, 1", "in null, 1")]),
 ("O6",  OW, "reset recovery 3 x 33 us (< 480 us after release)",   [S("set x, 12", "12", "2")]),
 ("O7",  OW, "no RX word after a reset",                            [D("push block", 1)]),
 ("O8",  OW, "no RX word after a transfer",                         [D("push block", 0)]),
 ("O9",  OW, "write-1/read low 15 us instead of 3",                 [S("set pindirs, 1 [2]", "[2]", "[14]")]),
 ("O10", OW, "write-1/read low 1 us instead of 3",                  [S("set pindirs, 1 [2]", "[2]", "[0]")]),
 ("O11", OW, "read sampled 20 us after the fall (release [14])",    [S("set pindirs, 0 [9]", "[9]", "[14]")]),
 ("O13", OW, "DQ never released after the write-1/read low",        [S("set pindirs, 0 [9]", "set pindirs, 0", "nop")]),
 ("O14", OW, "reads returned as zeros (in null)",                   [S("in pins, 1 [31]", "in pins, 1", "in null, 1")]),
 ("O15", OW, "slot 55 us instead of 70 (nop [9])",                  [S("nop [24]", "[24]", "[9]")]),
 ("O16", OW, "write-0 low 33 us (second nop removed)",              [D("nop [31]", 1)]),
 ("O17", OW, "write-0 low 59 us (first delay [25])",                [S("set pindirs, 1 [31]", "[31]", "[25]")]),
 ("O18", OW, "write-0 never releases DQ",                           [S("release, 5 us", "set pindirs, 0", "nop")]),
 ("O19", OW, "write-0 slot without the dummy IN (bit count off)",   [D("keeps the bit count aligned")]),
 ("O20", OW, "bit count n instead of n-1 (out y, 4)",               [S("out y, 5", "5", "4")]),
 ("O21", OW, "op bit inverted (reset <-> transfer)",                [S("jmp !x do_reset", "jmp !x", "jmp x--")]),
 ("O22", OW, "data bit inverted (write 0 <-> 1)",                   [S("jmp !x slot0", "jmp !x", "jmp x--")]),
 ("O23", OW, "bit loop never ends",                                 [S("jmp y-- bit", "jmp y-- bit", "jmp bit")]),
 ("O24", OW, "data shifted MSB first-style (out x, 2: skips bits)", [S("out x, 1", "out x, 1", "out x, 2", 1)]),
]
cache = {}


def layer_cocotb(T_, full=False):
    failed = []
    for t in TESTS:
        sh("rm -rf sim_build results.xml", TEST, 30)
        rc, out = sh("make -f Makefile.proto COCOTB_TEST_FILTER=%s" % t, TEST, 240)
        m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
        if rc == 124:
            failed.append(t + " [hang]")
        elif not m:
            return "error", "%s: no result line: %s" % (t, out[-160:].replace("\n", " "))
        elif m.group(3) != "0":
            failed.append(t)
        if failed and not full:
            break
    if not failed:
        return "pass", ""
    names = ", ".join(f.replace("test_onewire_", "") for f in failed)
    return ("hang" if all("[hang]" in f for f in failed) else "fail"), names


def layer_cpu(T_):
    rc, out = sh("python3 build_pio_onewire.py", T + "/tools", 120)
    if rc:
        return "error", "build_pio_onewire.py: " + out[-200:]
    shutil.copy(T + "/tools/pio_onewire_flash_image.hex", TEST + "/pio_onewire_flash_image.hex")
    if "cpu" not in cache:
        rc, cout = sh("iverilog -g2012 -I %s -o %s/tb_pio_cpu_onewire.vvp %s tb_pio_cpu_onewire.v" % (SRC, T, TOP), TEST, 300)
        cache["cpu"] = (rc == 0, cout)
    if not cache["cpu"][0]:
        return "error", "compile: " + cache["cpu"][1][-200:]
    rc, out = sh("vvp %s/tb_pio_cpu_onewire.vvp" % T, TEST, 900)
    if rc == 124:
        return "hang", "[hang] tb_pio_cpu_onewire did not finish in 900 s"
    if "PASS tb_pio_cpu_onewire" in out and "FAIL" not in out:
        return "pass", ""
    msg = [l.strip() for l in out.splitlines() if l.strip().startswith(("FAIL", "TIMEOUT"))]
    return "fail", (msg[0] if msg else "no PASS line")[:70]


if __name__ == "__main__":
    full = "--full" in sys.argv
    sys.exit(sweep(T, M, [("cocotb test_onewire_*", lambda t: layer_cocotb(t, full=full)),
                          ("tb_pio_cpu_onewire", layer_cpu)],
                   [OW], sys.argv[1:], "1-Wire mutation sweep (%d mutants)" % len(M)))
