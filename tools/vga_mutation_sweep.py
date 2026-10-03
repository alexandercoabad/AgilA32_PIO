#!/usr/bin/env python3
"""vga_mutation_sweep.py -- do the VGA testbenches notice a broken VGA program?

Applies ONE deliberate single-line break at a time to pio/vga_frame.pio or pio/vga_line.pio and runs
  layer 1  test/tb_pio_vga.v      (pio.v alone, VGA monitor model, ~15 s)
  layer 2  test/tb_pio_cpu_vga.v  (real CPU + top level, firmware rebuilt from the mutated programs) -- only for
                                  mutants layer 1 missed, or for all with --all-layers
and reports which mutants were caught.  Every mutant keeps the instruction count (23 + 9 = 32 words: the line
program is loaded at word 23), a deleted instruction becomes a `nop`.  Exit status 1 if anything survived or hung.

It works on a COPY of the project in a temp directory -- the working tree is never modified.  (tb_pio_vga.v reads
its programs from /tmp/pio_vga_frame.hex and /tmp/pio_vga_line.hex, so do not run two sweeps at once.)

    python3 tools/vga_mutation_sweep.py                  # all mutants (~15 min)
    python3 tools/vga_mutation_sweep.py V3 L4            # just these
    python3 tools/vga_mutation_sweep.py --list
    python3 tools/vga_mutation_sweep.py --all-layers     # run the CPU-level bench for every mutant too
"""
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mutation_common import S, D, N, copy_repo, sh, sweep

T = copy_repo("vga_mut_")
FR, LN = T + "/pio/vga_frame.pio", T + "/pio/vga_line.pio"
TEST, SRC = T + "/test", T + "/src"
PIO = "%s/pio.v %s/pio_sm.v %s/pio_fifo.v" % (SRC, SRC, SRC)
TOP = "%s/tt_um_agila32.v %s %s/rv32i_core.v %s/mem.v %s/qspi_shared_engine.v spi_ram_model.v" % (SRC, PIO, SRC, SRC, SRC)

M = [  # (id, file, description, ops)   -- frame program (SM0) first
 ("V1",  FR, "back porch 32 lines instead of 33",              [S("set x, 31", "31", "30")]),
 ("V2",  FR, "VSYNC lasts one line (second wait removed)",     [N("VSYNC line 2")]),
 ("V3",  FR, "VSYNC never goes low",                           [S("VSYNC low, black", "set pins, 0", "set pins, 8")]),
 ("V4",  FR, "VSYNC stuck low until the first picture line",   [S("VSYNC high, black", "set pins, 8", "set pins, 0")]),
 ("V5",  FR, "picture has 448 lines (outer count 13)",         [S("set x, 14", "14", "13")]),
 ("V6",  FR, "picture has 479 lines (inner count 30)",         [S("set y, 31", "31", "30")]),
 ("V7",  FR, "front porch 8 lines instead of 9",               [S("set x, 8", "8", "7")]),
 ("V8",  FR, "bar is 79 ticks (15-tick part 14)",              [S("out pins, 3 [14]", "[14]", "[13]")]),
 ("V9",  FR, "bar is 79 ticks (first 32-tick nop 31)",         [S("nop [31]", "[31]", "[30]", 0)]),
 ("V10", FR, "bar is 81 ticks (delay on the loop jump)", [S("jmp !osre bar", "jmp !osre bar", "jmp !osre bar [1]")]),
 ("V11", FR, "palette not reloaded each line (mov osr, isr -> nop)", [N("mov osr, isr")]),
 ("V12", FR, "no black after the last bar",                    [N("black again")]),
 ("V13", FR, "palette shifted by one bit per bar (out pins, 2)", [S("out pins, 3 [14]", "out pins, 3", "out pins, 2")]),
 ("V14", FR, "visible lines not counted (jmp y-- becomes jmp)", [S("jmp y-- vis", "jmp y-- vis", "jmp vis")]),
 ("L1",  LN, "HSYNC low 95 ticks",                             [S("set pins, 0 [31]", "[31]", "[30]", 2)]),
 ("L2",  LN, "HSYNC low 128 ticks (back-porch start late)",    [S("set pins, 1 [31]", "set pins, 1", "set pins, 0")]),
 ("L3",  LN, "HSYNC pulse split by a high tick-block",         [S("set pins, 0 [31]", "set pins, 0", "set pins, 1", 1)]),
 ("L4",  LN, "line is 799 ticks (set y delay 6)",              [S("set y, 23 [7]", "[7]", "[6]")]),
 ("L5",  LN, "line is 24 ticks short (nop [24])",              [S("nop [25]", "[25]", "[24]")]),
 ("L6",  LN, "line IRQ one tick late",                         [S("set pins, 1 [14]", "[14]", "[15]")]),
 ("L7",  LN, "line IRQ never raised",                          [N("irq set 0")]),
 ("L8",  LN, "line IRQ is flag 1 (frame program never sees it)", [S("irq set 0", "irq set 0", "irq set 1")]),
 ("L9",  LN, "HSYNC never goes high (all pulses low)",         [S("set pins, 1 [31]", "set pins, 1", "set pins, 0"), S("set pins, 1 [14]", "set pins, 1", "set pins, 0")]),
]
FILES = [FR, LN]
cache = {}


def compile_once(name, srcs):
    """iverilog the bench once per sweep; the programs are read at run time, so no recompile per mutant."""
    if name not in cache:
        rc, out = sh("iverilog -g2012 -I %s -o %s/%s.vvp %s" % (SRC, T, name, srcs), TEST, 300)
        cache[name] = (rc == 0, out)
    return cache[name]


def layer_pio(T_):
    ok, out = compile_once("tb_pio_vga", "%s tb_pio_vga.v" % PIO)
    if not ok:
        return "error", "compile: " + out[-200:]
    a = sh("python3 %s/tools/pioasm.py %s/pio/vga_frame.pio --format hex > /tmp/pio_vga_frame.hex" % (T, T), T, 60)
    b = sh("python3 %s/tools/pioasm.py %s/pio/vga_line.pio --origin 23 --format hex > /tmp/pio_vga_line.hex" % (T, T), T, 60)
    if a[0] or b[0]:
        return "error", "pioasm: " + (a[1] + b[1])[-200:]
    nf = sum(1 for l in open("/tmp/pio_vga_frame.hex") if l.strip())
    nl = sum(1 for l in open("/tmp/pio_vga_line.hex") if l.strip())
    if (nf, nl) != (23, 9):
        return "error", "mutant changed the instruction count (%d + %d words, need 23 + 9)" % (nf, nl)
    rc, out = sh("vvp %s/tb_pio_vga.vvp" % T, TEST, 150)
    if rc == 124:
        return "hang", "[hang] tb_pio_vga did not finish in 150 s"
    if "ALL TESTS PASSED" in out and "FAIL" not in out and "TIMEOUT" not in out:
        return "pass", ""
    msg = [l.strip() for l in out.splitlines() if l.strip().startswith(("FAIL", "TIMEOUT"))]
    return "fail", (msg[0] if msg else "no PASS line")[:70]


def layer_cpu(T_):
    rc, out = sh("python3 build_pio_vga.py", T + "/tools", 120)
    if rc:
        return "error", "build_pio_vga.py: " + out[-200:]
    shutil.copy(T + "/tools/pio_vga_flash_image.hex", TEST + "/pio_vga_flash_image.hex")
    ok, cout = compile_once("tb_pio_cpu_vga", "%s tb_pio_cpu_vga.v" % TOP)
    if not ok:
        return "error", "compile: " + cout[-200:]
    rc, out = sh("vvp %s/tb_pio_cpu_vga.vvp" % T, TEST, 900)
    if rc == 124:
        return "hang", "[hang] tb_pio_cpu_vga did not finish in 900 s"
    if "PASS tb_pio_cpu_vga" in out and "FAIL" not in out:
        return "pass", ""
    msg = [l.strip() for l in out.splitlines() if l.strip().startswith(("FAIL", "TIMEOUT"))]
    return "fail", (msg[0] if msg else "no PASS line")[:70]


if __name__ == "__main__":
    sys.exit(sweep(T, M, [("tb_pio_vga", layer_pio), ("tb_pio_cpu_vga", layer_cpu)], FILES, sys.argv[1:],
                   "VGA mutation sweep (%d mutants)" % len(M)))
