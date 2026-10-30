#!/usr/bin/env python3
"""check_mutants.py -- proves the formal harnesses are not vacuous.
Breaks a COPY of a design file in several ways, one at a time, and checks that the harness REJECTS (FAIL) every broken copy
while accepting the real one.  Run from formal/:   python3 check_mutants.py [fifo|fifo_equiv|sm|sm_clkdiv]   (default: all)
Needs yosys and sby (abc pdr engine).  The real design files in ../src are never modified."""
import os, shutil, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, "..", "src")

SBY = """[options]
mode prove
aigsmt none
[engines]
abc pdr
[script]
{read}
prep -top {top}
techmap t:$shiftx t:$shift
setundef -anyseq
[files]
{files}
"""

FIFO_MUTANTS = [   # (name, old text, new text)
    ("push ignores full",           "wire do_push = push && !full;", "wire do_push = push;"),
    ("pop ignores empty",           "wire do_pop  = pop  && !empty;", "wire do_pop  = pop;"),
    ("write pointer stuck",         "if (do_push) wp <= wp + 1'b1;", "if (do_push) wp <= wp;"),
    ("read pointer skips a word",   "if (do_pop)  rp <= rp + 1'b1;", "if (do_pop)  rp <= rp + 2'd2;"),
    ("level up on pop",             "2'b01:   cnt <= cnt - 1'b1;", "2'b01:   cnt <= cnt + 1'b1;"),
    ("push+pop changes level",      "default: cnt <= cnt;", "default: cnt <= cnt + {{DEPTH_LOG2{1'b0}}, (do_push && do_pop)};"),
    ("clear forgets read pointer",  "            rp  <= {DEPTH_LOG2{1'b0}};\n            cnt <= {(DEPTH_LOG2+1){1'b0}};\n        end else begin",
                                    "            cnt <= {(DEPTH_LOG2+1){1'b0}};\n        end else begin"),
    ("data stored one slot late",   "if (do_push) mem[wp] <= wdata;", "if (do_push) mem[wp + 1'b1] <= wdata;"),
    ("head read from write slot",   "assign rdata = mem[rp];", "assign rdata = mem[wp];"),
    ("stored data corrupted",       "if (do_push) mem[wp] <= wdata;", "if (do_push) mem[wp] <= {wdata[31:1], 1'b0};"),
    ("full flag one early",         "assign full  = (cnt == DEPTH[DEPTH_LOG2:0]);", "assign full  = (cnt >= DEPTH[DEPTH_LOG2:0] - 1);"),
]

SM_MUTANTS = [
    ("non-blocking PULL pops empty", "end else if (tx_empty) begin", "end else if (1'b0) begin"),
    ("non-blocking PUSH ignores full", "end else if (rx_full) begin", "end else if (1'b0) begin"),
    ("IN autopush ignores full RX",  "                    if (rx_full) begin", "                    if (1'b0) begin"),
    ("PULL ignores empty TX",        "if (tx_empty) stall = 1'b1;", "if (1'b0) stall = 1'b1;"),
    ("OUT autopull ignores empty TX","n_osr_cnt >= pull_thr && !tx_empty", "n_osr_cnt >= pull_thr"),
    ("ISR counter not capped",       "n_isr_cnt = (sum > 7'd32) ? 6'd32 : sum[5:0];", "n_isr_cnt = sum[5:0];"),
    ("OSR counter not capped",       "n_osr_cnt = (sum > 7'd32) ? 6'd32 : sum[5:0];", "n_osr_cnt = sum[5:0];"),
    ("delay slot executes",          "wire exec_now = run && !use_delay;", "wire exec_now = run;"),
    ("IRQ flags not gated by tick",  "assign irq_set  = exec_now ? c_irq_set : 8'h0;", "assign irq_set  = c_irq_set;"),
    ("side-set drives when idle",    "wire [15:0] so_m = (exec_now && s_o_en) ? s_mask : 16'h0;", "wire [15:0] so_m = (s_o_en) ? s_mask : 16'h0;"),
    ("delay one tick too long",      "delay <= dly_val;", "delay <= dly_val + 5'd1;"),
    ("PC advances on a stall",       "                    if (done) begin\n                        delay <= dly_val;", "                    begin\n                        delay <= dly_val;"),
    ("forced instruction moves PC",  "else if (src_fetch) pc <= pc_inc;", "else pc <= pc_inc;"),
    ("restart keeps OSR counter",    "osr_cnt <= 6'd32; isr <= 32'h0; isr_cnt <= 6'd0;", "isr <= 32'h0; isr_cnt <= 6'd0;"),
    ("output value leaks past mask", "assign o_val  = (i_o_val & io_m & ~so_m) | (s_val & so_m);", "assign o_val  = (i_o_val & ~so_m) | (s_val & so_m);"),
]

CLKDIV_MUTANTS = [
    ("divider ticks one clock late", "wire        div_tick = (acc_n >= div_eff);", "wire        div_tick = (acc_n > div_eff);"),
    ("divider keeps phase on restart", "if (!en || clkdiv_restart)\n                acc <= 25'd0;", "if (!en)\n                acc <= 25'd0;"),
]

# harness name -> (design file, harness kind, props / extra file, top, extra read flags, mutants)
HARNESSES = {
    "fifo":        ("pio_fifo.v", "props", "fifo_props.vh", "pio_fifo", "-D FORMAL", FIFO_MUTANTS),
    "fifo_equiv":  ("pio_fifo.v", "equiv", "pio_fifo_formal.v", "pio_fifo_formal", "", FIFO_MUTANTS),
    "sm":          ("pio_sm.v", "props", "sm_props.vh", "pio_sm", "-D FORMAL", SM_MUTANTS),
    "sm_clkdiv":   ("pio_sm.v", "props", "sm_props.vh", "pio_sm", "-D FORMAL -D CLKDIV -D CLKDIV_SMALL -D CLKDIV_ACC -D CLKDIV_TICK", CLKDIV_MUTANTS),
}

def run(design, text, kind, extra, top, flags):
    d = tempfile.mkdtemp(prefix="fm_")
    try:
        open(os.path.join(d, design), "w").write(text)
        if kind == "props":
            shutil.copy(os.path.join(HERE, extra), d)
            top_v = design.replace(".v", "_formal.v")
            subprocess.check_call([sys.executable, os.path.join(HERE, "instrument.py"), os.path.join(d, design), extra, os.path.join(d, top_v)])
            sby = SBY.format(read="read -formal %s %s" % (flags, top_v), top=top, files="%s\n%s" % (top_v, extra))
        else:
            shutil.copy(os.path.join(HERE, extra), d)
            sby = SBY.format(read="read -formal %s %s" % (design, extra), top=top, files="%s\n%s" % (design, extra))
        open(os.path.join(d, "t.sby"), "w").write(sby)
        out = subprocess.run(["sby", "-f", "t.sby"], cwd=d, capture_output=True, text=True).stdout
        return "PASS" if "returned PASS" in out else "FAIL" if "returned FAIL" in out else "ERROR"
    finally:
        shutil.rmtree(d, ignore_errors=True)

bad = 0
for name in (sys.argv[1:] or list(HARNESSES)):
    design, kind, extra, top, flags, mutants = HARNESSES[name]
    text = open(os.path.join(SRC, design)).read()
    r = run(design, text, kind, extra, top, flags)
    print("%-10s real design: %s %s" % (name, r, "" if r == "PASS" else "<-- UNEXPECTED"))
    bad += r != "PASS"
    for mname, old, new in mutants:
        if old not in text:
            print("  mutant '%s': pattern not found in src/%s -- update check_mutants.py" % (mname, design)); bad += 1; continue
        r = run(design, text.replace(old, new, 1), kind, extra, top, flags)
        print("  mutant %-32s -> %s %s" % (mname, r, "(caught)" if r == "FAIL" else "<-- NOT CAUGHT" if r == "PASS" else "<-- tool error"))
        bad += r != "FAIL"
print("every harness accepts the real design and rejects every mutant" if not bad else "%d problem(s)" % bad)
sys.exit(1 if bad else 0)
