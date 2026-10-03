#!/usr/bin/env python3
"""i2c_slave_mutation_sweep.py -- do the I2C slave tests actually notice a broken slave program?

Applies ONE deliberate single-line break at a time to pio/i2c_slave_rx.pio or pio/i2c_slave_tx.pio, runs the
slave cocotb tests (test_i2c_slave*), and reports which tests failed.  A mutant that no test notices
("SURVIVED") means a hole in the tests; a mutant noticed only by a timeout ("HANG") means the failure would be
slow and uninformative.  Exit status 1 if anything survived or hung.

It works on a COPY of the project in a temp directory -- the working tree is never modified.

    python3 tools/i2c_slave_mutation_sweep.py            # all mutants (~12 min)
    python3 tools/i2c_slave_mutation_sweep.py R15 T4     # just these
    python3 tools/i2c_slave_mutation_sweep.py --list
"""
import os, re, shutil, signal, subprocess, sys, tempfile, time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
T = tempfile.mkdtemp(prefix="i2c_mut_")
shutil.copytree(REPO, T, dirs_exist_ok=True,
                ignore=shutil.ignore_patterns(".git", "sim_build", "__pycache__", "results.xml", "*.pyc"))
RX, TX = T + "/pio/i2c_slave_rx.pio", T + "/pio/i2c_slave_tx.pio"
PRISTINE = REPO + "/pio/"

def find(lines, sub, nth=0):
    idx = [i for i, l in enumerate(lines) if sub in l and not l.lstrip().startswith(';')]
    assert len(idx) > nth, ("selector not found", sub, idx)
    return idx[nth]

def D(sub, nth=0):  return ("del", sub, nth, None)
def R(sub, new, nth=0): return ("rep", sub, nth, new)

M = [  # (id, file, description, ops)
 ("R1",  RX, "no address compare (ACK every address)",                [D("jmp x!=y idle")]),
 ("R2",  RX, "ignore R/W (ACK read requests too)",                    [D("read request")]),
 ("R3",  RX, "accept one byte too many",                              [R("set x, WRITE_BYTES + 1", "    set x, WRITE_BYTES + 2")]),
 ("R4",  RX, "accept one byte too few",                               [R("set x, WRITE_BYTES + 1", "    set x, WRITE_BYTES")]),
 ("R5",  RX, "never push received bytes",                             [D("data byte -> host")]),
 ("R6",  RX, "never drive ACK (SDA stays released)",                  [R("ACK: pull SDA low", "    set pindirs, 0")]),
 ("R7",  RX, "never release SDA after ACK",                           [D("set pindirs, 0")]),
 ("R8",  RX, "address shift off by one (out null, 25)",               [R("out null, 24", "    out null, 25")]),
 ("R9",  RX, "any SDA fall counts as START (no SCL-high check)",      [R("jmp pin, start", "    jmp start")]),
 ("R10", RX, "bit counter not reset per byte",                        [D("osr shift count")]),
 ("R11", RX, "sample SDA at the falling edge instead of the rising",  "swap_rx_sample"),
 ("R12", RX, "never finish (chk loops back to rxbyte)",               [R("jmp !x idle", "    jmp rxbyte")]),
 ("R13", RX, "no address/data dispatch (all bytes treated as data)",  [D("jmp !x isaddr")]),
 ("R14", RX, "no wait for SCL low after START",                       [D("START hold")]),
 ("R15", RX, "START detector skips the 'SDA high' precondition",      [D("SDA high ...")]),
 ("T1",  TX, "no address compare",                                    [D("not us")]),
 ("T2",  TX, "ACK write requests too",                                [D("write request")]),
 ("T3",  TX, "never ACK the address",                                 [R("ACK the address", "    set pindirs, 0")]),
 ("T4",  TX, "no clock stretching (plain pull)",                      [R("pull block side SCL_LOW", "    pull block")]),
 ("T5",  TX, "never release SDA for the master's ACK",                [D("release SDA for the master's ACK/NAK")]),
 ("T6",  TX, "ignore the master's NAK (keep sending)",                [D("NAK: the master is done")]),
 ("T7",  TX, "wrong ACK bit extraction (no bit reverse)",             [R("mov osr, ::isr", "    mov osr, isr")]),
 ("T8",  TX, "address shift off by one (out null, 25)",               [R("out null, 24", "    out null, 25")]),
 ("T9",  TX, "data setup: release SCL in the same tick as the bit",   "revert_setup"),
 ("T10", TX, "never release SCL after a stretch",                     [R("side SCL_HIGH", "    wait 1 pin, 1")]),
 ("T11", TX, "no wait for SCL low after the master's ACK clock",      [D("wait 0 pin, 1", 3)]),
 ("T12", TX, "any SDA fall counts as START",                          [R("jmp pin, start", "    jmp start")]),
 ("T13", TX, "bit counter not reset (no mov osr, null)",              [D("mov osr, null")]),
 ("T14", TX, "inverted NAK sense (stop on ACK)",                      [R("NAK: the master is done", "    jmp !x idle")]),
]

def apply(path, spec):
    lines = open(path).read().split("\n")
    if spec == "swap_rx_sample":
        i = find(lines, "sample SDA on the rising edge"); del lines[i]
        j = find(lines, "wait 0 pin, 1", 1) if False else None
        # after deletion, the 'wait 0 pin, 1' that followed it is at index i
        assert "wait 0 pin, 1" in lines[i]; lines.insert(i+1, "    in pins, 1")
    elif spec == "revert_setup":
        i = find(lines, "present the bit while SCL is still held low")
        lines[i] = "    out pindirs, 1 side SCL_HIGH"
        j = find(lines, "side SCL_HIGH   ;"); lines[j] = "    wait 1 pin, 1"
    else:
        for kind, sub, nth, new in spec:
            i = find(lines, sub, nth)
            if kind == "del": del lines[i]
            else: lines[i] = new
    open(path, "w").write("\n".join(lines))


def run():
    subprocess.run("rm -rf sim_build results.xml", shell=True, cwd=T + "/test")
    t0 = time.time()
    p = subprocess.Popen("make -f Makefile.proto COCOTB_TEST_FILTER=test_i2c_slave", shell=True, cwd=T + "/test",
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    try:
        out, _ = p.communicate(timeout=120); rc = p.returncode
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL); out, _ = p.communicate(); rc = 124
    failed = re.findall(r"\*\* test_pio_protocols\.(\S+)\s+FAIL", out)
    m = re.search(r"TESTS=(\d+) PASS=(\d+) FAIL=(\d+)", out)
    return dict(rc=rc, failed=failed, tot=m.groups() if m else None, secs=round(time.time() - t0))


def main():
    args = sys.argv[1:]
    if "--list" in args:
        for mid, path, desc, _ in M: print("%-4s %s  %s" % (mid, os.path.basename(path), desc))
        return 0
    for p, n in ((RX, "i2c_slave_rx.pio"), (TX, "i2c_slave_tx.pio")):      # the copy must be the real thing
        assert open(p).read() == open(PRISTINE + n).read()
    bak = {p: open(p).read() for p in (RX, TX)}
    base = run()
    print("baseline: %s tests, %s pass, %s fail" % base["tot"] if base["tot"] else "baseline: BUILD/RUN ERROR", flush=True)
    if not base["tot"] or base["failed"]:
        print("the unmutated tests must pass first"); return 2
    bad = []
    for mid, path, desc, spec in M:
        if args and mid not in args: continue
        try:
            apply(path, spec); r = run()
        except AssertionError as e:
            print("%-4s DID NOT APPLY (%s) -- fix the mutation" % (mid, e)); bad.append(mid); continue
        finally:
            for p in bak: open(p, "w").write(bak[p])
        if r["rc"] == 124:      verdict = "HANG    "; bad.append(mid)
        elif not r["failed"]:   verdict = "SURVIVED"; bad.append(mid)
        else:                   verdict = "caught  "
        print("%-4s %s %-58s %s" % (mid, verdict, desc, ", ".join(f.replace("test_i2c_slave_", "") for f in r["failed"])[:90]), flush=True)
    assert all(open(p).read() == open(PRISTINE + os.path.basename(p)).read() for p in (RX, TX))
    shutil.rmtree(T, ignore_errors=True)
    print("\n%s" % ("ALL MUTANTS CAUGHT" if not bad else "PROBLEMS: %s" % bad))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
