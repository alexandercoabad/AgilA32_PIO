"""mutation_common.py -- shared engine of the *_mutation_sweep.py scripts.

A sweep applies ONE deliberate single-line break at a time to a PIO program, on a COPY of the project in a temp
directory (the working tree is never modified), runs one or more test "layers" against it and reports

    caught    some test failed (the layers that noticed are listed)
    SURVIVED  every layer passed  -> a hole in the tests (or an equivalent mutant: say so in the docs)
    HANG      only our own time-out noticed it -> the failure would be slow and uninformative
    ERROR     the mutant did not build / the baseline output was not understood

Mutation operators (all keep or deliberately change the instruction count):
    S(select, old, new, nth=0)   replace text `old` by `new` inside the nth non-comment line containing `select`
    D(select, nth=0)             delete that line (the program gets one word shorter)
    N(select, nth=0)             replace that line by `nop` (the program keeps its length)
"""
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def copy_repo(prefix):
    t = tempfile.mkdtemp(prefix=prefix)
    shutil.copytree(REPO, t, dirs_exist_ok=True,
                    ignore=shutil.ignore_patterns(".git", "sim_build", "__pycache__", "results.xml", "*.pyc"))
    return t


def _find(lines, sub, nth):
    idx = [i for i, l in enumerate(lines) if sub in l and not l.lstrip().startswith(";")]
    assert len(idx) > nth, "selector %r (occurrence %d) not found, %d match(es)" % (sub, nth, len(idx))
    return idx[nth]


def S(select, old, new, nth=0): return ("sub", select, nth, old, new)
def D(select, nth=0):           return ("del", select, nth, None, None)
def N(select, nth=0):           return ("nop", select, nth, None, None)


def apply_ops(path, ops):
    lines = open(path).read().split("\n")
    for kind, sel, nth, old, new in ops:
        i = _find(lines, sel, nth)
        if kind == "del":
            del lines[i]
        elif kind == "nop":
            lines[i] = "    nop"
        else:
            assert old in lines[i], "%r not in line %r" % (old, lines[i])
            lines[i] = lines[i].replace(old, new, 1)
    open(path, "w").write("\n".join(lines))


def sh(cmd, cwd, timeout):
    """Run a shell command in its own process group. Returns (rc, output); rc 124 = our time-out."""
    p = subprocess.Popen(cmd, shell=True, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                         start_new_session=True)
    try:
        out, _ = p.communicate(timeout=timeout)
        return p.returncode, out
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        out, _ = p.communicate()
        return 124, out


def sweep(T, mutants, layers, files, argv, title, strip=""):
    """mutants: (id, file, description, ops); layers: [(name, fn(T) -> (status, detail))] where status is
    'pass' | 'fail' | 'hang' | 'error'.  Layers run in order; later ones only for mutants the earlier ones missed
    (unless --all-layers).  Returns the exit status."""
    args = [a for a in argv if not a.startswith("--")]
    if "--list" in argv:
        for mid, f, desc, _ in mutants:
            print("%-4s %-16s %s" % (mid, os.path.basename(f), desc))
        return 0
    bak = {f: open(f).read() for f in files}
    for f in files:
        assert open(f).read() == open(os.path.join(REPO, os.path.relpath(f, T))).read(), "copy differs from repo"
    print(title, flush=True)
    for name, fn in layers:                                    # the unmutated programs must pass every layer
        st, detail = fn(T)
        print("baseline %-26s %s %s" % (name, st, detail), flush=True)
        if st != "pass":
            print("the unmutated tests must pass first")
            shutil.rmtree(T, ignore_errors=True)
            return 2
    bad = []
    t_all = time.time()
    for mid, path, desc, ops in mutants:
        if args and mid not in args:
            continue
        t0 = time.time()
        noticed, verdict, hang = [], None, False
        try:
            apply_ops(path, ops)
            for name, fn in layers:
                st, detail = fn(T)
                if st == "error":
                    verdict = "ERROR   "
                    noticed.append("%s: %s" % (name, detail))
                    break
                if st == "hang":
                    hang = True
                if st in ("fail", "hang"):
                    noticed.append("%s: %s" % (name, detail))
                    if "--all-layers" not in argv:
                        break
        except AssertionError as e:
            verdict = "NOAPPLY "
            noticed.append(str(e))
        finally:
            for f in bak:
                open(f, "w").write(bak[f])
        if verdict is None:
            if not noticed:
                verdict = "SURVIVED"
            elif hang and all("[hang]" in n for n in noticed):
                verdict = "HANG    "
            else:
                verdict = "caught  "
        if verdict.strip() != "caught":
            bad.append(mid)
        print("%-4s %s %-56s %s (%ds)" % (mid, verdict, desc, "; ".join(noticed)[:110], time.time() - t0), flush=True)
    for f in files:
        assert open(f).read() == open(os.path.join(REPO, os.path.relpath(f, T))).read()
    shutil.rmtree(T, ignore_errors=True)
    print("\n%s  (%d s)" % ("ALL MUTANTS CAUGHT" if not bad else "PROBLEMS: %s" % bad, time.time() - t_all))
    return 1 if bad else 0
