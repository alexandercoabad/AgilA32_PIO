#!/usr/bin/env python3
"""sta.py -- quick pre-layout static timing estimate (register -> register) for a yosys
gate-level netlist (write_json) and a liberty corner.  NLDM lookup with input-slew and
fanout-load propagation; no wires beyond a per-sink cap estimate.  It is an ESTIMATE meant to
find which logic limits the clock; sign-off numbers come from the LibreLane/OpenSTA run."""
import json
import re
import sys
from bisect import bisect_right
from collections import defaultdict

WIRE_CAP_PER_SINK = 0.0025      # pF  (~2.5 fF per fanout: short local nets after placement)
WIRE_CAP_BASE = 0.0015


# ------------------------------------------------------------------ liberty parsing
TOK = re.compile(r'\s*(?:(/\*.*?\*/)|("(?:[^"\\]|\\.)*")|([{}();:,])|([^\s{}();:,"]+))', re.S)


def tokenize(text):
    text = text.replace("\\\n", " ")
    pos, n = 0, len(text)
    out = []
    while pos < n:
        m = TOK.match(text, pos)
        if not m:
            break
        pos = m.end()
        if m.group(1):
            continue
        out.append(m.group(2) or m.group(3) or m.group(4))
    return out


def parse_group(toks, i):
    """toks[i] is the group name; returns (node, next_i).  node = dict(kind,args,attrs,groups)"""
    kind = toks[i]
    i += 1
    args = []
    if toks[i] == "(":
        i += 1
        while toks[i] != ")":
            if toks[i] != ",":
                args.append(toks[i].strip('"'))
            i += 1
        i += 1
    node = {"kind": kind, "args": args, "attrs": {}, "groups": []}
    if i < len(toks) and toks[i] == "{":
        i += 1
        while toks[i] != "}":
            name = toks[i]
            nxt = toks[i + 1]
            if nxt == ":":
                j = i + 2
                val = []
                while toks[j] != ";":
                    val.append(toks[j])
                    j += 1
                node["attrs"][name] = " ".join(val).strip('"')
                i = j + 1
            elif nxt == "(":
                # either complex attribute  name(args);  or subgroup name(args){...}
                j = i + 2
                depth = 1
                while depth:
                    if toks[j] == "(":
                        depth += 1
                    elif toks[j] == ")":
                        depth -= 1
                    j += 1
                if j < len(toks) and toks[j] == "{":
                    sub, i = parse_group(toks, i)
                    node["groups"].append(sub)
                else:
                    a = [t.strip('"') for t in toks[i + 2:j - 1] if t != ","]
                    node["attrs"][name] = a
                    i = j + 1 if toks[j] == ";" else j
            else:
                i += 1
        i += 1
    elif i < len(toks) and toks[i] == ";":
        i += 1
    return node, i


def load_liberty(path):
    toks = tokenize(open(path).read())
    lib, _ = parse_group(toks, 0)
    return lib


def nums(s):
    return [float(x) for x in re.split(r"[\s,]+", s.strip()) if x]


class Table:
    def __init__(self, g, templates):
        self.i1 = nums(g["attrs"]["index_1"][0]) if "index_1" in g["attrs"] else None
        self.i2 = nums(g["attrs"]["index_2"][0]) if "index_2" in g["attrs"] else None
        rows = [nums(r) for r in g["attrs"]["values"]]
        tpl = templates.get(g["args"][0], {}) if g["args"] else {}
        if self.i1 is None:
            self.i1 = tpl.get("i1")
        if self.i2 is None:
            self.i2 = tpl.get("i2")
        self.v1 = tpl.get("v1", "input_net_transition")
        self.rows = rows

    def lookup(self, slew, load):
        if self.i2 is None:                       # 1-D
            return interp1(self.i1, self.rows[0], slew)
        if self.v1 == "input_net_transition" or self.v1 == "related_pin_transition":
            x, y = slew, load
        else:
            x, y = load, slew
        return interp2(self.i1, self.i2, self.rows, x, y)


def interp1(xs, ys, x):
    if len(xs) == 1:
        return ys[0]
    k = min(max(bisect_right(xs, x) - 1, 0), len(xs) - 2)
    t = (x - xs[k]) / (xs[k + 1] - xs[k])
    return ys[k] + t * (ys[k + 1] - ys[k])


def interp2(x1, x2, rows, a, b):
    k = min(max(bisect_right(x1, a) - 1, 0), len(x1) - 2)
    l = min(max(bisect_right(x2, b) - 1, 0), len(x2) - 2)
    ta = (a - x1[k]) / (x1[k + 1] - x1[k])
    tb = (b - x2[l]) / (x2[l + 1] - x2[l])
    v00, v01 = rows[k][l], rows[k][l + 1]
    v10, v11 = rows[k + 1][l], rows[k + 1][l + 1]
    return (v00 * (1 - ta) * (1 - tb) + v01 * (1 - ta) * tb + v10 * ta * (1 - tb) + v11 * ta * tb)


def build_cells(lib):
    templates = {}
    for g in lib["groups"]:
        if g["kind"] == "lu_table_template":
            t = {"v1": g["attrs"].get("variable_1", "input_net_transition")}
            if "index_1" in g["attrs"]:
                t["i1"] = nums(g["attrs"]["index_1"][0])
            if "index_2" in g["attrs"]:
                t["i2"] = nums(g["attrs"]["index_2"][0])
            templates[g["args"][0]] = t
    cells = {}
    for g in lib["groups"]:
        if g["kind"] != "cell":
            continue
        name = g["args"][0]
        c = {"caps": {}, "arcs": defaultdict(list), "seq": False, "setup": {}, "outs": set(), "ins": set()}
        for p in g["groups"]:
            if p["kind"] != "pin":
                continue
            pn = p["args"][0]
            direction = p["attrs"].get("direction", "")
            if direction == "input":
                c["ins"].add(pn)
                c["caps"][pn] = float(p["attrs"].get("capacitance", 0.0))
                if p["attrs"].get("clock", "") == "true":
                    c["clk"] = pn
            elif direction == "output":
                c["outs"].add(pn)
            for t in p["groups"]:
                if t["kind"] != "timing":
                    continue
                rel = t["attrs"].get("related_pin", "").strip('"')
                ttype = t["attrs"].get("timing_type", "combinational")
                tabs = {tg["kind"]: Table(tg, templates) for tg in t["groups"]
                        if tg["kind"] in ("cell_rise", "cell_fall", "rise_transition",
                                          "fall_transition", "rise_constraint", "fall_constraint")}
                if direction == "output":
                    if ttype in ("rising_edge", "falling_edge"):
                        c["seq"] = True
                    if "cell_rise" in tabs or "cell_fall" in tabs:
                        c["arcs"][pn].append((rel, ttype, tabs))
                elif ttype.startswith("setup"):
                    c["setup"][pn] = tabs
        cells[name] = c
    return cells


# ------------------------------------------------------------------ netlist analysis
def analyse(json_path, cells, top, report=8, split=None):
    nl = json.load(open(json_path))
    mod = nl["modules"][top]
    netname = {}
    for n, d in mod["netnames"].items():
        for k, b in enumerate(d["bits"]):
            if isinstance(b, int) and (b not in netname or not n.startswith("$")):
                netname[b] = "%s[%d]" % (n, k) if len(d["bits"]) > 1 else n

    driver = {}                    # bit -> (cell, port)
    sinks = defaultdict(list)      # bit -> [(cell, port)]
    inst = mod["cells"]
    for cn, c in inst.items():
        lc = cells.get(c["type"])
        if lc is None:
            continue
        for port, bits in c["connections"].items():
            for b in bits:
                if not isinstance(b, int):
                    continue
                if port in lc["outs"]:
                    driver[b] = (cn, port)
                elif port in lc["ins"]:
                    sinks[b].append((cn, port))

    def load(b):
        cap = WIRE_CAP_BASE
        for cn, port in sinks.get(b, []):
            cap += cells[inst[cn]["type"]]["caps"].get(port, 0.002) + WIRE_CAP_PER_SINK
        return cap

    # topological order over combinational cells
    arr, slew, back = {}, {}, {}
    seq_cells = [cn for cn, c in inst.items() if c["type"] in cells and cells[c["type"]]["seq"]]

    def out_bits(cn):
        return [(p, b) for p, bits in inst[cn]["connections"].items()
                if p in cells[inst[cn]["type"]]["outs"] for b in bits if isinstance(b, int)]

    for cn in seq_cells:
        lc = cells[inst[cn]["type"]]
        for p, b in out_bits(cn):
            best = None
            for rel, ttype, tabs in lc["arcs"].get(p, []):
                if ttype not in ("rising_edge", "falling_edge"):
                    continue
                cl = load(b)
                d = max(tabs[k].lookup(0.12, cl) for k in ("cell_rise", "cell_fall") if k in tabs)
                s = max(tabs[k].lookup(0.12, cl) for k in ("rise_transition", "fall_transition") if k in tabs)
                if best is None or d > best[0]:
                    best = (d, s)
            if best:
                arr[b], slew[b], back[b] = best[0], best[1], ("Q", cn, None)

    # comb cells in dependency order (Kahn)
    comb = [cn for cn, c in inst.items() if c["type"] in cells and not cells[c["type"]]["seq"]]
    indeg = {}
    fanout_cells = defaultdict(list)
    for cn in comb:
        lc = cells[inst[cn]["type"]]
        deps = set()
        for p in lc["ins"]:
            for b in inst[cn]["connections"].get(p, []):
                if isinstance(b, int) and b in driver and driver[b][0] in inst and \
                        not cells[inst[driver[b][0]]["type"]]["seq"]:
                    deps.add(driver[b][0])
        indeg[cn] = len(deps)
        for d in deps:
            fanout_cells[d].append(cn)
    ready = [cn for cn in comb if indeg[cn] == 0]
    while ready:
        cn = ready.pop()
        lc = cells[inst[cn]["type"]]
        for p, b in out_bits(cn):
            cl = load(b)
            best = None
            for rel, ttype, tabs in lc["arcs"].get(p, []):
                inb = inst[cn]["connections"].get(rel, [])
                if not inb or not isinstance(inb[0], int):
                    continue
                ib = inb[0]
                a0 = arr.get(ib, 0.0)
                s0 = slew.get(ib, 0.12)
                d = max(tabs[k].lookup(s0, cl) for k in ("cell_rise", "cell_fall") if k in tabs)
                s = max(tabs[k].lookup(s0, cl) for k in ("rise_transition", "fall_transition") if k in tabs)
                if best is None or a0 + d > best[0]:
                    best = (a0 + d, s, ib)
            if best:
                arr[b], slew[b], back[b] = best[0], best[1], ("C", cn, best[2])
        for f in fanout_cells[cn]:
            indeg[f] -= 1
            if indeg[f] == 0:
                ready.append(f)

    # endpoints: flop D pins
    ends = []
    for cn in seq_cells:
        lc = cells[inst[cn]["type"]]
        for p, tabs in lc["setup"].items():
            for b in inst[cn]["connections"].get(p, []):
                if isinstance(b, int) and b in arr:
                    su = max(tabs[k].lookup(slew.get(b, 0.12), 0.12) for k in ("rise_constraint", "fall_constraint") if k in tabs)
                    ends.append((arr[b] + su, b, cn, p, su))
    ends.sort(reverse=True)

    def trace(b):
        chain = []
        while b in back:
            kind, cn, prev = back[b]
            chain.append((inst[cn]["type"], cn, arr[b]))
            if kind == "Q":
                break
            b = prev
        return list(reversed(chain))

    res = []
    for t, b, cn, p, su in ends[:report]:
        res.append((t, netname.get(b, str(b)), cn, trace(b), su))
    return res, len(seq_cells), len(comb)


if __name__ == "__main__":
    lib = load_liberty(sys.argv[1])
    cells = build_cells(lib)
    res, nseq, ncomb = analyse(sys.argv[2], cells, sys.argv[3], report=int(sys.argv[4]) if len(sys.argv) > 4 else 6)
    print("flops=%d comb cells=%d" % (nseq, ncomb))
    for t, name, cn, chain, su in res:
        print("\nendpoint %-48s reg->reg delay %.2f ns (setup %.2f)  stages=%d" % (name[:48], t, su, len(chain)))
    t, name, cn, chain, su = res[0]
    print("\nWORST PATH  %.2f ns -> f_max(no margin) = %.1f MHz" % (t, 1000.0 / t))
    prev = 0
    for typ, c, a in chain:
        print("   %-24s %-14s +%.3f  =%.3f" % (typ.replace("sg13g2_", ""), c[:14], a - prev, a))
        prev = a
