#!/usr/bin/env python3
"""pioasm.py -- assembler + disassembler for the AgilA32 PIO block.

Accepts the same source syntax as the Raspberry Pi `pioasm` tool (the ISA is
RP2040-PIO-compatible), so programs from the Pico SDK / datasheet can be
pasted in unchanged:

    .program uart_tx
    .side_set 1 opt
        pull       side 1 [7]
        set x, 7   side 0 [7]
    bitloop:
        out pins, 1
        jmp x-- bitloop   [6]

Supported: .program  .side_set N [opt] [pindirs]  .define [public] NAME expr
.wrap_target  .wrap  .origin (ignored)  .lang_opt (ignored)  .word  labels,
`side <v>` and `[delay]` in either order, and integer expressions
(+ - * / ( ) with decimal/hex/binary literals and .define'd names).

    from pioasm import assemble
    prog = assemble(open("uart_tx.pio").read())
    prog.instrs          # list of 16-bit words
    prog.wrap_target, prog.wrap, prog.side_bits, prog.side_opt, ...

CLI:  python3 pioasm.py file.pio [--format list|hex|c|py]
"""
import re
import sys

# ---------------------------------------------------------------- encodings
JMP_COND = {"": 0, "!x": 1, "x--": 2, "!y": 3, "y--": 4, "x!=y": 5,
            "pin": 6, "!osre": 7}
IN_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "isr": 6, "osr": 7}
OUT_DST = {"pins": 0, "x": 1, "y": 2, "null": 3, "pindirs": 4, "pc": 5,
           "isr": 6, "exec": 7}
MOV_DST = {"pins": 0, "x": 1, "y": 2, "exec": 4, "pc": 5, "isr": 6, "osr": 7}
MOV_SRC = {"pins": 0, "x": 1, "y": 2, "null": 3, "status": 5, "isr": 6,
           "osr": 7}
MOV_OP = {"": 0, "!": 1, "~": 1, "::": 2}
SET_DST = {"pins": 0, "x": 1, "y": 2, "pindirs": 4}
WAIT_SRC = {"gpio": 0, "pin": 1, "irq": 2}


class AsmError(Exception):
    pass


class Program:
    def __init__(self, name):
        self.name = name
        self.origin = 0            # instruction-memory load address (jmp targets are relocated)
        self.instrs = []
        self.labels = {}
        self.defines = {}
        self.wrap_target = None
        self.wrap = None
        self.side_n = 0            # number of side-set data bits
        self.side_opt = False
        self.side_pindirs = False

    @property
    def side_bits(self):
        """Value for PINCTRL.SIDESET_COUNT (includes the opt enable bit)."""
        return self.side_n + (1 if self.side_opt else 0)

    @property
    def wrap_bottom(self):
        return self.origin if self.wrap_target is None else self.wrap_target + self.origin

    @property
    def wrap_top(self):
        return (len(self.instrs) - 1 if self.wrap is None else self.wrap) + self.origin

    # RP2040-layout register images for the state machine config -----------
    def execctrl(self, jmp_pin=0, status_sel=0, status_n=0):
        return (status_n & 0xF) | (status_sel << 4) | (self.wrap_bottom << 7) \
            | (self.wrap_top << 12) | ((jmp_pin & 0xF) << 24) \
            | (int(self.side_pindirs) << 29) | (int(self.side_opt) << 30)

    def __repr__(self):
        return "Program(%s, %d instrs)" % (self.name, len(self.instrs))


# ------------------------------------------------------------------ parsing
def _eval(expr, symbols, what="expression"):
    expr = expr.strip()
    if not expr:
        raise AsmError("empty " + what)

    def sub(m):
        tok = m.group(0)
        if tok in symbols:
            return "(%d)" % symbols[tok]
        raise AsmError("unknown symbol '%s'" % tok)
    # replace identifiers (not part of a 0x/0b literal) with their value
    e = re.sub(r"(?<![0-9A-Za-z_])[A-Za-z_][A-Za-z_0-9]*", sub, expr)
    if not re.fullmatch(r"[0-9a-fA-FxXbB_+\-*/() ]+", e):
        raise AsmError("bad %s '%s'" % (what, expr))
    e = e.replace("/", "//")
    try:
        return int(eval(e, {"__builtins__": {}}, {}))
    except Exception:
        raise AsmError("cannot evaluate %s '%s'" % (what, expr))


def _strip(line):
    for c in (";", "//"):
        if c in line:
            line = line.split(c, 1)[0]
    return line.strip()


def assemble(text, name=None, origin=0):
    prog = Program(name or "program")
    prog.origin = origin
    lines = []           # (lineno, kind, payload)
    pending_labels = []

    # ---- pass 1: directives, labels, collect instruction lines ----
    for no, raw in enumerate(text.splitlines(), 1):
        line = _strip(raw)
        if not line:
            continue
        # labels (possibly followed by an instruction on the same line)
        while True:
            m = re.match(r"^(public\s+)?([A-Za-z_][A-Za-z_0-9]*)\s*:\s*(.*)$", line)
            if not m:
                break
            prog.labels[m.group(2)] = len(lines)
            line = m.group(3).strip()
        if not line:
            continue
        if line.startswith("."):
            parts = line.split(None, 1)
            d = parts[0].lower()
            rest = parts[1] if len(parts) > 1 else ""
            if d == ".program":
                prog.name = rest.strip() or prog.name
            elif d == ".define":
                toks = rest.split(None, 1)
                if toks and toks[0] == "public":
                    toks = toks[1].split(None, 1)
                if len(toks) != 2:
                    raise AsmError("line %d: bad .define" % no)
                prog.defines[toks[0]] = _eval(toks[1], prog.defines, ".define")
            elif d == ".side_set":
                toks = rest.split()
                if not toks:
                    raise AsmError("line %d: .side_set needs a bit count" % no)
                prog.side_n = _eval(toks[0], prog.defines, ".side_set")
                prog.side_opt = "opt" in toks[1:]
                prog.side_pindirs = "pindirs" in toks[1:]
                if prog.side_n + int(prog.side_opt) > 5:
                    raise AsmError("line %d: side_set + opt exceeds 5 bits" % no)
            elif d == ".wrap_target":
                prog.wrap_target = len(lines)
            elif d == ".wrap":
                prog.wrap = len(lines) - 1
            elif d in (".origin", ".lang_opt", ".clock_div", ".fifo"):
                pass
            elif d == ".word":
                lines.append((no, "word", rest))
            else:
                raise AsmError("line %d: unknown directive %s" % (no, d))
            continue
        lines.append((no, "instr", line))

    symbols = dict(prog.defines)
    symbols.update({k: v + origin for k, v in prog.labels.items()})

    # ---- pass 2: encode ----
    for idx, (no, kind, line) in enumerate(lines):
        try:
            if kind == "word":
                prog.instrs.append(_eval(line, symbols) & 0xFFFF)
            else:
                prog.instrs.append(_encode(line, prog, symbols))
        except AsmError as e:
            raise AsmError("line %d: %s   [%s]" % (no, e, line))
    if origin + len(prog.instrs) > 32:
        raise AsmError("program (%d instructions at origin %d) overflows the 32-word instruction memory"
                       % (len(prog.instrs), origin))
    return prog


def _encode(line, prog, sym):
    line = line.strip()
    # -- pull off `side <expr>` and `[<expr>]` (either order) --
    side = None
    delay = 0
    m = re.search(r"\[([^\]]*)\]", line)
    if m:
        delay = _eval(m.group(1), sym, "delay")
        line = (line[:m.start()] + " " + line[m.end():]).strip()
    m = re.search(r"\bside\s+([^\s\[]+)", line)
    if m:
        side = _eval(m.group(1), sym, "side-set value")
        line = (line[:m.start()] + " " + line[m.end():]).strip()

    side_bits = prog.side_bits
    dly_bits = 5 - side_bits
    if delay < 0 or delay >= (1 << dly_bits):
        raise AsmError("delay %d out of range (max %d with %d side-set bit(s))"
                       % (delay, (1 << dly_bits) - 1, side_bits))
    if side is None:
        sfld = 0
    else:
        if side_bits == 0:
            raise AsmError("`side` used but no .side_set declared")
        if side < 0 or side >= (1 << prog.side_n):
            raise AsmError("side-set value %d out of range" % side)
        sfld = side | ((1 << prog.side_n) if prog.side_opt else 0)
    field = (sfld << dly_bits) | delay

    toks = re.sub(r",", " , ", line).split()
    if not toks:
        raise AsmError("empty instruction")
    mn = toks[0].lower()
    args = toks[1:]
    low = [t.lower() for t in args]

    def word(op, operand):
        return (op << 13) | (field << 8) | (operand & 0xFF)

    def need(n):
        if len(args) < n:
            raise AsmError("%s: missing operand" % mn)

    if mn == "nop":
        return word(5, (2 << 5) | (0 << 3) | 2)          # mov y, y

    if mn == "jmp":
        cond = ""
        rest = args
        if args and args[0].lower() in JMP_COND and args[0].lower() != "":
            cond = args[0].lower()
            rest = args[1:]
        elif len(args) >= 2 and args[0].lower() == "x" and args[1] == "--":
            cond, rest = "x--", args[2:]
        elif len(args) >= 2 and args[0].lower() == "y" and args[1] == "--":
            cond, rest = "y--", args[2:]
        elif len(args) >= 2 and args[0] == "!" and args[1].lower() in ("x", "y", "osre"):
            cond, rest = "!" + args[1].lower(), args[2:]
        if not rest:
            raise AsmError("jmp: missing target")
        tgt = _eval(" ".join(rest), sym, "jump target")
        if not 0 <= tgt < 32:
            raise AsmError("jump target %d out of range" % tgt)
        return word(0, (JMP_COND[cond] << 5) | tgt)

    if mn == "wait":
        # the Pico SDK writes `wait 1 pin, 1` (comma after the source); accept both spellings
        args = [t for t in args if t != ","]
        low = [t.lower() for t in args]
        need(2)
        pol = _eval(args[0], sym, "polarity")
        if pol not in (0, 1):
            raise AsmError("wait polarity must be 0 or 1")
        src = low[1]
        if src not in WAIT_SRC:
            raise AsmError("wait source must be gpio, pin or irq")
        need(3)
        idx = _eval(args[2], sym, "index")
        if src == "irq":
            rel = len(low) > 3 and low[3] == "rel"
            if not 0 <= idx < 8:
                raise AsmError("irq index out of range")
            idx = idx | (0x10 if rel else 0)
        elif not 0 <= idx < 32:
            raise AsmError("pin index out of range")
        return word(1, (pol << 7) | (WAIT_SRC[src] << 5) | idx)

    if mn in ("in", "out"):
        need(3)
        table = IN_SRC if mn == "in" else OUT_DST
        if low[0] not in table:
            raise AsmError("%s: bad %s '%s'" % (mn, "source" if mn == "in" else "destination", args[0]))
        cnt = _eval(args[2], sym, "bit count")
        if not 1 <= cnt <= 32:
            raise AsmError("bit count must be 1..32")
        return word(2 if mn == "in" else 3, (table[low[0]] << 5) | (cnt & 31))

    if mn in ("push", "pull"):
        iff = "iffull" in low or "ifempty" in low
        blk = 0 if "noblock" in low else 1
        return word(4, ((1 if mn == "pull" else 0) << 7) | (int(iff) << 6) | (blk << 5))

    if mn == "mov":
        need(3)
        if low[0] not in MOV_DST:
            raise AsmError("mov: bad destination '%s'" % args[0])
        rest = "".join(args[2:]).lower()
        op = ""
        for pfx in ("::", "!", "~"):
            if rest.startswith(pfx):
                op, rest = pfx, rest[len(pfx):]
                break
        if rest not in MOV_SRC:
            raise AsmError("mov: bad source '%s'" % rest)
        return word(5, (MOV_DST[low[0]] << 5) | (MOV_OP[op] << 3) | MOV_SRC[rest])

    if mn == "irq":
        mode = "set"
        rest = list(args)
        if rest and rest[0].lower() in ("set", "nowait", "wait", "clear"):
            mode = rest.pop(0).lower()
        if not rest:
            raise AsmError("irq: missing index")
        idx = _eval(rest[0], sym, "irq index")
        if not 0 <= idx < 8:
            raise AsmError("irq index out of range")
        if len(rest) > 1 and rest[1].lower() == "rel":
            idx |= 0x10
        bits = {"set": 0, "nowait": 0, "wait": 1 << 5, "clear": 1 << 6}[mode]
        return word(6, bits | idx)

    if mn == "set":
        need(3)
        if low[0] not in SET_DST:
            raise AsmError("set: bad destination '%s'" % args[0])
        val = _eval(args[2], sym, "set value")
        if not 0 <= val < 32:
            raise AsmError("set value must be 0..31")
        return word(7, (SET_DST[low[0]] << 5) | val)

    raise AsmError("unknown instruction '%s'" % mn)


# ------------------------------------------------------------- disassembler
def disasm(w, side_bits=0, side_opt=False):
    op = (w >> 13) & 7
    fld = (w >> 8) & 31
    arg = w & 0xFF
    dbits = 5 - side_bits
    dly = fld & ((1 << dbits) - 1)
    sfld = fld >> dbits
    extra = ""
    if side_bits:
        if side_opt:
            if sfld >> (side_bits - 1) & 1:
                extra += " side %d" % (sfld & ((1 << (side_bits - 1)) - 1))
        else:
            extra += " side %d" % sfld
    if dly:
        extra += " [%d]" % dly
    inv = lambda t, v: {v2: k for k, v2 in t.items()}.get(v, "?%d" % v)
    if op == 0:
        c = inv(JMP_COND, arg >> 5)
        s = "jmp %s%d" % ((c + ", ") if c else "", arg & 31)
    elif op == 1:
        src = inv(WAIT_SRC, (arg >> 5) & 3)
        i = arg & 31
        s = "wait %d %s %d%s" % (arg >> 7, src, i & (7 if src == "irq" else 31),
                                 " rel" if src == "irq" and i & 0x10 else "")
    elif op == 2:
        s = "in %s, %d" % (inv(IN_SRC, arg >> 5), (arg & 31) or 32)
    elif op == 3:
        s = "out %s, %d" % (inv(OUT_DST, arg >> 5), (arg & 31) or 32)
    elif op == 4:
        kind = "pull" if arg & 0x80 else "push"
        cond = (" ifempty" if kind == "pull" else " iffull") if arg & 0x40 else ""
        s = "%s%s %s" % (kind, cond, "block" if arg & 0x20 else "noblock")
    elif op == 5:
        o = {0: "", 1: "!", 2: "::", 3: "?"}[(arg >> 3) & 3]
        if w & 0xE0FF == 0xA042:
            s = "nop"
        else:
            s = "mov %s, %s%s" % (inv(MOV_DST, arg >> 5), o, inv(MOV_SRC, arg & 7))
    elif op == 6:
        m = "clear" if arg & 0x40 else ("wait" if arg & 0x20 else "set")
        s = "irq %s %d%s" % (m, arg & 7, " rel" if arg & 0x10 else "")
    else:
        s = "set %s, %d" % (inv(SET_DST, arg >> 5), arg & 31)
    return s + extra


# --------------------------------------------------------------------- CLI
def main(argv):
    import argparse
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("file")
    ap.add_argument("--origin", type=int, default=0,
                    help="instruction-memory address the program will be loaded at")
    ap.add_argument("--format", default="list",
                    choices=["list", "hex", "c", "py"])
    a = ap.parse_args(argv)
    p = assemble(open(a.file).read(), origin=a.origin)
    if a.format == "hex":
        print("\n".join("%04x" % w for w in p.instrs))
    elif a.format == "c":
        print("static const uint16_t %s_program[] = {" % p.name)
        for i, w in enumerate(p.instrs):
            print("    0x%04x, // %2d: %s" % (w, i, disasm(w, p.side_bits, p.side_opt)))
        print("};\n// wrap_target=%d wrap=%d side_bits=%d opt=%d pindirs=%d"
              % (p.wrap_bottom, p.wrap_top, p.side_bits, p.side_opt, p.side_pindirs))
    elif a.format == "py":
        print("%s = dict(instrs=%r, wrap_bottom=%d, wrap_top=%d, side_bits=%d, side_opt=%r, side_pindirs=%r)"
              % (p.name, p.instrs, p.wrap_bottom, p.wrap_top, p.side_bits, p.side_opt, p.side_pindirs))
    else:
        print("; %s   wrap %d..%d  side-set %d bit(s)%s%s" % (
            p.name, p.wrap_bottom, p.wrap_top, p.side_bits,
            " opt" if p.side_opt else "", " pindirs" if p.side_pindirs else ""))
        for i, w in enumerate(p.instrs):
            print("%2d: %04x  %s" % (i, w, disasm(w, p.side_bits, p.side_opt)))


if __name__ == "__main__":
    main(sys.argv[1:])
