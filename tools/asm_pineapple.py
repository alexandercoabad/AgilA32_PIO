#!/usr/bin/env python3
"""RV32I mini-assembler for AgilA32, extracted from
tools/build_boot_rom.py's Asm class (same encoding, same finalize()
fixup logic) so it can be imported by other programs instead of only
existing as inline script code. Adds SRLI, which the boot ROM itself
never needed but this LCD driver does (extracting the MSB of a byte
for bit-banged SPI). JAL here is standard RISC-V semantics -- writes
its return address ONLY to the encoded rd (unlike AgilA8's core,
which hardwires JAL's link register to r7 regardless of rd -- no such
quirk here, JAL(0, label) is a genuinely side-effect-free plain jump).

Wraps every OP_IMM/OP_REG/branch/load/store/U-type instruction
rv32i_core.v's ALU and decode actually implement -- confirmed against
that file directly (its `alu_op` case statement and `branch_cond` case
statement), not assumed. Earlier versions of this assembler only
wrapped whichever handful of opcodes a given script happened to need
at the time (ADD/OR/AND/XOR, ADDI/ANDI/ORI/SLLI/SRLI, BEQ/BNE/BLT,
LW/LBU, SB/SW) -- SUB, the other register-register shifts (SLL/SRL/
SRA), both SLT variants (register and immediate), XORI, SRAI, the
other three branches (BGE/BLTU/BGEU), the other loads (LB/LH/LHU), SH,
and LUI/AUIPC were all sitting unused in the core the entire time. No
RTL work needed to add any of this -- the gap was purely that nobody
had written the Python encoder for opcodes nothing had needed yet.
"""

OPC = dict(
    OP_IMM=0b0010011, OP_REG=0b0110011, OP_LOAD=0b0000011,
    OP_STORE=0b0100011, OP_BRANCH=0b1100011, OP_JAL=0b1101111,
    OP_JALR=0b1100111, OP_LUI=0b0110111, OP_AUIPC=0b0010111,
    OP_SYSTEM=0b1110011,
)


def _s12(v):
    return v & 0xFFF


class Asm:
    def __init__(self, base=0):
        self.instrs = []
        self.labels = {}
        self.fixups = []
        # Address encoded for the FIRST instruction -- 0 for a normal
        # standalone program (unchanged default, everything upstream of
        # PagedAsm keeps working exactly as before), or LOAD_BASE for a
        # page built by PagedAsm, since every page's code physically
        # executes starting at that fixed chip address regardless of
        # where the page's bytes sit in the flash image. Only affects
        # PC-relative JAL/branch encoding -- doesn't change instruction
        # order or count.
        self.base = base

    def addr(self):
        return self.base + len(self.instrs) * 4

    def label(self, name):
        assert name not in self.labels, name
        self.labels[name] = self.addr()

    def emit(self, word_or_none, fixup=None):
        idx = len(self.instrs)
        self.instrs.append(word_or_none)
        if fixup:
            self.fixups.append((idx,) + fixup)
        return idx

    def _r(self, funct7, rs2, rs1, funct3, rd, opcode):
        return (funct7 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

    def ADD(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b000, rd, OPC['OP_REG']))
    def SUB(self, rd, rs1, rs2): self.emit(self._r(0x20, rs2, rs1, 0b000, rd, OPC['OP_REG']))
    def SLL(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b001, rd, OPC['OP_REG']))
    def SLT(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b010, rd, OPC['OP_REG']))
    def SLTU(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b011, rd, OPC['OP_REG']))
    def XOR(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b100, rd, OPC['OP_REG']))
    def SRL(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b101, rd, OPC['OP_REG']))
    def SRA(self, rd, rs1, rs2): self.emit(self._r(0x20, rs2, rs1, 0b101, rd, OPC['OP_REG']))
    def OR(self, rd, rs1, rs2):  self.emit(self._r(0, rs2, rs1, 0b110, rd, OPC['OP_REG']))
    def AND(self, rd, rs1, rs2): self.emit(self._r(0, rs2, rs1, 0b111, rd, OPC['OP_REG']))

    def _i(self, imm, rs1, funct3, rd, opcode):
        return (_s12(imm) << 20) | (rs1 << 15) | (funct3 << 12) | (rd << 7) | opcode

    def ADDI(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b000, rd, OPC['OP_IMM']))
    def SLTI(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b010, rd, OPC['OP_IMM']))
    def SLTIU(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b011, rd, OPC['OP_IMM']))
    def XORI(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b100, rd, OPC['OP_IMM']))
    def ANDI(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b111, rd, OPC['OP_IMM']))
    def ORI(self, rd, rs1, imm):  self.emit(self._i(imm, rs1, 0b110, rd, OPC['OP_IMM']))
    def SLLI(self, rd, rs1, sh):  self.emit(self._i(sh & 0x1F, rs1, 0b001, rd, OPC['OP_IMM']))
    def SRLI(self, rd, rs1, sh):  self.emit(self._i(sh & 0x1F, rs1, 0b101, rd, OPC['OP_IMM']))
    # SRAI reuses SRLI's funct3 (101); the ALU picks arithmetic vs
    # logical purely off imm[10] (funct7's bit 5) -- see rv32i_core.v's
    # `funct7_b5` -- so the encoding here is identical to SRLI's except
    # for that one extra bit (0x400) folded into the immediate field.
    def SRAI(self, rd, rs1, sh):  self.emit(self._i(0x400 | (sh & 0x1F), rs1, 0b101, rd, OPC['OP_IMM']))
    def LB(self, rd, rs1, imm):   self.emit(self._i(imm, rs1, 0b000, rd, OPC['OP_LOAD']))
    def LH(self, rd, rs1, imm):   self.emit(self._i(imm, rs1, 0b001, rd, OPC['OP_LOAD']))
    def LW(self, rd, rs1, imm):   self.emit(self._i(imm, rs1, 0b010, rd, OPC['OP_LOAD']))
    def LBU(self, rd, rs1, imm):  self.emit(self._i(imm, rs1, 0b100, rd, OPC['OP_LOAD']))
    def LHU(self, rd, rs1, imm):  self.emit(self._i(imm, rs1, 0b101, rd, OPC['OP_LOAD']))
    def JALR(self, rd, rs1, imm): self.emit(self._i(imm, rs1, 0b000, rd, OPC['OP_JALR']))

    # ---- U-type (LUI/AUIPC) ----
    def _u(self, imm20, rd, opcode):
        return ((imm20 & 0xFFFFF) << 12) | (rd << 7) | opcode

    def LUI(self, rd, imm20):   self.emit(self._u(imm20, rd, OPC['OP_LUI']))
    def AUIPC(self, rd, imm20): self.emit(self._u(imm20, rd, OPC['OP_AUIPC']))

    def SB(self, rs2, rs1, imm):
        imm = _s12(imm)
        w = ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b000 << 12) | ((imm & 0x1F) << 7) | OPC['OP_STORE']
        self.emit(w)

    def SW(self, rs2, rs1, imm):
        imm = _s12(imm)
        w = ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b010 << 12) | ((imm & 0x1F) << 7) | OPC['OP_STORE']
        self.emit(w)

    def SH(self, rs2, rs1, imm):
        imm = _s12(imm)
        w = ((imm >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (0b001 << 12) | ((imm & 0x1F) << 7) | OPC['OP_STORE']
        self.emit(w)

    def _branch(self, funct3, rs1, rs2, target):
        self.emit(None, fixup=('branch', funct3, rs1, rs2, target))

    def BEQ(self, rs1, rs2, target):  self._branch(0b000, rs1, rs2, target)
    def BNE(self, rs1, rs2, target):  self._branch(0b001, rs1, rs2, target)
    def BLT(self, rs1, rs2, target):  self._branch(0b100, rs1, rs2, target)
    def BGE(self, rs1, rs2, target):  self._branch(0b101, rs1, rs2, target)
    def BLTU(self, rs1, rs2, target): self._branch(0b110, rs1, rs2, target)
    def BGEU(self, rs1, rs2, target): self._branch(0b111, rs1, rs2, target)

    def EBREAK(self):
        # imm[11:0]=0x001, rs1=0, funct3=0, rd=0, opcode=OP_SYSTEM --
        # halts the core (rv32i_core.v parks in ST_HALTED until reset).
        # ECALL (imm=0x000) is deliberately not wrapped here since the
        # core still treats it as a NOP, not a halt.
        self.emit(self._i(0x001, 0, 0b000, 0, OPC['OP_SYSTEM']))

    def JAL(self, rd, target):
        self.emit(None, fixup=('jal', rd, target))

    def finalize(self):
        words = list(self.instrs)
        for entry in self.fixups:
            idx = entry[0]
            kind = entry[1]
            pc = self.base + idx * 4
            if kind == 'branch':
                _, _, funct3, rs1, rs2, target = entry
                off = self.labels[target] - pc
                assert -4096 <= off < 4096 and off % 2 == 0, f"branch offset {off} out of range"
                imm = off & 0x1FFF
                b12 = (imm >> 12) & 1
                b10_5 = (imm >> 5) & 0x3F
                b4_1 = (imm >> 1) & 0xF
                b11 = (imm >> 11) & 1
                words[idx] = ((b12 << 31) | (b10_5 << 25) | (rs2 << 20) | (rs1 << 15) | (funct3 << 12) |
                              (b4_1 << 8) | (b11 << 7) | OPC['OP_BRANCH'])
            elif kind == 'jal':
                _, _, rd, target = entry
                off = self.labels[target] - pc
                assert -1048576 <= off < 1048576 and off % 2 == 0, f"jal offset {off} out of range"
                imm = off & 0x1FFFFF
                b20 = (imm >> 20) & 1
                b10_1 = (imm >> 1) & 0x3FF
                b11 = (imm >> 11) & 1
                b19_12 = (imm >> 12) & 0xFF
                words[idx] = ((b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | (rd << 7) | OPC['OP_JAL'])
            else:
                raise ValueError(kind)
        assert all(w is not None for w in words)
        return words


def words_to_bytes(words):
    out = bytearray()
    for w in words:
        out += w.to_bytes(4, 'little')
    return bytes(out)


class PagedAsm:
    """Builds a bank-switched flash image for mem.v's FLASH_PAGE
    mechanism: a program larger than one WINDOW_BYTES-sized chip-address
    window (44 bytes / 11 instructions by default), broken into pages,
    each occupying its own WINDOW_BYTES-sized slice of the flash image.
    Page N's bytes live at flash offset N*WINDOW_BYTES -- matching
    mem.v's `flash_page_base = flash_page * WINDOW_BYTES` exactly, so
    writing page number N into FLASH_PAGE (0xFC) makes the chip-address
    window (LOAD_BASE..LOAD_BASE+WINDOW_BYTES-1) show page N's bytes.

    Why a page can't just "fall off the end" into the next one: PC
    keeps incrementing through the FIXED chip-address window regardless
    of FLASH_PAGE -- switching pages only changes what's fetched from
    those SAME addresses going forward, it doesn't extend the window.
    So every page reserves its OWN last three words as:
      switch_offset-4: ADDI scratch,x0,next_page  (from the caller-
                        usable budget, not the reservation itself)
      switch_offset (WINDOW_BYTES-8): SW scratch,0(FLASH_PAGE)
      trampoline_offset (WINDOW_BYTES-4): JAL x0, LOAD_BASE
    which is why each page only has WINDOW_BYTES-12 bytes (8
    instructions at the default 44-byte window) of caller-usable code
    -- the ADDI that loads the page number eats into that budget too,
    it's not free alongside the SW+trampoline. Confirmed in practice
    that even 8 is tight: a fairly minimal 8-bit bit-bang SPI send loop
    needs ~10 instructions as a single routine, which does NOT fit in
    one page's budget. Splitting such a routine across pages isn't
    something this class does for you -- see this project's LCD driver
    notes for what that actually requires (a hand-written state machine
    passing progress in registers across an explicit switch, not a
    plain subroutine call, since a page switch can only ever land at
    LOAD_BASE of the target page, never at an arbitrary offset within
    it).

    The switch write happens EXACTLY WINDOW_BYTES-8 bytes into the
    page (not "wherever the switch call happens to be") because of the
    same same-cycle-redirect hazard documented in
    build_flash_handoff_stub.py: FLASH_PAGE takes effect for the very
    next fetch, which is always PC+4 relative to the SW that set it --
    so that next fetch must land on a word that's valid REGARDLESS of
    which page just became active, which only works if that word (the
    trampoline) is byte-identical in every single page. Fixing the
    switch's own position at WINDOW_BYTES-8 is what guarantees PC+4
    always equals WINDOW_BYTES-4 (the trampoline slot), every time.

    Page 0 is special: it's reached via build_flash_handoff_stub.py's
    RAM-resident handoff, not via another page's switch_to(), and that
    stub's own PC+4 lands on LOAD_BASE+4 -- flash byte 0 is never
    fetched by anything (see that file's "Why flash byte 0 is dead").
    This class inserts that dead NOP automatically for page 0 so the
    caller doesn't have to remember to.
    """

    def __init__(self, load_base=0xB4, window_bytes=44, flash_page_addr=0xFC,
                 switch_scratch_reg=30):
        self.load_base = load_base
        self.window_bytes = window_bytes
        self.switch_offset = window_bytes - 8
        self.trampoline_offset = window_bytes - 4
        self.flash_page_addr = flash_page_addr
        self.switch_scratch_reg = switch_scratch_reg
        self.pages = []          # finished pages, each `window_bytes` long
        self._cur_page_num = 0
        self._cur = Asm(base=load_base)
        if load_base != 0:
            # page 0's own byte 0 is dead (see class docstring) -- only
            # page 0, since every OTHER page is entered via a switch_to()
            # trampoline that jumps straight to LOAD_BASE (offset 0 of
            # that page), with no equivalent "PC+4 already past it" loss.
            self._cur.ADDI(0, 0, 0)  # dead NOP, never fetched

    @property
    def page(self):
        """The Asm instance for the page currently being built -- use
        its normal ADDI/SW/JAL/etc methods. Labels/branches are only
        valid WITHIN the current page; nothing carries across a
        switch_to() boundary except register contents."""
        return self._cur

    def bytes_used(self):
        """How many of this page's usable (pre-reservation) bytes are
        already spoken for -- for checking there's room before adding
        more instructions."""
        return self._cur.addr() - self._cur.base

    def bytes_free(self):
        return self.switch_offset - self.bytes_used()

    def switch_to(self, next_page_num, scratch_reg=None):
        """Ends the current page: pads it out to the fixed switch
        offset with NOPs, emits the page-switch write (targeting
        `next_page_num`) and the mandatory trampoline, finalizes this
        page's bytes, and starts a fresh page (this object's next
        sequential page index) for subsequent .page calls to build
        into. `next_page_num` is a runtime VALUE written into
        FLASH_PAGE -- for a simple linear chain you'll want it to equal
        the new page's own index in this object's sequence, but nothing
        enforces that (a page can jump to any page number, including
        backward, e.g. to loop a sequence of pages)."""
        reg = self.switch_scratch_reg if scratch_reg is None else scratch_reg
        # The ADDI (loading the page number) comes from the page's own
        # USABLE budget -- only the SW itself (the actual triggering
        # write) needs to sit at the fixed switch_offset, so pad to
        # 4 bytes before it, not to switch_offset itself.
        self._pad_to(self.switch_offset - 4)
        self._cur.ADDI(reg, 0, next_page_num)
        assert self.bytes_used() == self.switch_offset, (
            f"internal error: SW should land at offset {self.switch_offset}, "
            f"about to emit it at {self.bytes_used()}"
        )
        self._cur.SW(reg, 0, self.flash_page_addr)
        self._emit_trampoline()
        self._finish_current_page()

    def switch_to_computed(self, scratch_reg=None):
        """Like switch_to(), but for the case a REAL multi-page program
        actually needs and switch_to() can't express: the next page
        number isn't known until runtime (e.g. a bulk-transfer loop
        deciding "one more iteration" vs "done" from a pixel counter
        in a register). switch_to() always emits its OWN ADDI to load
        a compile-time constant -- there's no way to hand it a value
        you already computed. This is that other half: it does NOT
        emit an ADDI. The caller must have already loaded the target
        page number into `scratch_reg` (default: switch_scratch_reg,
        same register switch_to() itself uses -- matters if a
        convergent branch is what computed it, see below) by whatever
        runtime logic they need, landing at or before switch_offset,
        BEFORE calling this. This only pads the remaining gap (if any)
        up to switch_offset and emits the SW + trampoline -- the exact
        same fixed-offset reservation switch_to() uses, so the same
        same-cycle-redirect safety (PC+4 always lands on the
        page-identical trampoline) holds here too.

        Typical shape for the caller's own runtime-conditional code
        (a two-armed branch converging before this call, both arms
        loading switch_scratch_reg with a DIFFERENT page number):
            p.page.ADDI(5, 5, -1)              # decrement counter
            p.page.BNE(5, 0, "NOT_DONE")
            p.page.ADDI(30, 0, DONE_PAGE)      # counter hit 0
            p.page.JAL(0, "CONVERGE")
            p.page.label("NOT_DONE")
            p.page.ADDI(30, 0, LOOP_BACK_PAGE) # counter still going
            p.page.label("CONVERGE")
            p.switch_to_computed()             # both arms already set x30
        Both arms MUST end up at the exact same byte offset (the
        `CONVERGE` label) for this to be well-formed -- pad the
        shorter arm with NOPs if their instruction counts differ, or
        this lands the SW at a different offset depending on which
        branch was taken, breaking the one guarantee this whole
        mechanism depends on. Not checked automatically (this class
        has no way to know both arms are "supposed" to converge);
        get it wrong and the failure mode is silent -- a page-
        dependent trampoline offset, which the fixed-position
        assumption everywhere else in this class does NOT defend
        against outside of switch_to()'s own single straight-line
        path.
        """
        reg = self.switch_scratch_reg if scratch_reg is None else scratch_reg
        self._pad_to(self.switch_offset)
        assert self.bytes_used() == self.switch_offset, (
            f"internal error: SW should land at offset {self.switch_offset}, "
            f"about to emit it at {self.bytes_used()}"
        )
        self._cur.SW(reg, 0, self.flash_page_addr)
        self._emit_trampoline()
        self._finish_current_page()

    def finalize_last_page(self, scratch_reg=None):
        """Ends the final page of the program. Still reserves the same
        fixed switch+trampoline slot as switch_to() -- for uniformity,
        and as a defensive fallback (it targets this page's OWN
        number, so falling through restarts this same page rather than
        executing whatever garbage would otherwise sit there) -- but
        this page's own code must not naturally fall through into that
        reserved area if restarting isn't the intended behavior: end it
        with its own internal infinite loop (e.g. `p.JAL(0, "SPIN")`
        with a "SPIN" label right before it) well within the usable
        budget. Call `finalize()` right after this; don't use `.page`
        again afterward."""
        self.switch_to(self._cur_page_num, scratch_reg=scratch_reg)

    def _pad_to(self, target_offset):
        used = self.bytes_used()
        assert used <= target_offset, (
            f"page {self._cur_page_num}'s code doesn't fit: already at "
            f"offset {used} bytes, but the page-switch reservation starts "
            f"at offset {target_offset} -- this page has only "
            f"{self.switch_offset} bytes ({self.switch_offset // 4} "
            f"instructions) of usable code, {used} of which are used"
        )
        while self.bytes_used() < target_offset:
            self._cur.ADDI(0, 0, 0)  # NOP

    def _emit_trampoline(self):
        # JAL x0, LOAD_BASE -- a fixed, page-independent offset (always
        # -trampoline_offset from this exact point), so this encodes
        # identically in every page. Encoded directly rather than via
        # Asm's label/fixup machinery since it's a compile-time
        # constant, not something that needs resolving against a label.
        pc = self._cur.addr()
        off = self.load_base - pc
        assert off == -self.trampoline_offset, (
            f"internal error: trampoline offset should be exactly "
            f"-{self.trampoline_offset}, computed {off} -- page-switch "
            f"reservation math is inconsistent with actual page position"
        )
        assert -1048576 <= off < 1048576 and off % 2 == 0
        imm = off & 0x1FFFFF
        b20 = (imm >> 20) & 1
        b10_1 = (imm >> 1) & 0x3FF
        b11 = (imm >> 11) & 1
        b19_12 = (imm >> 12) & 0xFF
        word = (b20 << 31) | (b10_1 << 21) | (b11 << 20) | (b19_12 << 12) | (0 << 7) | OPC['OP_JAL']
        self._cur.emit(word)

    def _finish_current_page(self):
        words = self._cur.finalize()
        page_bytes = words_to_bytes(words)
        assert len(page_bytes) == self.window_bytes, (
            f"internal error: page {self._cur_page_num} assembled to "
            f"{len(page_bytes)} bytes, expected exactly {self.window_bytes}"
        )
        self.pages.append(page_bytes)
        self._cur_page_num += 1
        self._cur = Asm(base=self.load_base)

    def finalize(self):
        """Returns the full flat flash image: every finished page
        concatenated in order, page N at byte offset N*window_bytes --
        matching mem.v's flash_page_base computation, so page number
        equals position in this image, not an independent numbering.
        Call finalize_last_page() (not switch_to()) for the final page
        before calling this, or its instructions are silently lost."""
        return b''.join(self.pages)
