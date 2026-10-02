#!/usr/bin/env python3
"""pio_host.py -- build AgilA32 flash images that program the PIO block.

The RV32I core has only an 11-instruction on-chip execute window, and every
PIO register write costs several instructions (index write + data write, and
32-bit constants need LUI+ADDI), so a realistic "load this PIO program,
configure the state machine, start it" sequence has to run from paged flash,
exactly like tools/build_st7789_flash_image.py. This module does the tedious
part: it packs PIO register operations into PagedAsm pages (never splitting an
operation across a page switch) and emits the flash image.

    from pioasm  import assemble
    from pio_host import PioHost, pinctrl, execctrl, R_CTRL, sm_reg, SM_PINCTRL

    h = PioHost()
    h.load_program(assemble(open("pio/uart_tx.pio").read()))
    h.write_reg(sm_reg(0, SM_PINCTRL), pinctrl(out_base=0, out_count=1, ...))
    h.write_reg(R_CTRL, 0x1)                    # enable SM0
    h.halt()                                    # EBREAK -- PIO keeps running
    image = h.build()                           # bytes, page N at N*44

The bus (see rtl pio.v): two memory-mapped bytes, PIO_DATA = 0xFE and
PIO_IDX = 0xFF. Stores to 0xFE take a full 32-bit word; the index register
selects which PIO register that word reaches.

Ordering rule for forced instructions (SMx_INSTR): a forced instruction runs at
the SM's next clock tick. On a *disabled* SM that is the very next clock, far
sooner than the CPU's next instruction, so back-to-back `force()` calls are
safe. On an *enabled* SM with a large clock divider the previous forced
instruction may still be pending -- poll EXECCTRL[31] first (`wait_force_done`)
or keep such SMs disabled while you force.
"""
from asm_pineapple import PagedAsm

PIO_DATA = 0xFE
PIO_IDX = 0xFF

# ---- register-index map (mirrors rtl pio.v / test/tb_pio_common.vh) ----
R_CTRL, R_IRQ, R_IRQ_FORCE, R_FSTAT = 0x00, 0x01, 0x02, 0x03
R_PIN_OWN, R_SYNC_BYP, R_PINS_IN, R_PINS_OUT, R_INFO = 0x04, 0x05, 0x06, 0x07, 0x08
R_IMEM = 0x20
SM_CLKDIV, SM_EXEC, SM_SHIFT, SM_PINCTRL = 0, 1, 2, 3
SM_INSTR, SM_ADDR, SM_TXF, SM_RXF, SM_FLEVEL = 4, 5, 6, 7, 8


def sm_reg(sm, r):
    """Register index of state-machine `sm`'s register `r`."""
    return 0x40 + 0x10 * sm + r


def pinctrl(out_base=0, out_count=0, set_base=0, set_count=5,
            side_base=0, side_count=0, in_base=0):
    return ((out_base & 0xF) | ((set_base & 0xF) << 5) | ((side_base & 0xF) << 10)
            | ((in_base & 0xF) << 15) | ((out_count & 0xF) << 20)
            | ((set_count & 0x7) << 26) | ((side_count & 0x7) << 29))


def execctrl(wrap_bottom=0, wrap_top=31, side_en=False, side_pindir=False,
             jmp_pin=0, status_sel=0, status_n=0):
    return ((status_n & 0xF) | ((status_sel & 1) << 4) | ((wrap_bottom & 0x1F) << 7)
            | ((wrap_top & 0x1F) << 12) | ((jmp_pin & 0xF) << 24)
            | (int(side_pindir) << 29) | (int(side_en) << 30))


def shiftctrl(autopush=False, autopull=False, in_right=True, out_right=True,
              push_thresh=0, pull_thresh=0):
    return ((int(autopush) << 16) | (int(autopull) << 17) | (int(in_right) << 18)
            | (int(out_right) << 19) | ((push_thresh & 0x1F) << 20)
            | ((pull_thresh & 0x1F) << 25))


def clkdiv_reg(int_part=1, frac=0):
    """CLKDIV register value: 16-bit integer, 8-bit fraction (RP2040 layout)."""
    return ((int_part & 0xFFFF) << 16) | ((frac & 0xFF) << 8)


def _s12(v):
    v &= 0xFFF
    return v - 0x1000 if v & 0x800 else v


def li_plan(value):
    """('addi', imm) or ('lui', hi20, lo12) sequence that loads a 32-bit value."""
    value &= 0xFFFFFFFF
    sval = value - (1 << 32) if value & 0x80000000 else value
    if -2048 <= sval < 2048:
        return [("addi", sval)]
    lo = _s12(sval)
    hi = ((sval - lo) >> 12) & 0xFFFFF
    return [("lui", hi)] + ([("addi_r", lo)] if lo else [])


class PioHost:
    TMP = 5           # scratch register for constants

    def __init__(self, paged=None):
        self.p = paged or PagedAsm()
        self._page_no = 0

    # ---- page management --------------------------------------------------
    def _room(self, nbytes):
        # PagedAsm needs its last usable word for the page-number ADDI
        return self.p.bytes_used() + nbytes <= self.p.switch_offset - 4

    def _reserve(self, nbytes):
        if not self._room(nbytes):
            assert nbytes <= self.p.switch_offset - 4 - 4, "operation too large for one page"
            self._page_no += 1
            self.p.switch_to(self._page_no)

    # ---- primitive instruction sequences -------------------------------------
    def _li(self, rd, value):
        a = self.p.page
        for step in li_plan(value):
            if step[0] == "addi":
                a.ADDI(rd, 0, step[1])
            elif step[0] == "lui":
                a.LUI(rd, step[1])
            else:
                a.ADDI(rd, rd, step[1])

    @staticmethod
    def _li_bytes(value):
        return 4 * len(li_plan(value))

    # ---- PIO register operations (each is atomic within one page) -------------
    def write_idx(self, idx, autoinc=False):
        """Point PIO_IDX at register `idx` (bit 7 = auto-increment)."""
        v = (idx & 0x7F) | (0x80 if autoinc else 0)
        self._reserve(self._li_bytes(v) + 4)
        self._li(self.TMP, v)
        self.p.page.SW(self.TMP, 0, PIO_IDX)

    def write_data(self, value):
        """Store a 32-bit word to PIO_DATA (register selected by write_idx)."""
        self._reserve(self._li_bytes(value) + 4)
        self._li(self.TMP, value)
        self.p.page.SW(self.TMP, 0, PIO_DATA)

    def read_data(self, rd):
        """LW rd <- PIO_DATA."""
        self._reserve(4)
        self.p.page.LW(rd, 0, PIO_DATA)

    def write_reg(self, idx, value):
        self.write_idx(idx)
        self.write_data(value)

    # ---- conveniences ---------------------------------------------------------
    def load_program(self, prog):
        """Stream an assembled pioasm Program into instruction memory."""
        self.write_idx(R_IMEM + prog.origin, autoinc=True)
        for w in prog.instrs:
            self.write_data(w)
        self.write_idx(0)                      # auto-increment off again

    def force(self, sm, instr):
        """Execute one PIO instruction on state machine `sm` right now."""
        self.write_reg(sm_reg(sm, SM_INSTR), instr & 0xFFFF)

    def tx_push(self, sm, *words):
        self.write_idx(sm_reg(sm, SM_TXF))
        for w in words:
            self.write_data(w)

    def wait_tx_ready(self, sm):
        """Spin until TX FIFO `sm` has room (FSTAT.TXFULL[sm] == 0).  One atomic page op:
        select FSTAT, then  LUI mask ; L: LW/AND/BNE L.  Leaves PIO_IDX pointing at FSTAT."""
        nbytes = self._li_bytes(R_FSTAT) + 4 + 4 * 4
        self._reserve(nbytes)
        self._li(self.TMP, R_FSTAT)
        a = self.p.page
        a.SW(self.TMP, 0, PIO_IDX)
        self._wait_n = getattr(self, "_wait_n", 0) + 1
        lbl = "txw%d" % self._wait_n
        a.LUI(6, 1 << (16 + sm - 12))                # mask = 1 << (16 + sm)
        a.label(lbl)
        a.LW(7, 0, PIO_DATA)
        a.AND(7, 7, 6)
        a.BNE(7, 0, lbl)

    def tx_push_paced(self, sm, *words):
        """Like tx_push, but waits for FIFO room before every word (the CPU can outrun a slow
        state machine; the 4-deep TX FIFO silently drops writes when full)."""
        for w in words:
            self.wait_tx_ready(sm)
            self.write_idx(sm_reg(sm, SM_TXF))
            self.write_data(w)

    def wait_rx_ready(self, sm):
        """Spin until RX FIFO `sm` holds a word (FSTAT.RXEMPTY[sm] == 0).  One atomic page op,
        the mirror image of wait_tx_ready.  Leaves PIO_IDX pointing at FSTAT."""
        nbytes = self._li_bytes(R_FSTAT) + 4 + self._li_bytes(1 << (8 + sm)) + 4 * 3
        self._reserve(nbytes)
        self._li(self.TMP, R_FSTAT)
        a = self.p.page
        a.SW(self.TMP, 0, PIO_IDX)
        self._wait_n = getattr(self, "_wait_n", 0) + 1
        lbl = "rxw%d" % self._wait_n
        self._li(6, 1 << (8 + sm))                    # mask = RXEMPTY[sm]
        a.label(lbl)
        a.LW(7, 0, PIO_DATA)
        a.AND(7, 7, 6)
        a.BNE(7, 0, lbl)                              # still empty -> keep spinning

    def wait_sm_idle(self, sm, pc):
        """Spin until state machine `sm` has consumed every TX word AND is back at instruction
        `pc` (its command-loop top, where it stalls on an empty `pull`).  Two atomic page ops.
        Use it when the SM keeps working AFTER it produced the last RX word (e.g. trailing TMS
        clocks): the RX word alone does not mean the sequence is finished.  Order matters: the TX
        FIFO is checked first, so once it is empty no further word can restart the loop, and a
        later PC == pc means the last command ran to completion.  Leaves PIO_IDX on SM_ADDR."""
        # -- TX level (FLEVEL[2:0]) == 0
        self._reserve(self._li_bytes(sm_reg(sm, SM_FLEVEL)) + 4 + 4 * 4)
        self._li(self.TMP, sm_reg(sm, SM_FLEVEL))
        a = self.p.page
        a.SW(self.TMP, 0, PIO_IDX)
        self._wait_n = getattr(self, "_wait_n", 0) + 1
        lbl = "idl%d" % self._wait_n
        a.label(lbl)
        a.LW(7, 0, PIO_DATA)
        a.ANDI(7, 7, 7)
        a.BNE(7, 0, lbl)
        # -- ADDR[4:0] == pc
        self._reserve(self._li_bytes(sm_reg(sm, SM_ADDR)) + 4 + 4 * 5)
        self._li(self.TMP, sm_reg(sm, SM_ADDR))
        a = self.p.page
        a.SW(self.TMP, 0, PIO_IDX)
        lbl2 = "idp%d" % self._wait_n
        a.label(lbl2)
        a.LW(7, 0, PIO_DATA)
        a.ANDI(7, 7, 31)
        a.ADDI(7, 7, -pc)
        a.BNE(7, 0, lbl2)

    def rx_pop(self, sm, rd):
        """rd <- RX FIFO `sm` head word (the read pops it).  Call wait_rx_ready first."""
        self.write_idx(sm_reg(sm, SM_RXF))
        self.read_data(rd)

    def shift_left(self, rd, sh):
        """rd <<= sh  (e.g. move an 8-bit RX byte into the top byte an MSB-first OSR expects)."""
        self._reserve(4)
        self.p.page.SLLI(rd, rd, sh)

    def shift_right(self, rd, sh):
        """rd >>= sh (logical)."""
        self._reserve(4)
        self.p.page.SRLI(rd, rd, sh)

    def store_data_reg(self, rs):
        """SW rs -> PIO_DATA: push a CPU register value into whatever PIO_IDX selects."""
        self._reserve(4)
        self.p.page.SW(rs, 0, PIO_DATA)

    def tx_push_reg_paced(self, sm, rs):
        """Wait for TX room, then push the value held in register `rs` (the CPU relays data)."""
        self.wait_tx_ready(sm)
        self.write_idx(sm_reg(sm, SM_TXF))
        self.store_data_reg(rs)

    def write_gpio_out_imm(self, value):
        """LED_OUT (0xF0) <- immediate byte (drives the uo_out pads that PIO does not own)."""
        self._reserve(self._li_bytes(value) + 4)
        self._li(self.TMP, value)
        self.p.page.SB(self.TMP, 0, 0xF0)

    def delay_iterations(self, n):
        """Busy-wait for exactly `n` passes of a 2-instruction countdown loop (ADDI / BNE), one atomic
        op. MEASURED on this core with the testbenches' fast QSPI setting (qspi_div_sel = 0): about
        274 core clocks per pass (~137 per instruction: the loop runs out of the paged flash window),
        so n = 560 sleeps about 153000 clocks. With a slower QSPI divider it is slower still -- re-measure
        before relying on a particular duration."""
        n = max(1, int(n))
        self._reserve(self._li_bytes(n) + 8)
        self._li(self.TMP, n)
        self._delay_n = getattr(self, "_delay_n", 0) + 1
        lbl = "dly%d" % self._delay_n
        a = self.p.page
        a.label(lbl)
        a.ADDI(self.TMP, self.TMP, -1)
        a.BNE(self.TMP, 0, lbl)

    def delay(self, clocks):
        """Busy-wait for AT LEAST `clocks` core clocks: clocks // 10 passes of the countdown loop.
        A pass really takes ~274 clocks (see delay_iterations), so this over-waits by ~27x --
        harmless where a protocol merely has to be finished, wrong where timing matters (use
        delay_iterations there)."""
        self.delay_iterations(max(1, clocks // 10))

    def rx_get(self, sm, rd):
        """Alias of rx_pop (kept for build_pio_i2c_slave.py)."""
        self.rx_pop(sm, rd)

    def byte_plus1_to_i2c_slave_word(self, rd):
        """rd = RX word (byte in bits [7:0]) -> TX word that makes the I2C-slave program send
        (byte + 1): ((~(byte + 1)) & 0xFF) << 24  (PINDIR polarity, see pio_i2c_slave.tx_word)."""
        self._reserve(5 * 4)
        a = self.p.page
        a.ANDI(rd, rd, 0xFF)
        a.ADDI(rd, rd, 1)
        a.XORI(rd, rd, -1)
        a.ANDI(rd, rd, 0xFF)
        a.SLLI(rd, rd, 24)

    def ps2_frame_to_byte(self, rd):
        """rd = RX word of pio/ps2_rx.pio (frame in bits [31:21]) -> the data byte, (word >> 22) & 0xFF.
        Start / parity / stop are not judged here (see decode_ps2_word in test/pio_tb_lib.py)."""
        self._reserve(2 * 4)
        a = self.p.page
        a.SRLI(rd, rd, 22)
        a.ANDI(rd, rd, 0xFF)

    def write_data_reg(self, rs):
        """SW rs -> PIO_DATA (the register picked by the last write_idx)."""
        self._reserve(4)
        self.p.page.SW(rs, 0, PIO_DATA)

    def write_gpio_out(self, reg, addr=0xF0):
        """SB reg -> LED_OUT (0xF0)."""
        self._reserve(4)
        self.p.page.SB(reg, 0, addr)

    def halt(self):
        """EBREAK: park the CPU. The PIO state machines keep running."""
        self._reserve(4)
        self.p.page.EBREAK()

    def build(self):
        """Close the final page and return the flat flash image (bytes)."""
        self.p.finalize_last_page()
        return self.p.finalize()

    @property
    def pages(self):
        return self._page_no + 1


def write_image(image, stem):
    """Write `<stem>.bin` and `<stem>.hex` (one byte per line, $readmemh format)."""
    with open(stem + ".bin", "wb") as f:
        f.write(image)
    with open(stem + ".hex", "w") as f:
        for b in image:
            f.write("%02x\n" % b)
