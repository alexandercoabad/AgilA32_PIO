"""pio_tb_lib.py -- cocotb helpers for exercising pio.v at the protocol level.

  * PioBus   : register access exactly the way the RV32I core does it
               (a store = one clock of valid&we; a load = one clock of valid, data
               sampled one clock later, RXF pops applied a clock after that).
  * World    : cycle-accurate model of the pads.  Pins 0-7 are ui_in (input) / uo_out
               (output); pins 8-9 are uio[4]/uio[5] (bidirectional, real output enable).
               Pins 8/9 are an open-drain wired-AND bus with a pull-up, so a slave can
               share them with the PIO.
  * Peers    : SerialSource (UART/anything bit-banged in), SpiSlave (modes 0-3),
               I2cSlave (start/stop/ack/read/clock-stretch, with protocol-rule checks).
"""
import os
import sys

from cocotb.triggers import ClockCycles, RisingEdge, Timer

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from pioasm import assemble                                    # noqa: E402,F401
from pio_host import (sm_reg, pinctrl, shiftctrl, clkdiv_reg,  # noqa: E402,F401
                      R_CTRL, R_IRQ, R_FSTAT, R_PIN_OWN, R_SYNC_BYP, R_PINS_IN,
                      R_PINS_OUT, R_INFO, R_IMEM, SM_CLKDIV, SM_EXEC, SM_SHIFT,
                      SM_PINCTRL, SM_INSTR, SM_ADDR, SM_TXF, SM_RXF, SM_FLEVEL)

PIO_DATA = 0xFE
PIO_IDX = 0xFF
FIFO_DEPTH = 4


# ----------------------------------------------------------------------------- bus
class PioBus:
    def __init__(self, dut):
        self.d = dut
        self.en_mask = 0

    async def _cyc(self, valid, we, addr, wdata=0):
        d = self.d
        d.valid.value = valid
        d.we.value = we
        d.addr.value = addr
        d.wdata.value = wdata
        await RisingEdge(d.clk)
        d.valid.value = 0
        d.we.value = 0

    async def w_idx(self, idx, autoinc=False):
        await self._cyc(1, 1, PIO_IDX, (0x80 if autoinc else 0) | (idx & 0x7F))

    async def w_data(self, val):
        await self._cyc(1, 1, PIO_DATA, val & 0xFFFFFFFF)

    async def r_data(self):
        await self._cyc(1, 0, PIO_DATA)
        await Timer(1, unit="ns")           # core samples one clock after `valid`
        v = int(self.d.rdata.value)
        await RisingEdge(self.d.clk)        # deferred side effects (pop / auto-inc) land here
        return v

    async def write(self, idx, val):
        await self.w_idx(idx)
        await self.w_data(val)

    async def read(self, idx):
        await self.w_idx(idx)
        return await self.r_data()

    # ------------------------------------------------------------------ helpers
    async def load_program(self, prog):
        await self.w_idx(R_IMEM + prog.origin, autoinc=True)
        for w in prog.instrs:
            await self.w_data(w)

    async def force(self, sm, instr):
        await self.write(sm_reg(sm, SM_INSTR), instr)
        await ClockCycles(self.d.clk, 3)

    async def set_enable(self, sm, on=True):
        if on:
            self.en_mask |= 1 << sm
        else:
            self.en_mask &= ~(1 << sm)
        await self.write(R_CTRL, self.en_mask)

    async def fifo_clear(self, sm):
        await self.write(R_CTRL, self.en_mask | (1 << (12 + sm)))
        await self.write(R_CTRL, self.en_mask)

    async def sm_setup(self, sm, prog, *, out_base=0, out_count=0, set_base=0, set_count=5,
                       side_base=0, in_base=0, jmp_pin=0, autopush=False, autopull=False,
                       in_right=True, out_right=True, push_thresh=0, pull_thresh=0,
                       div=(1, 0), entry=None):
        """Configure (but do not enable) a state machine for `prog`."""
        await self.write(sm_reg(sm, SM_CLKDIV), clkdiv_reg(*div))
        await self.write(sm_reg(sm, SM_EXEC), prog.execctrl(jmp_pin=jmp_pin))
        await self.write(sm_reg(sm, SM_SHIFT), shiftctrl(autopush, autopull, in_right, out_right,
                                                        push_thresh, pull_thresh))
        await self.write(sm_reg(sm, SM_PINCTRL),
                         pinctrl(out_base, out_count, set_base, set_count, side_base,
                                 prog.side_bits, in_base))
        await self.force(sm, prog.wrap_bottom if entry is None else entry)   # `jmp entry`

    async def tx_level(self, sm):
        return (await self.read(sm_reg(sm, SM_FLEVEL))) & 7

    async def rx_level(self, sm):
        return ((await self.read(sm_reg(sm, SM_FLEVEL))) >> 4) & 7

    async def tx_put(self, sm, word):
        await self.write(sm_reg(sm, SM_TXF), word)

    async def rx_get(self, sm):
        return await self.read(sm_reg(sm, SM_RXF))

    async def tx_put_blocking(self, sm, word, limit=200000):
        for _ in range(limit):
            if await self.tx_level(sm) < FIFO_DEPTH:
                await self.tx_put(sm, word)
                return
        raise TimeoutError("TX FIFO never drained")


# ----------------------------------------------------------------------------- world
class World:
    """Pad model.  Devices implement step(world) and run right after every clock edge."""

    def __init__(self, dut):
        self.d = dut
        self.ext_in = 0x000          # value driven onto pads 0-7 by the outside world
        self.ext_low = 0x000         # open-drain pull-downs by external devices (pads 8/9)
        self.devs = []
        self.cycle = 0
        self.pin_out = self.pin_dir = self.own = 0
        self.mlow = 0
        self.out_trace = []          # pin_out each cycle (bits are meaningful where owned)
        self.bus_trace = []          # resolved pads 8/9 each cycle (bit0=SDA, bit1=SCL)
        self.irq_seen = 0

    def resolve(self):
        return 0x300 & ~(self.mlow | self.ext_low)

    def pins_raw(self):
        return (self.ext_in & 0xFF) | self.resolve()

    async def run(self):
        d = self.d
        while True:
            await RisingEdge(d.clk)
            self.pin_out = int(d.pin_out.value)
            self.pin_dir = int(d.pin_dir.value)
            self.own = int(d.pin_own.value)
            self.mlow = self.own & self.pin_dir & ~self.pin_out & 0x300
            self.cycle += 1
            for dev in self.devs:
                dev.step(self)
            self.out_trace.append(self.pin_out)
            self.bus_trace.append(((self.resolve() >> 8) & 3))
            d.pins_raw.value = self.pins_raw()

    def bit_trace(self, pin):
        return [(v >> pin) & 1 for v in self.out_trace]


class Wire:
    """Connects an owned PIO output pin to a PIO input pin (loopback)."""

    def __init__(self, src, dst):
        self.src, self.dst = src, dst

    def step(self, w):
        v = (w.pin_out >> self.src) & 1
        w.ext_in = (w.ext_in & ~(1 << self.dst)) | (v << self.dst)


class SerialSource:
    """Plays a list of (level, n_cycles) onto an input pin, then idles at `idle`."""

    def __init__(self, pin, idle=1):
        self.pin, self.idle = pin, idle
        self.q = []
        self.cur = idle
        self.left = 0
        self.busy = False

    def add_uart_frame(self, byte, bit_clk=8.0, stop=1, gap=0.0):
        bits = [0] + [(byte >> k) & 1 for k in range(8)] + [stop]
        edge = 0
        for k, b in enumerate(bits):
            nxt = int(round((k + 1) * bit_clk))
            self.q.append((b, nxt - edge))
            edge = nxt
        if gap:
            self.q.append((1, int(round(gap * bit_clk))))

    def step(self, w):
        if self.left == 0 and self.q:
            self.cur, self.left = self.q.pop(0)
        if self.left:
            self.left -= 1
            self.busy = True
        else:
            self.cur = self.idle
            self.busy = False
        w.ext_in = (w.ext_in & ~(1 << self.pin)) | (self.cur << self.pin)


class Ps2Keyboard:
    """PS/2 device driving CLOCK (pin 3) and DATA (pin 4), both idle high (ui_in[3] / ui_in[4]).

    One bit cell = 2*half clocks: DATA changes while CLOCK is high, CLOCK falls half//2 later
    (data setup), stays low for `half`, then rises and DATA is held for the rest of the cell
    (hold), like a real keyboard.  Frame = start(0), 8 data bits LSB first, odd parity, stop(1).
    The host samples DATA on the FALLING edge of CLOCK."""

    def __init__(self, clk_pin=3, data_pin=4):
        self.clk_pin, self.data_pin = clk_pin, data_pin
        self.q = []
        self.left = 0
        self.cur = (1, 1)
        self.busy = False

    def add_frame(self, byte, half=100, gap=0, *, parity=None, start=0, stop=1, nbits=11):
        """Queue one frame.  `parity`/`start`/`stop` override the legal values (fault injection);
        `nbits` < 11 truncates the frame (the device stops clocking mid-frame)."""
        ones = bin(byte & 0xFF).count("1")
        p = (1 ^ (ones & 1)) if parity is None else parity         # odd parity over the data bits
        bits = ([start] + [(byte >> k) & 1 for k in range(8)] + [p, stop])[:nbits]
        setup = max(2, half // 2)
        for b in bits:
            self.q += [(1, b, setup), (0, b, half), (1, b, half - setup)]
        if gap:
            self.q.append((1, 1, gap))

    def add_idle(self, cycles):
        self.q.append((1, 1, cycles))

    def step(self, w):
        if self.left == 0 and self.q:
            clk, data, self.left = self.q.pop(0)
            self.cur = (clk, data)
        if self.left:
            self.left -= 1
            self.busy = True
        else:
            self.cur = (1, 1)
            self.busy = False
        clk, data = self.cur
        mask = (1 << self.clk_pin) | (1 << self.data_pin)
        w.ext_in = (w.ext_in & ~mask) | (clk << self.clk_pin) | (data << self.data_pin)


def decode_ps2_word(word):
    """`pio/ps2_rx.pio` autopushes 11 bits shifted right, so the frame sits in bits [31:21]:
    start[0] data[8:1] parity[9] stop[10] of (word >> 21).  Returns (data, frame_ok) where
    frame_ok = start low, stop high and odd parity over data+parity."""
    f = (word >> 21) & 0x7FF
    start, data, parity, stop = f & 1, (f >> 1) & 0xFF, (f >> 9) & 1, (f >> 10) & 1
    ok = start == 0 and stop == 1 and ((bin(data).count("1") + parity) & 1) == 1
    return data, ok


# ----------------------------------------------------------------------------- UART decode
def decode_uart(trace, bit_clk, nframes=None, start_at=0):
    """Decode 8N1 LSB-first frames from a per-cycle bit trace, checking exact bit timing.
    Returns (bytes, problems)."""
    out, problems = [], []
    i = start_at
    while i < len(trace) - 1:
        if trace[i] == 1 and trace[i + 1] == 0:
            s = i + 1
            if s + int(10 * bit_clk) + 1 > len(trace):
                break
            for k in range(10):
                lo, hi = int(round(s + k * bit_clk)), int(round(s + (k + 1) * bit_clk))
                seg = trace[lo:hi]
                if len(set(seg)) != 1:
                    problems.append("frame %d bit %d not stable for %d cycles: %s"
                                    % (len(out), k, hi - lo, seg))
            mid = lambda k: trace[int(s + (k + 0.5) * bit_clk)]      # noqa: E731
            if mid(0) != 0:
                problems.append("start bit not low")
            if mid(9) != 1:
                problems.append("stop bit not high")
            out.append(sum(mid(k + 1) << k for k in range(8)))
            i = int(round(s + 10 * bit_clk)) - 1
            if nframes and len(out) >= nframes:
                break
        else:
            i += 1
    return out, problems


# ----------------------------------------------------------------------------- SPI slave
class SpiSlave:
    """SPI slave on MOSI=pin0 (PIO out), SCK=pin1 (PIO out), MISO=pin2 (into the PIO).
    mode = CPOL*2 + CPHA.
      mode 0: idle low,  sample on rising SCK,  shift on falling.
      mode 1: idle low,  shift on rising,       sample on falling.
      mode 2: idle high, sample on falling SCK, shift on rising.
      mode 3: idle high, shift on falling,      sample on rising.
    With CPHA = 0 the first bit must already be valid before the first (leading) edge."""

    def __init__(self, mode, miso_bytes, mosi_pin=0, sck_pin=1, miso_pin=2):
        assert mode in (0, 1, 2, 3)
        self.mode, self.miso_bytes = mode, list(miso_bytes)
        self.cpol, self.cpha = mode >> 1, mode & 1
        self.sample_rise = self.cpol == self.cpha
        self.mosi_pin, self.sck_pin, self.miso_pin = mosi_pin, sck_pin, miso_pin
        self.prev_sck = 0
        self.idle_sck = None            # SCK level when the slave first looked (must equal CPOL)
        self.bit = 0
        self.bidx = 0
        self.shift = 0
        self.mosi_bytes = []
        self.rise_cycles = []
        self.fall_cycles = []
        self.started = False
        self.prev_mosi = None
        self.last_mosi_change = None    # cycle of the most recent MOSI level change
        self.last_sample = None         # cycle of the most recent sampling edge
        self.setup = []                 # clocks MOSI was stable before each sampling edge
        self.hold = []                  # clocks MOSI stayed stable after a sampling edge

    def _miso(self):
        if self.bidx < len(self.miso_bytes):
            return (self.miso_bytes[self.bidx] >> (7 - self.bit)) & 1
        return 0

    def _drive(self, w):
        w.ext_in = (w.ext_in & ~(1 << self.miso_pin)) | (self._miso() << self.miso_pin)

    def _sample(self, w):
        if self.last_mosi_change is not None:
            self.setup.append(w.cycle - self.last_mosi_change)
        self.last_sample = w.cycle
        self.shift = ((self.shift << 1) | ((w.pin_out >> self.mosi_pin) & 1)) & 0xFF
        self.bit += 1
        if self.bit == 8:
            self.mosi_bytes.append(self.shift)
            self.bit = 0
            self.bidx += 1

    def step(self, w):
        sck = (w.pin_out >> self.sck_pin) & 1
        if not self.started:
            self.started = True
            self.prev_sck = sck
            self.idle_sck = sck
            if self.cpha == 0:
                self._drive(w)                      # first bit must be valid before the first edge
        rise, fall = sck and not self.prev_sck, self.prev_sck and not sck
        mosi = (w.pin_out >> self.mosi_pin) & 1
        if self.prev_mosi is not None and mosi != self.prev_mosi:
            self.last_mosi_change = w.cycle
            if self.last_sample is not None:
                self.hold.append(w.cycle - self.last_sample)
        self.prev_mosi = mosi
        if rise:
            self.rise_cycles.append(w.cycle)
        if fall:
            self.fall_cycles.append(w.cycle)
        if self.sample_rise:
            if rise:
                self._sample(w)
            if fall:
                self._drive(w)
        else:
            if rise:
                self._drive(w)
            if fall:
                self._sample(w)
        self.prev_sck = sck


# ----------------------------------------------------------------------------- I2C slave
class I2cSlave:
    """Open-drain I2C slave on pads 8 (SDA) / 9 (SCL); flags protocol violations."""

    def __init__(self, addr=0x50, read_bytes=(), stretch=0):
        self.addr = addr
        self.read_bytes = list(read_bytes)
        self.stretch = stretch
        self.mode = "idle"
        self.bitcnt = 0
        self.shift = 0
        self.sda_low = 0
        self.scl_low = 0
        self.stretch_left = 0
        self.prev_scl = 1
        self.prev_sda = 1
        self.rx = []                 # ("A"|"D", byte)
        self.starts = self.stops = 0
        self.violations = []
        self.scl_edges = []          # (cycle, level) of resolved SCL
        self.rw = 0
        self.last_was_addr = False
        self.last_ack = False
        self.tx_idx = 0
        self.tx_byte = 0
        self.master_ack = False

    def _apply(self, w):
        w.ext_low = (w.ext_low & ~0x300) | (0x100 if self.sda_low else 0) | (0x200 if self.scl_low else 0)

    def _drive_bit(self, i):
        self.sda_low = 0 if (self.tx_byte >> (7 - i)) & 1 else 1

    def _load_tx(self):
        self.tx_byte = self.read_bytes[self.tx_idx] if self.tx_idx < len(self.read_bytes) else 0xFF
        self.tx_idx += 1
        self.bitcnt = 0

    def step(self, w):
        lines = w.resolve()
        sda, scl = (lines >> 8) & 1, (lines >> 9) & 1
        if self.stretch_left:
            self.stretch_left -= 1
            if not self.stretch_left:
                self.scl_low = 0
        start = self.prev_scl and scl and self.prev_sda and not sda
        stop = self.prev_scl and scl and (not self.prev_sda) and sda
        rise, fall = scl and not self.prev_scl, self.prev_scl and not scl
        if scl != self.prev_scl:
            self.scl_edges.append((w.cycle, scl))
        if scl and self.prev_scl and sda != self.prev_sda and not (start or stop):
            self.violations.append("cycle %d: SDA changed while SCL high" % w.cycle)
        if start:
            self.starts += 1
            self.mode, self.bitcnt, self.shift, self.sda_low = "addr", 0, 0, 0
        elif stop:
            self.stops += 1
            self.mode, self.sda_low = "idle", 0
        else:
            if rise:
                self._rise(sda)
            if fall:
                self._fall()
        self.prev_scl, self.prev_sda = scl, sda
        self._apply(w)

    def _rise(self, sda):
        if self.mode in ("addr", "rxdata"):
            if self.bitcnt < 8:
                self.shift = ((self.shift << 1) | sda) & 0xFF
                self.bitcnt += 1
            else:
                self.bitcnt = 9
        elif self.mode == "txdata":
            if self.bitcnt < 8:
                self.bitcnt += 1
            else:
                self.bitcnt = 9
                self.master_ack = (sda == 0)

    def _fall(self):
        if self.mode in ("addr", "rxdata"):
            if self.bitcnt == 8:
                byte = self.shift
                self.last_was_addr = (self.mode == "addr")
                if self.last_was_addr:
                    self.rw = byte & 1
                    self.last_ack = (byte >> 1) == self.addr
                    self.rx.append(("A", byte))
                else:
                    self.last_ack = True
                    self.rx.append(("D", byte))
                if self.last_ack:
                    self.sda_low = 1
                    if self.stretch:
                        self.scl_low, self.stretch_left = 1, self.stretch
            elif self.bitcnt == 9:
                self.sda_low = 0
                self.bitcnt, self.shift = 0, 0
                if not self.last_ack:
                    self.mode = "ignore"
                elif self.last_was_addr and self.rw:
                    self.mode = "txdata"
                    self._load_tx()
                    self._drive_bit(0)
                else:
                    self.mode = "rxdata"
        elif self.mode == "txdata":
            if self.bitcnt < 8:
                self._drive_bit(self.bitcnt)
            elif self.bitcnt == 8:
                self.sda_low = 0
            else:
                if self.master_ack:
                    self._load_tx()
                    self._drive_bit(0)
                else:
                    self.mode, self.sda_low = "ignore", 0
