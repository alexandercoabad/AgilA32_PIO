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
from functools import reduce
from operator import or_

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
        self.ext_low = 0x000         # external devices pulling pads 8/9 low
        self.ext_high = 0x000        # external devices driving pads 8/9 high (push-pull, e.g. USB)
        self.pull = 0x300            # idle level of pads 8/9 when nobody drives (I2C: both pulled up;
                                     # low-speed USB: D+ pulled down, D- pulled up -> 0x200)
        self.devs = []
        self.cycle = 0
        self.pin_out = self.pin_dir = self.own = 0
        self.mlow = 0
        self.out_trace = []          # pin_out each cycle (bits are meaningful where owned)
        self.bus_trace = []          # resolved pads 8/9 each cycle (bit0=SDA, bit1=SCL)
        self.irq_seen = 0

    def resolve(self):
        drv = self.own & self.pin_dir & 0x300                  # pads the PIO actually drives
        val = (self.pull & ~drv) | (self.pin_out & drv)        # PINDIR = 1 drives the pin_out value
        val = (val | self.ext_high) & ~self.ext_low
        return val & 0x300

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
            # several open-drain devices may share pads 8/9 (wired-AND): OR their pull-downs
            lows = [dev.low for dev in self.devs if hasattr(dev, "low")]
            if lows:
                self.ext_low = (self.ext_low & ~0x300) | (reduce(or_, lows) & 0x300)
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


class Ws2812Strip:
    """WS2812 / SK6812 receiver watching one pin (the PIO's side-set pin).  Like the real chip it
    decodes each bit from the width of its HIGH pulse (>= `thresh_ns` is a 1) and treats a LOW of at
    least `reset_us` as the latch (end of frame).  It records every pulse in clocks so tests can check
    the datasheet windows, and every frame as a list of bits.
        high_clk[i], low_clk[i]   high width of bit i / low time AFTER bit i (to the next rising edge;
                                  for the last bit of a frame: until the latch, measured at the latch)
        frames                    list of closed frames (each a list of 0/1 bits)
        cur                       bits of the frame still open (not yet latched)"""

    def __init__(self, pin=0, clk_ns=41.667, reset_us=50.0, thresh_ns=600.0):
        self.pin, self.clk_ns = pin, clk_ns
        self.reset_clk = int(reset_us * 1000.0 / clk_ns)
        self.thresh_clk = thresh_ns / clk_ns
        self.level = 0
        self.rise = None            # cycle of the last rising edge
        self.fall = None            # cycle of the last falling edge
        self.high_clk, self.low_clk, self.period_clk = [], [], []
        self.frame_of_bit = []      # frame index each recorded bit belongs to
        self.frames, self.cur = [], []
        self.glitches = 0           # pulses shorter than 2 clocks

    def step(self, w):
        lvl = (w.pin_out >> self.pin) & 1
        if lvl and not self.level:                           # rising edge
            if self.fall is not None and self.low_clk and self.cur:
                self.low_clk[-1] = w.cycle - self.fall       # low time after the previous bit
            if self.rise is not None and self.cur:
                self.period_clk.append(w.cycle - self.rise)
            self.rise = w.cycle
        elif self.level and not lvl:                         # falling edge: one bit complete
            width = w.cycle - self.rise
            if width < 2:
                self.glitches += 1
            self.high_clk.append(width)
            self.low_clk.append(0)
            self.cur.append(1 if width >= self.thresh_clk else 0)
            self.frame_of_bit.append(len(self.frames))
            self.fall = w.cycle
        elif not lvl and self.fall is not None and self.cur and w.cycle - self.fall >= self.reset_clk:
            self.low_clk[-1] = w.cycle - self.fall           # long idle: latch
            self.frames.append(self.cur)
            self.cur = []
        self.level = lvl

    @staticmethod
    def pixels(bits, nbits=24):
        """Split a frame's bits into pixel words, MSB first."""
        assert len(bits) % nbits == 0, "frame of %d bits is not a whole number of %d-bit pixels" % (len(bits), nbits)
        return [int("".join(map(str, bits[i:i + nbits])), 2) for i in range(0, len(bits), nbits)]


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

    @property
    def low(self):
        return (0x100 if self.sda_low else 0) | (0x200 if self.scl_low else 0)

    def _apply(self, w):
        w.ext_low = (w.ext_low & ~0x300) | self.low

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


# ----------------------------------------------------------------------------- low-speed USB device
class UsbLsDevice:
    """Low-speed USB device on pads 8 (D+) / 9 (D-), 16 clocks per bit (8 PIO ticks x CLKDIV 2).
    Idle = J (D+ low, D- high, from the pad pulls: set world.pull = 0x200).

    * Records every host packet it sees (start of K -> sampled at bit centres -> SE0 ends it) in
      `rx_packets` as dicts {bits (stuffed logical bits), start, se0_start, gaps}.
    * After a host packet it sends the next entry of `responses` (each a list of stuffed logical
      bits, e.g. pio_usb.data_bits(...)), `turnaround` clocks after the host released the bus.
    * `spontaneous` = [(cycle, bits)] are sent unprompted (to test the receiver alone).
    """
    BIT = 16

    def __init__(self, responses=(), turnaround=24, spontaneous=(), scale=1.0):
        self.scale = scale                   # device bit period = 16 * scale clocks (clock error)
        self.responses = list(responses)
        self.turn = turnaround
        self.spont = sorted(spontaneous, key=lambda x: x[0])
        self.rx_packets = []
        self.sent = []                       # (start_cycle, bits) of packets the device sent
        self.state = "idle"
        self.wave = []
        self.resp_at = None
        self.pending = None
        self.samples = []
        self.next_sample = 0
        self.se0 = 0
        self.t0 = 0
        self.edges = []                      # (cycle, (dp, dm)) every time the resolved bus changed
        self._last = None

    # -- waveform helpers --------------------------------------------------------------
    def wave_of(self, bits):
        J, K, SE0 = (0, 1), (1, 0), (0, 0)
        lvl, out = J, []
        syms = []
        for b in bits:
            if b == 0:
                lvl = K if lvl == J else J
            syms.append(lvl)
        syms += [SE0, SE0, J]
        for i, sym in enumerate(syms):            # bit i ends at round((i+1) * BIT * scale)
            end = int(round((i + 1) * self.BIT * self.scale))
            out += [sym] * (end - len(out))
        return out

    def _drive(self, w, dp, dm):
        w.ext_high = (dp << 8) | (dm << 9)
        w.ext_low = ((1 - dp) << 8) | ((1 - dm) << 9)

    def _release(self, w):
        w.ext_high = w.ext_low = 0

    def step(self, w):
        v = (w.resolve() >> 8) & 3
        cur = (v & 1, (v >> 1) & 1)
        if cur != self._last:
            self.edges.append((w.cycle, cur))
            self._last = cur
        if self.wave:
            dp, dm = self.wave.pop(0)
            self._drive(w, dp, dm)
            if not self.wave:
                self._release(w)
                self.state = "idle"
            return
        if self.spont and w.cycle >= self.spont[0][0] and self.state == "idle":
            _, bits = self.spont.pop(0)
            self.sent.append((w.cycle, bits))
            self.wave = self.wave_of(bits)
            self.state = "tx"
            return
        dp, dm = cur
        if self.state == "idle":
            if dp == 1 and dm == 0:                      # K: start of SYNC
                self.state = "rx"
                self.t0 = w.cycle
                self.samples = []
                self.next_sample = w.cycle + self.BIT // 2
                self.se0 = 0
        elif self.state == "rx":
            self.se0 = self.se0 + 1 if (dp == 0 and dm == 0) else 0
            if w.cycle == self.next_sample:
                self.samples.append((dp, dm))
                self.next_sample += self.BIT
            if self.se0 >= 6:                            # EOP
                lvl, bits = (0, 1), []                   # start from J
                for s in self.samples:
                    if s == (0, 0):
                        break
                    bits.append(1 if s == lvl else 0)
                    lvl = s
                self.rx_packets.append(dict(bits=bits, start=self.t0, se0_start=w.cycle - 5))
                self.state = "wait"
                self.resp_at = w.cycle + (2 * self.BIT - 6) + self.BIT + self.turn
        elif self.state == "wait":
            if w.cycle >= self.resp_at:
                if self.responses:
                    bits = self.responses.pop(0)
                    self.sent.append((w.cycle, bits))
                    self.wave = self.wave_of(bits)
                    self.state = "tx"
                else:
                    self.state = "idle"
# ----------------------------------------------------------------------------- I2C master
class I2cMaster:
    """Behavioural multi-master-capable I2C master on pads 8 (SDA) / 9 (SCL).

    Runs a script of ("start",) ("repstart",) ("write", byte) ("read", ack) ("stop",) steps, one
    cooperative generator advanced every clock.  Everything a real multi-master must do is here:
      * clock synchronisation: it releases SCL, then WAITS until SCL is actually high (another
        master, or a stretching slave, may hold it low) and only then times its high phase;
      * arbitration: while it releases SDA for a 1 it watches the bus, and the moment SDA reads 0 it
        has lost -- it lets go of BOTH lines at once, sets `lost`, records the bit index, and stops;
      * bus-free wait before START (both lines high for `t_free` clocks).
    `period` is the SCL period in clocks; `start_at` delays the first action (absolute cycle).
    """

    def __init__(self, script, period=64, start_at=0, t_free=128, name="M2", join_start=False,
                 sample_delay=8, hold=None):
        self.script, self.period, self.start_at, self.t_free, self.name = script, period, start_at, t_free, name
        self.join_start = join_start        # START the moment another master's START is seen
        self.sample_delay = sample_delay
        self.hold = hold if hold is not None else period // 4   # SDA stays put this long after SCL falls
        self.low = 0                     # 0x100 = pulling SDA low, 0x200 = pulling SCL low
        self.sda = self.scl = 1
        self.cycle = 0
        self.lost = False
        self.lost_at = None              # (byte index, bit index) where arbitration was lost
        self.done = False
        self.acks = []                   # ACK level seen after each written byte (0 = ACK)
        self.reads = []
        self.stops = 0
        self._pend = None
        self._gen = self._run()
        self._byte_no = 0

    # -- cooperative scheduler -------------------------------------------------------------
    def step(self, w):
        lines = w.resolve()
        self.sda, self.scl = (lines >> 8) & 1, (lines >> 9) & 1
        self.cycle = w.cycle
        while not self.done:
            if self._pend is None:
                try:
                    self._pend = next(self._gen)
                except StopIteration:
                    self.done = True
                    return
            if isinstance(self._pend, int):
                if self._pend > 0:
                    self._pend -= 1
                    return
                self._pend = None
            else:
                if not self._pend():
                    return
                self._pend = None

    def _sda(self, level):
        self.low = (self.low & ~0x100) | (0 if level else 0x100)

    def _scl(self, level):
        self.low = (self.low & ~0x200) | (0 if level else 0x200)

    def _release_all(self):
        self.low = 0

    # -- protocol pieces -------------------------------------------------------------------
    def _wait_bus_free(self):
        idle = 0
        while idle < self.t_free:
            idle = idle + 1 if (self.sda and self.scl) else 0
            yield 1

    def _clock_high(self):
        """Release SCL, wait until it is really high (clock sync), then wait `sample_delay`
        clocks and return -- the caller samples SDA there.  Sampling early matters: another
        master with a shorter high phase may pull SCL low (and then change SDA) soon after."""
        self._scl(1)
        yield lambda: self.scl == 1
        yield self.sample_delay

    def _high_remaining(self, n):
        """Rest of our SCL high phase.  Clock synchronisation: if another master pulls SCL low
        first, our high phase ends right there (and our low phase counts from that fall)."""
        for _ in range(n):
            if self.scl == 0:
                return
            yield 1

    def _bit_out(self, b):
        q = self.period // 4
        self._sda(b)
        yield q
        yield from self._clock_high()
        if b and self.sda == 0:                       # released SDA but the bus is low: lost
            self.lost = True
            self.lost_at = (self._byte_no, self._bitno)
            self._release_all()
            return False
        yield from self._high_remaining(2 * q - self.sample_delay)
        self._scl(0)
        yield self.hold
        return True

    def _write_byte(self, byte):
        for i in range(8):
            self._bitno = i
            ok = yield from self._bit_out((byte >> (7 - i)) & 1)
            if not ok:
                return False
        # ACK slot: release SDA, read what the slave does
        q = self.period // 4
        self._sda(1)
        yield q
        yield from self._clock_high()
        self.acks.append(self.sda)
        yield from self._high_remaining(2 * q - self.sample_delay)
        self._scl(0)
        yield self.hold
        self._byte_no += 1
        return True

    def _read_byte(self, ack):
        """Release SDA for 8 bits and sample what the slave drives; then ACK (drive low) or NAK."""
        q = self.period // 4
        val = 0
        for _ in range(8):
            self._sda(1)
            yield q
            yield from self._clock_high()            # waits while a slave stretches SCL
            val = (val << 1) | self.sda
            yield from self._high_remaining(2 * q - self.sample_delay)
            self._scl(0)
            yield q
        self._sda(0 if ack else 1)
        yield q
        yield from self._clock_high()
        yield from self._high_remaining(2 * q - self.sample_delay)
        self._scl(0)
        yield q
        self._sda(1)
        self.reads.append(val)

    def _run(self):
        yield self.start_at
        for op in self.script:
            k = op[0]
            q = self.period // 4
            if k == "start":
                if self.join_start:
                    yield lambda: self.sda == 0 and self.scl == 1   # someone else just STARTed
                else:
                    yield from self._wait_bus_free()
                self._sda(0)                          # START: SDA falls while SCL high
                yield q
                self._scl(0)
                yield q
            elif k == "write":
                ok = yield from self._write_byte(op[1])
                if not ok:
                    return
            elif k == "read":
                yield from self._read_byte(op[1])
            elif k == "repstart":
                self._sda(1)
                yield q
                self._scl(1)
                yield lambda: self.scl == 1
                yield q
                self._sda(0)                          # repeated START: SDA falls while SCL high
                yield q
                self._scl(0)
                yield q
            elif k == "stop":
                self._sda(0)
                yield q
                self._scl(1)
                yield lambda: self.scl == 1
                yield q
                self._sda(1)
                yield q
                self.stops += 1
            else:
                raise ValueError(k)
        self._release_all()


# ----------------------------------------------------------------------------- 1-Wire
def crc8_maxim(data):
    """Dallas/Maxim 1-Wire CRC-8 (x^8 + x^5 + x^4 + 1, reflected).  crc8_maxim(rom_with_crc) == 0."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8C if crc & 1 else crc >> 1
    return crc


def make_rom(family=0x28, serial=(0x11, 0x22, 0x33, 0x44, 0x55, 0x66)):
    rom = [family] + list(serial)
    return rom + [crc8_maxim(rom)]


class OneWireSlave:
    """1-Wire device on pad 8 (open drain, wired-AND with the PIO).  Time is in clocks; `us` is how
    many clocks make one microsecond (24 at this chip's 24 MHz, which with CLKDIV 24 is a 1 us tick).

    Behaviour (Maxim standard speed):
      * a master low pulse >= `reset_min` us is a RESET: the device answers `presence_delay` us after the
        release with a `presence_len` us low pulse (unless present=False);
      * after a reset the device collects 8 bits LSB-first per command byte; a write slot is read as 0 if
        the master is still low 30 us after the falling edge, else 1;
      * a command byte listed in `responses` makes the device send those bytes (LSB first, one bit per
        read slot): for a 0 it pulls the line low from the falling edge for `read_hold` us (the spec
        guarantees >= 15), for a 1 it leaves the line alone. Beyond the list it sends 1s.
    Flags (in `violations`) anything a real device could not cope with: a low pulse < 1 us, a low pulse
    that is neither a write-1/read slot (<= 15 us), a write-0 (60..120 us) nor a reset, a slot starting
    < 60 us after the previous one, or < 1 us of recovery."""

    def __init__(self, responses=None, present=True, us=24, read_hold=15, presence_delay=30,
                 presence_len=120, reset_min=480):
        self.us = us
        self.responses = {k: list(v) for k, v in (responses or {}).items()}
        self.present = present
        self.read_hold, self.presence_delay = read_hold, presence_delay
        self.presence_len, self.reset_min = presence_len, reset_min
        self.mode = "idle"                 # idle until the first reset
        self.cmd_bits = []
        self.tx_bits = []
        self.cmds = []                     # bytes received, one list per reset
        self.sent = []                     # bits the device offered in read slots
        self.slots = []                    # (fall_cycle, low_clks, bit_or_None)
        self.resets = []                   # (fall_cycle, low_clks)
        self.violations = []
        self.ml_prev = 0
        self.t_fall = self.t_rise = None
        self.last_fall = None
        self.drive_from = self.drive_until = 0
        self.pending_presence = None
        self._now = 0

    @property
    def low(self):
        return 0x100 if self.drive_from <= self._now < self.drive_until else 0

    def _us(self, clks):
        return clks / self.us

    def _flag(self, msg):
        self.violations.append("t=%d: %s" % (self._now, msg))

    def step(self, w):
        now = w.cycle
        self._now = now
        ml = 1 if (w.mlow & 0x100) else 0
        if ml and not self.ml_prev:
            self._fall(now)
        elif self.ml_prev and not ml:
            self._rise(now)
        self.ml_prev = ml
        if self.pending_presence is not None and now >= self.pending_presence:
            self.drive_from, self.drive_until = now, now + self.presence_len * self.us
            self.pending_presence = None

    def _fall(self, now):
        if self.t_rise is not None and self._us(now - self.t_rise) < 1.0:
            self._flag("recovery %.2f us < 1 us" % self._us(now - self.t_rise))
        if self.last_fall is not None and self.mode != "idle" and self.pending_presence is None \
                and now > self.drive_until and self._us(now - self.last_fall) < 60 \
                and not (self.resets and self.resets[-1][0] == self.last_fall):
            self._flag("slot starts %.1f us after the previous one (< 60 us)" % self._us(now - self.last_fall))
        self.t_fall = self.last_fall = now
        if self.mode == "send":                       # a read slot: the device must act NOW
            bit = self.tx_bits.pop(0) if self.tx_bits else 1
            self.sent.append(bit)
            if bit == 0:
                self.drive_from, self.drive_until = now + 1, now + self.read_hold * self.us

    def _rise(self, now):
        self.t_rise = now
        low = now - self.t_fall
        lus = self._us(low)
        if lus >= self.reset_min:
            self.resets.append((self.t_fall, low))
            self.mode, self.cmd_bits, self.tx_bits = "cmd", [], []
            self.cmds.append([])
            if self.present:
                self.pending_presence = now + self.presence_delay * self.us
            return
        bit = None
        if lus < 1.0:
            self._flag("low pulse %.2f us < 1 us" % lus)
        elif 15 < lus < 60 or lus > 120:
            self._flag("low pulse %.1f us is neither a write-1/read (<=15), a write-0 (60..120) nor a reset" % lus)
        if self.mode == "cmd":
            bit = 0 if lus > 30 else 1                # the device samples 30 us after the fall
            self.cmd_bits.append(bit)
            if len(self.cmd_bits) == 8:
                byte = sum(b << i for i, b in enumerate(self.cmd_bits))
                self.cmds[-1].append(byte)
                self.cmd_bits = []
                if byte in self.responses:
                    self.mode = "send"
                    self.tx_bits = [(v >> i) & 1 for v in self.responses[byte] for i in range(8)]
        elif self.mode == "send" and lus > 15:
            self._flag("read slot low pulse %.1f us > 15 us" % lus)
        self.slots.append((self.t_fall, low, bit))
