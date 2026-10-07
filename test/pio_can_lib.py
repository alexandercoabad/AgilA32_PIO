"""pio_can_lib.py -- CAN bus model and host driver for the CAN PIO tests (test_pio_can.py).

CanBus   wired-AND bus: TXD of the chip (uo_out[0], only while the PIO owns the pin) AND the outputs of the node models.
         RXD (ui_in[1]) is the bus level seen through a transceiver loop delay (`delay` clock cycles).
         Fault injection: glitch(c0, c1) pulls the wire dominant, rxd_stuck(c0, c1, level) overrides what RXD sees.
CanNode  a CAN controller model: hard synchronisation at SOF, sampling at 75 % of the bit, destuffing, CRC check,
         ACK, arbitration (it stops driving when it sends recessive and reads dominant), error resynchronisation.
         It transmits frames on request (`send`) either at a given cycle or by joining a start of frame it sees.
CanHost  what the firmware does: set up the two state machines, queue TX commands, drain RX captures.
"""
import os
import sys
from collections import deque

from cocotb.triggers import ClockCycles

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
import can_frame as F                                       # noqa: E402
from pio_tb_lib import assemble, R_PIN_OWN, R_SYNC_BYP, R_CTRL   # noqa: E402

PIO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "pio") + os.sep
TX_SM, RX_SM = 0, 1


class CanNode:
    def __init__(self, bus, name="N", ack=True, join_delay=2, bit_scale=1.0):
        self.bus, self.name, self.ack, self.join_delay = bus, name, ack, join_delay
        self.bit_scale = bit_scale        # node bit time = bit_scale * the chip's (oscillator error)
        self.out = 1
        self.state = "IDLE"
        self.pending = None            # (bits, how, arg)
        self.tx_bits = None
        self.t0 = 0
        self.raw = []
        self.ack_idx = None
        self.crc_ok = False
        self.frames = []               # parsed frames received (parse_frame dicts)
        self.sent = []                 # frames this node finished transmitting
        self.lost_at = None            # raw index at which this node lost arbitration
        self.busy_until = 0
        self.recessive_run = 0
        self.errors = 0
        self.was_tx = False
        self.noparse = False

    # ------------------------------------------------------------- requests
    def send(self, ident, data=b"", rtr=False, ext=False, at=None, dlc=None):
        """Queue a frame.  at=None: start together with the next start of frame on the bus (like a node that was
        waiting for the bus to become idle), at=cycle: start at that cycle."""
        bits = F.stuff(F.frame_bits(ident, data, rtr, ext, dlc)) + [1]
        self.pending = (bits, at)

    def send_raw(self, bits, at):
        """Send an arbitrary bit sequence (no frame parsing, no ACK) starting at cycle `at`."""
        self.pending = (list(bits), at)
        self.noparse = True

    # ------------------------------------------------------------- per cycle
    def step(self, c, wire, prev_wire):
        B = self.bus.B * self.bit_scale
        S = self.bus.sample_cycle
        st = self.state
        if st == "IDLE":
            self.out = 1
            if c < self.busy_until:
                return
            if self.pending is not None and self.pending[1] is not None and c >= self.pending[1]:
                self._begin(c, tx=True)
            elif wire == 0 and prev_wire == 1:                       # SOF seen
                if self.pending is not None and self.pending[1] is None:
                    self._begin(c + self.join_delay, tx=True)
                else:
                    self._begin(c, tx=False)
            return
        if st == "RESYNC":
            self.out = 1
            if int((c - self.t0) % B) == S:
                self.recessive_run = self.recessive_run + 1 if wire else 0
                if self.recessive_run >= 11:
                    self.state = "IDLE"
                    self.busy_until = c
            return
        # ---- FRAME (transmitting or receiving)
        if c < self.t0:
            self.out = 1
            return
        i, p = divmod(c - self.t0, B)
        i = int(i)
        if self.noparse and self.tx_bits is not None and i >= len(self.tx_bits):
            self.noparse, self.tx_bits, self.state, self.out = False, None, "IDLE", 1
            self.busy_until = c
            return
        if self.tx_bits is not None and i < len(self.tx_bits):
            self.out = self.tx_bits[i]
        elif self.ack_idx is not None and i == self.ack_idx and self.ack and self.crc_ok and self.tx_bits is None:
            self.out = 0
        else:
            self.out = 1
        if p >= S and i == len(self.raw):
            self.raw.append(wire)
            if self.tx_bits is not None and i < len(self.tx_bits):
                if self.tx_bits[i] == 1 and wire == 0:               # lost arbitration: carry on as a receiver
                    self.lost_at = i
                    self.tx_bits = None
                elif self.tx_bits[i] == 0 and wire == 1:
                    self.errors += 1
            self._after_sample(c)

    def _begin(self, t0, tx):
        self.state = "FRAME"
        self.t0 = t0
        self.raw, self.ack_idx, self.crc_ok, self.lost_at = [], None, False, None
        self.was_tx = tx
        if tx:
            self.tx_bits = self.pending[0]
            self.pending = None
        else:
            self.tx_bits = None

    def _after_sample(self, c):
        B = self.bus.B
        raw = self.raw
        if self.noparse:
            return
        if self.ack_idx is None:
            r = F.parse_frame(raw, through_crc=True)
            if r["stuff_error"]:
                self.state, self.recessive_run, self.errors = "RESYNC", 0, self.errors + 1
                return
            if r["error"] is None:                                   # SOF..CRC complete, delimiter is next
                self.ack_idx = len(raw) + 1
                self.crc_ok = r["crc_ok"]
                if not self.crc_ok:
                    self.state, self.recessive_run = "RESYNC", 0
            return
        if len(raw) == self.ack_idx + 1 + 8:                          # ACK slot, ACK delimiter, EOF
            r = F.parse_frame(raw)
            self.frames.append(r)
            if self.was_tx and self.lost_at is None:
                self.sent.append(r)
            self.state = "IDLE"
            self.busy_until = self.t0 + (len(raw) + 3) * B - B / 2
            self.tx_bits = None


class CanBus:
    def __init__(self, bit_cycles=16, delay=3, sample_frac=0.75):
        self.B = bit_cycles
        self.sample_cycle = int(bit_cycles * sample_frac)
        self.delay = delay
        self.nodes = []
        self.hist = deque([1] * (delay + 1), maxlen=delay + 1)
        self.wire = 1
        self.cycle = 0
        self.wire_trace = []
        self.txd_trace = []
        self.glitches = []
        self.rxd_faults = []
        self.contention = 0
        self.fault = None              # armed bit fault: [kind, bit index, SOF cycle or None]

    def set_delay(self, d):
        self.delay = d
        self.hist = deque(list(self.hist)[-1:] * (d + 1), maxlen=d + 1)

    def node(self, **kw):
        n = CanNode(self, **kw)
        self.nodes.append(n)
        return n

    def glitch(self, c0, c1):
        self.glitches.append((c0, c1))

    def rxd_stuck(self, c0, c1, level):
        self.rxd_faults.append((c0, c1, level))

    def arm_bit_fault(self, kind, k):
        """kind 'glitch': the wire is pulled dominant around the sample point of bit k of the next frame the chip
        sends (counted from its SOF edge); kind 'rxd1': RXD reads recessive there although the wire is dominant."""
        assert kind in ("glitch", "rxd1")
        self.fault = [kind, k, None]

    def step(self, w):
        c = self.cycle
        txd = (w.pin_out & 1) if (w.own & 1) else 1
        if self.fault is not None and self.fault[2] is None and txd == 0 and (not self.txd_trace or self.txd_trace[-1] == 1):
            self.fault[2] = c
            kind, k, t0 = self.fault
            a, b = t0 + k * self.B + 1, t0 + (k + 1) * self.B - 1
            if kind == "glitch":
                self.glitches.append((a, b))
            else:
                self.rxd_faults.append((a, b + 4, 1))
        prev = self.wire_trace[-2] if len(self.wire_trace) > 1 else 1
        for n in self.nodes:
            n.step(c, self.wire, prev)
        wire = txd
        for n in self.nodes:
            wire &= n.out
        for (a, b) in self.glitches:
            if a <= c < b:
                wire = 0
        self.wire = wire
        self.wire_trace.append(wire)
        self.txd_trace.append(txd)
        self.hist.append(wire)
        rxd = self.hist[0]
        for (a, b, lv) in self.rxd_faults:
            if a <= c < b:
                rxd = lv
        w.ext_in = (w.ext_in & ~2) | (rxd << 1)
        self.cycle += 1

    # ---- waveform helpers
    def edges(self, trace=None):
        t = self.wire_trace if trace is None else trace
        return [i for i in range(1, len(t)) if t[i] != t[i - 1]]


class CanHost:
    def __init__(self, dut, bus, world, div=(1, 0)):
        self.dut, self.bus, self.world, self.div = dut, bus, world, div
        self.tx = assemble(open(PIO_DIR + "can_tx.pio").read(), origin=0)
        self.rx = assemble(open(PIO_DIR + "can_rx.pio").read(), origin=len(self.tx.instrs))
        self.rx_words = []
        self.tx_results = []
        self.captures = []

    async def setup(self, bypass=False, rx=True, tx=True):
        b = self.bus
        await b.load_program(self.tx)
        await b.load_program(self.rx)
        await b.sm_setup(TX_SM, self.tx, out_base=0, out_count=1, set_base=0, set_count=1, in_base=1, jmp_pin=1,
                         div=self.div)
        await b.sm_setup(RX_SM, self.rx, in_base=1, jmp_pin=1, autopush=True, push_thresh=0, div=self.div,
                         entry=self.rx.wrap_top)
        await b.force(TX_SM, F.SET_PINS_1)                   # TXD recessive before the PIO owns the pin
        await ClockCycles(self.dut.clk, 3)
        await b.write(R_PIN_OWN, 0x001)
        if bypass:
            await b.write(R_SYNC_BYP, 1 << 1)
        if tx:
            await b.set_enable(TX_SM)
        if rx:
            await b.set_enable(RX_SM)

    async def restart_tx(self):
        """After an abort: disable, clear both FIFOs, `jmp cmd`, enable (RX FIFO of the TX state machine only)."""
        b = self.bus
        await b.set_enable(TX_SM, False)
        await b.fifo_clear(TX_SM)
        await b.force(TX_SM, self.tx.wrap_bottom)            # jmp cmd  (jmp 0 = 0x0000 | addr)
        await b.set_enable(TX_SM, True)

    async def service(self, tx_words=(), until=None, limit=200000):
        """Feed the TX FIFO, drain both RX FIFOs, until `until()` is true (checked every loop)."""
        b = self.bus
        pend = list(tx_words)
        for _ in range(limit):
            if pend and await b.tx_level(TX_SM) < 4:
                await b.tx_put(TX_SM, pend.pop(0))
            if await b.rx_level(TX_SM):
                self.tx_results.append(await b.rx_get(TX_SM))
            if await b.rx_level(RX_SM):
                self.rx_words.append(await b.rx_get(RX_SM))
            if not pend and until is not None and until():
                return
            if not pend and until is None:
                return
        raise TimeoutError("CAN host service loop timed out (pending TX words %d)" % len(pend))

    def take_captures(self):
        caps, rest = F.split_captures(self.rx_words)
        self.rx_words = rest
        self.captures += caps
        return caps
