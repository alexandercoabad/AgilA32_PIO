"""CAN on the PIO: pio/can_tx.pio and pio/can_rx.pio against a CAN bus model (test/pio_can_lib.py).

    cd test && make -f Makefile.proto          (module test_pio_can)

The chip side is TXD = uo_out[0] and RXD = ui_in[1] of a CAN transceiver; the bus model (pio_can_lib.CanBus) is a
wired-AND of TXD and CAN controller models (CanNode: hard sync, 75 % sampling, destuffing, CRC check, ACK,
arbitration, error resynchronisation).  The CRC-15 is also checked against an independent polynomial-division
implementation in this file.
"""
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from pio_tb_lib import PioBus, World, assemble
from pio_can_lib import CanBus, CanHost, TX_SM, RX_SM
import can_frame as F


async def setup(dut, div=(1, 0), delay=3, bypass=False, nodes=1, **node_kw):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 0
    dut.valid.value = 0
    dut.we.value = 0
    dut.addr.value = 0
    dut.wdata.value = 0
    dut.pins_raw.value = 0
    await ClockCycles(dut.clk, 4)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 2)
    bus = PioBus(dut)
    world = World(dut)
    B = int(round(16 * (div[0] + div[1] / 256.0)))
    can = CanBus(bit_cycles=B, delay=delay)
    world.devs.append(can)
    cocotb.start_soon(world.run())
    host = CanHost(dut, bus, world, div)
    await host.setup(bypass=bypass)
    return host, can, world




# ============================================================================= helpers
def crc15_reference(bits):
    """CRC-15/CAN by long division over GF(2): remainder of (bits * x^15) divided by the generator polynomial."""
    poly = (1 << 15) | 0x4599                       # literal: the reference must not share the constant under test
    reg = 0
    for b in list(bits) + [0] * 15:
        reg = (reg << 1) | b
        if reg >> 15:
            reg ^= poly
    return reg


async def tx_frame(host, words, limit=6000):
    n0 = len(host.tx_results)
    await host.service(words, until=lambda: len(host.tx_results) > n0, limit=limit)
    return host.tx_results[-1]


async def idle(host, can, cycles):
    target = can.cycle + cycles
    await host.service((), until=lambda: can.cycle >= target, limit=400000)


async def captures(host, can, n, limit_cycles=60000):
    """Service until n captures (each ends with its tag word) have arrived; returns the decoded frames."""
    target = can.cycle + limit_cycles
    await host.service((), until=lambda: len(F.split_captures(host.rx_words)[0]) + len(host.captures) >= n
                       or can.cycle > target, limit=400000)
    host.take_captures()
    assert len(host.captures) >= n, "only %d captures: %r" % (len(host.captures), host.rx_words)
    return [F.decode_capture(c) for c in host.captures[:n]]


def same_frame(r, ident, data=b"", rtr=False, ext=False, dlc=None):
    return (r["error"] is None and r["crc_ok"] and r["ident"] == ident and r["data"] == bytes(data)
            and r["rtr"] == rtr and r["ext"] == ext and (dlc is None or r["dlc"] == dlc))


# ============================================================================= pure Python
@cocotb.test()
async def test_can_crc15_matches_the_reference_and_has_zero_remainder(dut):
    """crc15() equals an independent GF(2) long division on 2000 random frames, a message followed by its CRC has
    remainder 0, and one flipped bit always changes the CRC."""
    rnd = random.Random(15)
    for _ in range(2000):
        bits = [rnd.getrandbits(1) for _ in range(rnd.randint(19, 83))]
        c = F.crc15(bits)
        assert c == crc15_reference(bits), (bits, c)
        assert crc15_reference(bits + [(c >> (14 - k)) & 1 for k in range(15)]) == 0
        k = rnd.randrange(len(bits))
        flipped = list(bits)
        flipped[k] ^= 1
        assert F.crc15(flipped) != c
    assert F.crc15([]) == 0


@cocotb.test()
async def test_can_stuffing_rules(dut):
    """Stuff bits are inserted after five equal bits (they count for the next run), destuffing inverts it and flags a
    sixth equal bit, and worst-case frames never carry more than 5 equal bits in a row."""
    assert F.stuff([0] * 5) == [0, 0, 0, 0, 0, 1]
    assert F.stuff([0] * 10) == [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1]
    assert F.stuff([1, 1, 1, 1, 1, 0, 0, 0, 0]) == [1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 1]   # the stuffed 0 starts a run
    assert F.stuff([0, 1] * 8) == [0, 1] * 8
    assert F.destuff([0] * 6) == ([0] * 5, 5)
    rnd = random.Random(5)
    for _ in range(500):
        bits = [rnd.getrandbits(1) if rnd.random() < .5 else 0 for _ in range(rnd.randint(1, 140))]
        st = F.stuff(bits)
        d, err = F.destuff(st)
        assert err is None and d == bits
        run, last = 0, None
        for b in st:
            run = run + 1 if b == last else 1
            last = b
            assert run <= 5
    for data in (b"\x00" * 8, b"\xff" * 8, b"\xaa" * 8, b"\x0f\xf0" * 4):
        for ident, ext in ((0, False), (0x7FF, False), (0, True), (0x1FFFFFFF, True)):
            st = F.tx_stream(ident, data, False, ext)
            assert len(st) <= (160 if ext else 132), len(st)


@cocotb.test()
async def test_can_frame_encode_parse_roundtrip(dut):
    """parse_frame() recovers identifier, data, DLC, RTR, extended flag and CRC of 3000 random frames, reports ACK and
    EOF, and detects a flipped bit as a CRC or stuff error."""
    rnd = random.Random(21)
    for _ in range(3000):
        ext = rnd.random() < .5
        ident = rnd.getrandbits(29 if ext else 11)
        rtr = rnd.random() < .2
        n = rnd.randint(0, 8)
        data = b"" if rtr else bytes(rnd.choice([0, 255, rnd.getrandbits(8)]) for _ in range(n))
        dlc = rnd.randint(0, 15) if rtr else (n if n < 8 else rnd.choice([8, 8, 9, 15]))
        st = F.tx_stream(ident, data, rtr, ext, dlc)
        acked = rnd.random() < .7
        r = F.parse_frame(st + [0 if acked else 1] + [1] * 10)
        assert r["error"] is None and r["ident"] == ident and r["ext"] == ext and r["rtr"] == rtr, (ident, ext, rtr, r)
        assert r["data"] == data and r["dlc"] == dlc and r["ack"] == acked and r["eof_ok"] and r["crc_ok"]
        assert r["raw_len"] == len(st)
        k = rnd.randrange(len(st) - 1)
        bad = list(st)
        bad[k] ^= 1
        rb = F.parse_frame(bad + [0] + [1] * 10)
        assert rb["error"] is not None or not rb["crc_ok"] or rb["ident"] != ident or rb["data"] != data or \
            rb["dlc"] != dlc or rb["rtr"] != rtr, "flipped bit %d went unnoticed" % k


@cocotb.test()
async def test_can_programs_fit_and_layout(dut):
    """TX (20 words) and RX (10 words) fit together in the 32 word instruction memory; the wrap windows are exactly the
    programs; the RX program is entered at its last instruction."""
    from pio_can_lib import CanHost
    class D:  # no hardware needed to assemble
        pass
    h = CanHost(None, None, None)
    assert len(h.tx.instrs) == 20 and len(h.rx.instrs) == 10
    assert (h.tx.wrap_bottom, h.tx.wrap_top) == (0, 19)
    assert (h.rx.wrap_bottom, h.rx.wrap_top) == (20, 29)
    assert len(h.tx.instrs) + len(h.rx.instrs) <= 32
    assert F.MOV_ISR_NULL == assemble("mov isr, null").instrs[0]
    assert F.MOV_X_NOT_NULL == assemble("mov x, ~null").instrs[0]
    assert F.SET_PINS_1 == assemble("set pins, 1").instrs[0]


# ============================================================================= TX on the wire
def txd_edges(can):
    return can.edges(can.txd_trace)


def sof_cycle(can, start=0):
    for i in range(max(start, 1), len(can.txd_trace)):
        if can.txd_trace[i] == 0 and can.txd_trace[i - 1] == 1:
            return i
    return None


TX_FRAMES = [
    dict(ident=0x123, data=b"\x11\x22\x33"),
    dict(ident=0x000, data=b"\x00" * 8),                       # worst case stuffing
    dict(ident=0x7FF, data=b"\xff" * 8),
    dict(ident=0x555, data=b""),                               # DLC 0
    dict(ident=0x2AA, rtr=True, dlc=3),                        # remote frame
    dict(ident=0x1ABCDEF0, data=b"\xde\xad\xbe\xef\x01", ext=True),
    dict(ident=0x1FFFFFFF, data=b"\x00\xff\x00\xff\x00\xff\x00\xff", ext=True),
    dict(ident=0x0, rtr=True, ext=True),
]


async def check_tx_frames(dut, div, delay=3, frames=TX_FRAMES, tol=0):
    host, can, world = await setup(dut, div=div, delay=delay)
    B = can.B
    n1 = can.node(name="acker")
    for k, fr in enumerate(frames):
        strict = F.tx_stream(**fr)
        start = len(can.txd_trace)
        r = F.tx_result(await tx_frame(host, F.tx_command(strict)))
        assert not r["aborted"], (fr, r)
        assert r["ack"] and r["tail_ok"], (fr, r)
        t0 = sof_cycle(can, start)
        want = F.wire_bits(strict, acked=True)
        # TXD: SOF .. CRC delimiter exactly on the 16 tick grid; the tail is all recessive
        edges = [e for e in txd_edges(can) if t0 <= e < t0 + len(strict) * B + B]
        for e in edges:
            off = (e - t0) % B
            assert min(off, B - off) <= tol, "TXD edge %d off the bit grid by %d (frame %d)" % (e, off, k)
        for i, b in enumerate(strict):
            c = t0 + i * B + B // 2
            assert can.txd_trace[c] == b, "TXD bit %d wrong (frame %d)" % (i, k)
        # wire: the ACK slot is dominant, the 10 bits after it are recessive
        a = t0 + len(strict) * B
        assert can.wire_trace[a + B // 2 + 2] == 0, "no ACK on the wire"
        for j in range(1, 12):
            assert can.wire_trace[a + j * B + B // 2] == 1
        assert n1.frames and same_frame(n1.frames[-1], **fr), (fr, n1.frames[-1])
        assert n1.frames[-1]["ack"] and n1.frames[-1]["eof_ok"]
        # TXD stays recessive after the frame
        assert can.txd_trace[-1] == 1
    # the chip heard itself: every frame is in the RX capture too
    got = await captures(host, can, len(frames))
    for g, fr in zip(got, frames):
        assert same_frame(g, **fr) and g["ack"], (fr, g)


@cocotb.test()
async def test_can_tx_frames_on_the_wire(dut):
    """Eight frames (standard, extended, remote, DLC 0..8, worst-case stuffing): TXD edges are exactly on the 16 tick
    bit grid, the strict part matches the stuffed frame, the ACK slot is acknowledged by the node, the tail is
    recessive, the node decodes the frame, and the chip's own RX capture holds the same frame."""
    await check_tx_frames(dut, (1, 0))


@cocotb.test()
async def test_can_tx_unacknowledged_frame(dut):
    """Without a receiver the ACK slot reads recessive: the result says 'not acknowledged', the tail is still clean."""
    host, can, world = await setup(dut)
    strict = F.tx_stream(0x321, b"\xa5\x5a")
    r = F.tx_result(await tx_frame(host, F.tx_command(strict)))
    assert not r["aborted"] and not r["ack"] and r["tail_ok"], r
    t0 = sof_cycle(can)
    assert all(can.wire_trace[t0 + (len(strict) + j) * can.B + can.B // 2] == 1 for j in range(12))


@cocotb.test()
async def test_can_tx_error_frame_command(dut):
    """An active error flag is the same command with n = 6: six dominant bits, then 12 recessive bits."""
    host, can, world = await setup(dut)
    r = F.tx_result(await tx_frame(host, F.error_frame_command()))
    assert not r["aborted"] and not r["ack"] and r["tail_ok"], r
    t0 = sof_cycle(can)
    B = can.B
    wire = [can.wire_trace[t0 + i * B + B // 2] for i in range(18)]
    assert wire == [0] * 6 + [1] * 12, wire


@cocotb.test()
async def test_can_tx_back_to_back_commands(dut):
    """Three commands queued at once: each frame is acknowledged, and the next SOF follows the intermission of the
    previous frame with exactly 3 clocks of extra gap (the machine's own push / pull / out instructions), which also pins the length of the ACK slot + tail to 12 bits of 16 ticks."""
    host, can, world = await setup(dut)
    can.node(name="acker")
    frames = TX_FRAMES[:3]
    words, strict = [], []
    for fr in frames:
        s = F.tx_stream(**fr)
        strict.append(s)
        words += F.tx_command(s)
    await host.service(words, until=lambda: len(host.tx_results) >= 3, limit=8000)
    assert [F.tx_result(w)["ack"] for w in host.tx_results] == [True] * 3
    sofs, pos = [], 0
    for k in range(3):
        pos = sof_cycle(can, pos)
        sofs.append(pos)
        pos += (len(strict[k]) + 12) * can.B - can.B // 2
    for k in range(2):
        gap = sofs[k + 1] - sofs[k] - (len(strict[k]) + 12) * can.B
        assert gap == 3, "gap between frames %d clocks, expected 3 (push, pull, out, then the first bit)" % gap


@cocotb.test()
async def test_can_tx_bit_rates(dut):
    """Integer and fractional clock dividers: the bit time is 16 * CLKDIV clocks (TXD edges within one clock of the
    grid for the fractional divider)."""
    await check_tx_frames(dut, (2, 0), frames=TX_FRAMES[:2] + TX_FRAMES[5:6])


@cocotb.test()
async def test_can_tx_fractional_divider(dut):
    await check_tx_frames(dut, (3, 128), frames=TX_FRAMES[:1] + TX_FRAMES[4:6], tol=2)


# ============================================================================= arbitration and bit errors
def first_diff(a, b):
    for i, (x, y) in enumerate(zip(a, b)):
        if x != y:
            return i
    return None


async def drain_to_idle(host, can, bits=40):
    await idle(host, can, bits * can.B)


@cocotb.test()
async def test_can_tx_loses_arbitration(dut):
    """A node with the lower identifier starts together with the chip: the chip's readback sees dominant while it sent
    recessive, stops at exactly that bit, releases TXD, reports the bit index; the winner's frame goes through intact
    (and is captured by the chip's RX state machine); after a restart the chip sends its own frame fine."""
    host, can, world = await setup(dut)
    winner = can.node(name="winner", ack=False)
    acker = can.node(name="acker")
    mine = F.tx_stream(0x123, b"\xca\xfe")
    theirs = F.tx_stream(0x0F0, b"\x5a")
    winner.send(0x0F0, b"\x5a")
    word = await tx_frame(host, F.tx_command(mine))
    r = F.tx_result(word)
    assert r["aborted"], r
    k = first_diff(mine, theirs)
    assert 1 <= k <= 12 and mine[k] == 1 and theirs[k] == 0
    assert F.abort_index(word, len(mine)) == k, (F.abort_index(word, len(mine)), k)
    await drain_to_idle(host, can, 160)
    t0 = sof_cycle(can)
    assert all(can.txd_trace[t0 + k * can.B + 4 + j] == 1 for j in range(can.B - 6)), "TXD not released at the abort"
    assert can.txd_trace[-1] == 1 and all(v == 1 for v in can.txd_trace[t0 + (k + 1) * can.B:]), "TXD must stay recessive"
    assert winner.frames and same_frame(winner.frames[-1], 0x0F0, b"\x5a") and winner.frames[-1]["ack"]
    assert winner.sent, "the winner completed its transmission"
    got = await captures(host, can, 1)
    assert same_frame(got[0], 0x0F0, b"\x5a") and got[0]["ack"]
    # restart: the next frame goes out normally and is acknowledged
    await host.restart_tx()
    n_before = len(host.tx_results)
    r2 = F.tx_result(await tx_frame(host, F.tx_command(mine)))
    assert not r2["aborted"] and r2["ack"] and r2["tail_ok"], r2
    assert same_frame(acker.frames[-1], 0x123, b"\xca\xfe")


@cocotb.test()
async def test_can_tx_wins_arbitration(dut):
    """The chip has the lower identifier: the other node loses at the first differing bit, becomes a receiver and
    acknowledges the chip's frame; the chip's frame is complete and acknowledged."""
    host, can, world = await setup(dut)
    loser = can.node(name="loser")
    mine = F.tx_stream(0x0F0, b"\x01\x02\x03")
    theirs = F.tx_stream(0x123, b"\xca\xfe")
    loser.send(0x123, b"\xca\xfe")
    r = F.tx_result(await tx_frame(host, F.tx_command(mine)))
    assert not r["aborted"] and r["ack"] and r["tail_ok"], r
    assert loser.lost_at == first_diff(mine, theirs), loser.lost_at
    assert loser.frames and same_frame(loser.frames[-1], 0x0F0, b"\x01\x02\x03") and loser.frames[-1]["ack"]
    assert not loser.sent


@cocotb.test()
async def test_can_tx_arbitration_at_every_identifier_bit(dut):
    """The abort position tracks the first differing identifier bit: ten pairs of identifiers that differ first at
    bit 0..10 of the identifier."""
    host, can, world = await setup(dut)
    winner = can.node(name="winner", ack=False)
    can.node(name="acker")
    for b in range(11):
        hi = 0x400 >> 0            # our identifier has a 1 where the winner has a 0
        ours = (0x7FF >> b << b) & 0x7FF            # 1 in bit (10-b), zeros after
        ours = (0x555 & ~((1 << (10 - b)) - 1)) | (1 << (10 - b))
        theirs = ours & ~(1 << (10 - b))
        a, c = F.tx_stream(ours, b"\x11"), F.tx_stream(theirs, b"\x22")
        winner.send(theirs, b"\x22")
        w = await tx_frame(host, F.tx_command(a))
        assert F.tx_result(w)["aborted"]
        assert F.abort_index(w, len(a)) == first_diff(a, c), (b, ours, theirs)
        await drain_to_idle(host, can, 160)
        assert winner.frames and same_frame(winner.frames[-1], theirs, b"\x22")
        await host.restart_tx()


async def bit_fault_case(dut, kind, want_bit):
    host, can, world = await setup(dut)
    can.node(name="acker")
    strict = F.tx_stream(0x123, b"\xff\x00\xa5")
    k = next(i for i in range(24, len(strict) - 2) if strict[i] == want_bit)
    can.arm_bit_fault(kind, k)
    w = await tx_frame(host, F.tx_command(strict))
    assert F.tx_result(w)["aborted"], F.tx_result(w)
    assert F.abort_index(w, len(strict)) == k, (F.abort_index(w, len(strict)), k)
    t0 = sof_cycle(can)
    await drain_to_idle(host, can, 60)
    assert all(v == 1 for v in can.txd_trace[t0 + k * can.B + 2 + (0 if want_bit == 1 else 12):]), "TXD must be released"
    await host.restart_tx()
    r = F.tx_result(await tx_frame(host, F.tx_command(strict)))
    assert not r["aborted"] and r["ack"], "recovery after a bit error"


@cocotb.test()
async def test_can_tx_bit_error_recessive_sent_dominant_read(dut):
    """A glitch pulls the bus dominant while the chip sends a recessive data bit: abort at that bit, TXD released,
    restart works."""
    await bit_fault_case(dut, "glitch", 1)


@cocotb.test()
async def test_can_tx_bit_error_dominant_sent_recessive_read(dut):
    """RXD reads recessive while the chip sends a dominant data bit (transceiver / wiring fault): abort at that bit."""
    await bit_fault_case(dut, "rxd1", 0)


@cocotb.test()
async def test_can_tx_readback_margin(dut):
    """The readback is taken at tick 14 of 16 (12 ticks after the TXD edge): loop delays of 0..7 clocks (TXD -> transceiver -> RXD, before the 2 clock
    synchroniser) work, including the ACK from a node; delays of 10 and 14 clocks read the SOF back as recessive and
    the frame aborts at bit 0."""
    host, can, world = await setup(dut)
    can.node(name="acker")
    strict = F.tx_stream(0x2F1, b"\x81")
    for d in range(0, 8):
        can.set_delay(d)
        r = F.tx_result(await tx_frame(host, F.tx_command(strict)))
        assert not r["aborted"] and r["ack"], (d, r)
    for d in (10, 14):
        can.set_delay(d)
        w = await tx_frame(host, F.tx_command(strict))
        assert F.tx_result(w)["aborted"] and F.abort_index(w, len(strict)) == 0, d
        await host.restart_tx()
        await idle(host, can, 600)


@cocotb.test()
async def test_can_works_with_the_synchroniser_bypassed(dut):
    """SYNC_BYP on RXD (the pad goes straight in): same frames, same results."""
    host, can, world = await setup(dut, bypass=True)
    n1 = can.node(name="acker")
    for fr in TX_FRAMES[:4]:
        r = F.tx_result(await tx_frame(host, F.tx_command(F.tx_stream(**fr))))
        assert not r["aborted"] and r["ack"] and r["tail_ok"], (fr, r)
        assert same_frame(n1.frames[-1], **fr)
    got = await captures(host, can, 4)
    assert all(same_frame(g, **fr) for g, fr in zip(got, TX_FRAMES[:4]))


# ============================================================================= RX capture
async def node_sends(host, can, node, bits_or_frame, at_delta=200, **kw):
    at = can.cycle + at_delta
    if isinstance(bits_or_frame, list):
        node.send_raw(bits_or_frame, at)
    else:
        node.send(at=at, **bits_or_frame)
    return at


@cocotb.test()
async def test_can_rx_captures_frames(dut):
    """The RX state machine alone (TX idle): frames sent by a node are captured, decoded and acknowledged by a
    second node.  An acknowledged frame is exactly strict + 12 samples long (the capture ends with the 11 recessive
    bits after the ACK slot)."""
    host, can, world = await setup(dut, tx=False) if False else await setup(dut)
    sender = can.node(name="sender")
    can.node(name="acker")
    for fr in TX_FRAMES:
        n0 = len(host.captures)
        await node_sends(host, can, sender, fr)
        got = await captures(host, can, n0 + 1)
        g = got[n0]
        assert same_frame(g, **fr) and g["ack"] and g["eof_ok"], (fr, g)
        assert len(g["samples"]) == len(F.tx_stream(**fr)) + 12, (len(g["samples"]), len(F.tx_stream(**fr)))
        # word for word the same as the reference model of the RX program
        ref = F.capture_words(F.wire_bits(F.tx_stream(**fr), acked=True) + [1] * 20)
        assert host.captures[n0] == ref, ([hex(w) for w in host.captures[n0]], [hex(w) for w in ref])


@cocotb.test()
async def test_can_rx_unacknowledged_frame(dut):
    """No node acknowledges: the capture still decodes and says ack = False."""
    host, can, world = await setup(dut)
    sender = can.node(name="sender", ack=False)
    for fr in TX_FRAMES[:3]:
        n0 = len(host.captures)
        await node_sends(host, can, sender, fr)
        got = await captures(host, can, n0 + 1)
        g = got[n0]
        assert same_frame(g, **fr) and not g["ack"], (fr, g)


@cocotb.test()
async def test_can_rx_back_to_back_frames(dut):
    """Two frames with the minimum intermission (SOF of the second right after the third intermission bit of the
    first): two captures, no frame lost, each with its own tag."""
    host, can, world = await setup(dut)
    a = can.node(name="A")
    b = can.node(name="B")
    for fa, fb in ((TX_FRAMES[0], TX_FRAMES[5]), (TX_FRAMES[1], TX_FRAMES[3]), (TX_FRAMES[6], TX_FRAMES[0])):
        n0 = len(host.captures)
        at = can.cycle + 200
        a.send(at=at, **fa)
        b.send(at=at + (len(F.tx_stream(**fa)) + 12) * can.B, **fb)
        got = await captures(host, can, n0 + 2, limit_cycles=40000)
        assert same_frame(got[n0], **fa) and got[n0]["ack"], got[n0]
        assert same_frame(got[n0 + 1], **fb) and got[n0 + 1]["ack"], got[n0 + 1]


@cocotb.test()
async def test_can_rx_every_partial_word_length(dut):
    """Raw bit streams of 20..131 samples (all residues mod 32, including a multiple of 32 where the partial word is
    empty): the capture words reproduce the samples exactly."""
    host, can, world = await setup(dut)
    node = can.node(name="raw")
    rnd = random.Random(77)
    lengths = [14, 15, 31, 32, 33, 34, 63, 64, 65, 95, 96, 97, 127, 128, 129, 131, 47, 80]
    for S in lengths:
        L = S - 12                       # SOF + payload + 11 recessive
        while True:
            payload = [rnd.getrandbits(1) for _ in range(L - 1)] + [0]
            run, ok = 0, True
            for bit in payload:
                run = run + 1 if bit else 0
                ok &= run <= 10
            if ok:
                break
        bits = [0] + payload + [1] * 11
        assert len(bits) == S
        n0 = len(host.captures)
        await node_sends(host, can, node, bits)
        await captures(host, can, n0 + 1)
        words = host.captures[n0]
        assert F.capture_bits(words) == bits, (S, [hex(w) for w in words])
        assert len(words) == S // 32 + 2


@cocotb.test()
async def test_can_rx_error_flag_and_stuff_violation(dut):
    """Bus errors are captured like everything else: an error flag + delimiter (6 dominant, 11 recessive samples) and
    a frame with six equal bits in the identifier (parse_frame reports the stuff error)."""
    host, can, world = await setup(dut)
    node = can.node(name="raw")
    await node_sends(host, can, node, [0] * 6 + [1] * 12)
    g = await captures(host, can, 1)
    assert g[0]["samples"] == [0] * 6 + [1] * 11
    assert g[0]["stuff_error"]
    bad = F.frame_bits(0x000, b"\x01")                 # identifier 0: 11 dominant bits without stuffing
    n0 = len(host.captures)
    await node_sends(host, can, node, bad + [1] * 12)
    g = await captures(host, can, n0 + 1)
    assert g[n0]["stuff_error"] and g[n0]["error"].startswith("stuff")
    # and a normal frame after both
    sender = can.node(name="sender")
    can.node(name="acker")
    fr = TX_FRAMES[0]
    n0 = len(host.captures)
    await node_sends(host, can, sender, fr)
    g = await captures(host, can, n0 + 1)
    assert same_frame(g[n0], **fr)


@cocotb.test()
async def test_can_rx_slow_host_still_gets_whole_frames(dut):
    """The longest frame (extended, 8 data bytes, worst-case stuffing, 7 words) is drained while it is captured
    because the host polls the RX FIFO between its other work; at the other end a host that polls rarely (one poll
    per 6 bit times at 1 Mbit/s) still keeps up with a bit time of 32 clocks."""
    host, can, world = await setup(dut, div=(2, 0))
    sender = can.node(name="sender")
    can.node(name="acker")
    fr = TX_FRAMES[6]
    n0 = len(host.captures)
    await node_sends(host, can, sender, fr)
    g = await captures(host, can, n0 + 1, limit_cycles=60000)
    assert same_frame(g[n0], **fr) and g[n0]["ack"] and len(host.captures[n0]) >= 6


@cocotb.test()
async def test_can_long_mixed_session(dut):
    """Fourteen random frames, alternately sent by the chip (acknowledged by the node) and by the node (captured and
    acknowledged by nobody but the chip's listener being passive: the node's peer acks), every one decoded identically
    from the chip's own RX capture, in order."""
    rnd = random.Random(2027)
    host, can, world = await setup(dut)
    peer = can.node(name="peer")
    other = can.node(name="other")
    expect = []
    for i in range(14):
        ext = rnd.random() < .4
        fr = dict(ident=rnd.getrandbits(29 if ext else 11), ext=ext)
        if rnd.random() < .15:
            fr.update(rtr=True, dlc=rnd.randint(0, 8))
        else:
            fr["data"] = bytes(rnd.choice([0, 255, rnd.getrandbits(8)]) for _ in range(rnd.randint(0, 8)))
        expect.append(fr)
        if i % 2 == 0:
            r = F.tx_result(await tx_frame(host, F.tx_frame_command(**fr)))
            assert not r["aborted"] and r["ack"] and r["tail_ok"], (i, fr, r)
        else:
            n0 = len(host.captures)
            sender = peer if i % 4 == 1 else other
            await node_sends(host, can, sender, fr)
            await captures(host, can, n0 + 1)
        await captures(host, can, i + 1)
    got = await captures(host, can, 14)
    for i, (g, fr) in enumerate(zip(got, expect)):
        assert same_frame(g, **fr) and g["ack"], (i, fr, g)


@cocotb.test()
async def test_can_rx_clock_tolerance_of_the_sampling_point(dut):
    """No resynchronisation after SOF, so the sample point (tick 12 of 16 after the first edge) drifts against a
    transmitter whose clock is off: over the longest frame a node that is FASTER may gain 4 ticks (3.5 pass, 5.5 fail)
    and a node that is SLOWER may lose 10 ticks (9.5 pass, 11.5 fail) -- measured: about 0.19 % faster / 0.47 % slower
    over 133 bits.  A sample point one tick off in either direction fails one of the two ok cases."""
    host, can, world = await setup(dut)
    fr = TX_FRAMES[6]
    strict = F.tx_stream(**fr)
    wire = strict + [1] * 14
    k_last = max(i for i in range(1, len(wire)) if wire[i] != wire[i - 1])
    results = {}
    for name, drift in (("fast_ok", 3.5), ("fast_bad", 5.5), ("slow_ok", -9.5), ("slow_bad", -11.5)):
        scale = 1.0 - drift / (16.0 * k_last)
        node = can.node(name=name, ack=False, bit_scale=scale)
        n0 = len(host.captures)
        at = can.cycle + 200
        node.send(at=at, **fr)
        await captures(host, can, n0 + 1, limit_cycles=int(60000 * scale))
        g = F.decode_capture(host.captures[n0])
        results[name] = same_frame(g, **fr) and g["samples"][:len(strict)] == strict
        can.nodes.remove(node)
        await idle(host, can, 60 * can.B)
    assert results == {"fast_ok": True, "fast_bad": False, "slow_ok": True, "slow_bad": False}, results
