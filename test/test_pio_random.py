"""Constrained-random protocol tests for the AgilA32 PIO block (src/pio.v).

Same bench as test_pio_protocols.py (pio.v is the toplevel, cocotb plays the CPU and the
peer devices), but every parameter is drawn at random inside the range the program is
specified for, and the result is checked against a reference model of the protocol:

    UART TX   clock divider, frame count, byte values          -> exact 8N1 waveform, bit-exact timing
    UART RX   baud error (+-1.5 %), idle gaps, byte values,
              injected break frames                            -> bytes received, framing IRQ, recovery
    SPI       mode 0..3, divider, SYNC_BYP, byte count/values  -> MOSI bytes at the slave, MISO bytes in RX FIFO
    I2C       7-bit address, write / read / register-read,
              clock divider, slave clock stretching, wrong
              address                                          -> ACK/NAK, data, START/STOP count, no SDA-while-SCL-high
    PS/2 RX   CLOCK half period, inter-frame gap, byte values   -> one RX word per frame, parity ok

Run:   cd test && make -f Makefile.random
Seeds: PIO_RANDOM_SEED=<n>  (default 1)   PIO_RANDOM_N=<iterations per test> (default 12)
Every iteration derives its own generator from (seed, test, iteration) and logs its parameters,
so a failure is reproduced by re-running with the same seed:  PIO_RANDOM_SEED=<n> make -f Makefile.random
"""
import os
import random
from collections import Counter

import cocotb
from cocotb.triggers import ClockCycles

from pio_tb_lib import (assemble, SerialSource, SpiSlave, I2cSlave, Ps2Keyboard, decode_ps2_word, decode_uart,
                        R_IRQ, R_PIN_OWN, R_SYNC_BYP, FIFO_DEPTH)
import pio_i2c                       # noqa: E402  (tools/ is put on sys.path by pio_tb_lib)
from test_pio_protocols import (setup, load_src, uart_tx_run, collect_rx, spi_run, i2c_setup, i2c_run,
                                ps2_setup, collect_ps2, settle, check_mosi_timing, scl_period_and_high)

SEED = int(os.environ.get("PIO_RANDOM_SEED") or "1")
N = int(os.environ.get("PIO_RANDOM_N") or "12")


def rng_for(dut, name, it):
    r = random.Random("%d/%s/%d" % (SEED, name, it))
    dut._log.info("random test %s iteration %d (PIO_RANDOM_SEED=%d)" % (name, it, SEED))
    return r


def rand_bytes(r, lo, hi):
    n = r.randint(lo, hi)
    specials = [0x00, 0xFF, 0x55, 0xAA, 0x01, 0x80]
    return [r.choice(specials) if r.random() < 0.25 else r.randrange(256) for _ in range(n)]


# ============================================================================= UART TX
async def uart_tx_run_long(dut, div, bit_clk, data):
    """Like test_pio_protocols.uart_tx_run, but drains FIFO_DEPTH + 3 frames after the last put (the last
    frame needs its stop bit and one idle bit on the trace before the decoder can accept it)."""
    bus, world = await setup(dut)
    prog = load_src("uart_tx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=0, out_count=1, set_base=0, set_count=1, side_base=0,
                       out_right=True, div=div)
    await bus.force(0, assemble("set pins, 1").instrs[0])
    await bus.write(R_PIN_OWN, 0x1)
    await bus.set_enable(0)
    for b in data:
        await bus.tx_put_blocking(0, b)
    await ClockCycles(dut.clk, int(bit_clk * 10 * (FIFO_DEPTH + 3)) + 40)
    tr = world.bit_trace(0)
    return decode_uart(tr, bit_clk, nframes=len(data)), tr


@cocotb.test()
@cocotb.parametrize(it=range(N))
async def test_random_uart_tx(dut, it):
    """Random integer CLKDIV 1..8 (8 clocks per bit each) and 1..8 random bytes: the pad must show
    exact 8N1 frames, every bit stable for exactly 8 * CLKDIV clocks."""
    r = rng_for(dut, "uart_tx", it)
    div = r.randint(1, 8)
    data = rand_bytes(r, 1, 8)
    dut._log.info("div=%d data=%s" % (div, [hex(b) for b in data]))
    (got, problems), _ = await uart_tx_run_long(dut, (div, 0), 8 * div, data)
    assert not problems, problems
    assert got == data, [hex(g) for g in got]


# ============================================================================= UART RX
async def uart_rx_setup_div(dut, div):
    bus, world = await setup(dut)
    prog = load_src("uart_rx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, in_base=0, jmp_pin=0, in_right=True, div=(div, 0))
    src = SerialSource(0)
    world.devs.append(src)
    world.ext_in |= 1
    await ClockCycles(dut.clk, 4)
    await bus.set_enable(0)
    return bus, world, src


@cocotb.test()
@cocotb.parametrize(it=range(N))
async def test_random_uart_rx(dut, it):
    """Random CLKDIV 1..6, sender baud error uniform in +-1.5 % (inside the +-2 % the receiver is
    specified for), random idle gaps, random bytes; sometimes a break frame in the middle.  Every
    good byte must arrive in order, a break must set IRQ flag 4 and push nothing, and the
    receiver must recover on the next frame."""
    r = rng_for(dut, "uart_rx", it)
    div = r.randint(1, 6)
    err = r.uniform(-0.015, 0.015)
    bit_clk = 8.0 * div * (1 + err)
    data = rand_bytes(r, 2, 7)
    brk_at = r.randrange(len(data)) if r.random() < 0.4 else None
    dut._log.info("div=%d err=%+.2f%% brk_at=%s data=%s" % (div, err * 100, brk_at, [hex(b) for b in data]))
    bus, world, src = await uart_rx_setup_div(dut, div)
    expect = []
    for i, b in enumerate(data):
        if i == brk_at:
            src.add_uart_frame(0x00, bit_clk, stop=0, gap=0)       # break
            src.q.append((0, int(round(bit_clk * 4))))
            src.q.append((1, int(round(bit_clk * 3))))
        else:
            src.add_uart_frame(b, bit_clk, gap=r.choice([0.0, 0.5, 1.0, 2.5, 4.0]) + 1.0)
            expect.append(b)
    got = await collect_rx(bus, 0, len(expect), timeout=20000 * div)
    assert got == expect, "expected %s got %s" % ([hex(b) for b in expect], [hex(b) for b in got])
    await settle(dut, src, extra=int(bit_clk * 12))       # let the whole line pattern (incl. a trailing break) play out
    irq = await bus.read(R_IRQ)
    if brk_at is not None:
        assert irq & 0x10, "break must raise IRQ flag 4"
    else:
        assert irq & 0x10 == 0, "no framing error expected"


# ============================================================================= SPI
@cocotb.test()
@cocotb.parametrize(it=range(N))
async def test_random_spi(dut, it):
    """Random SPI mode 0..3, integer CLKDIV 2..8 (or CLKDIV 1 with the MISO synchroniser bypassed),
    1..6 random bytes each way: the slave model must see the MOSI bytes MSB first and the RX FIFO
    must return the MISO bytes."""
    r = rng_for(dut, "spi", it)
    mode = r.randrange(4)
    fast = r.random() < 0.2
    div = (1, 0) if fast else (r.randint(2, 8), 0)
    n = r.randint(1, 6)
    tx = [r.randrange(256) for _ in range(n)]
    miso = [r.randrange(256) for _ in range(n)]
    dut._log.info("mode=%d div=%s fast_bypass=%s tx=%s miso=%s" % (
        mode, div, fast, [hex(b) for b in tx], [hex(b) for b in miso]))
    slave, got = await spi_run(dut, mode, tx, miso, div, bypass_miso=fast)
    assert slave.mosi_bytes == tx, "MOSI: %s" % [hex(b) for b in slave.mosi_bytes]
    assert got == miso, "MISO: %s" % [hex(b) for b in got]
    # timing: SCK period is exactly 4 * CLKDIV clocks, MOSI is stable 2 * CLKDIV clocks before and after each sampling edge
    d = [b - a for a, b in zip(slave.rise_cycles, slave.rise_cycles[1:])]
    assert d and set(d[:7]) == {4 * div[0]}, "SCK period should be %d clocks, got %s" % (4 * div[0], d[:8])
    check_mosi_timing(slave, min_clocks=2 * div[0])


# ============================================================================= I2C master
@cocotb.test()
@cocotb.parametrize(it=range(N))
async def test_random_i2c(dut, it):
    """Random 7-bit address, CLKDIV 1..4, optional slave clock stretching, and one of three
    transactions: write 1..6 bytes, read 1..5 bytes, or register read (write reg, repeated START,
    read 1..4).  Checks ACK/data at the slave, the master's received bytes, START/STOP counts,
    and that SDA never changes while SCL is high outside START/STOP."""
    r = rng_for(dut, "i2c", it)
    addr = r.randint(0x08, 0x77)
    div = r.randint(1, 4)
    stretch = r.choice([0, 0, 60, 200])
    kind = r.choice(["write", "read", "regread"])
    wr = rand_bytes(r, 1, 6)
    nrd = r.randint(1, 5)
    rd = [r.randrange(256) for _ in range(nrd)]
    dut._log.info("kind=%s addr=0x%02x div=%d stretch=%d write=%s read=%s" % (
        kind, addr, div, stretch, [hex(b) for b in wr], [hex(b) for b in rd]))
    slave = I2cSlave(addr=addr, read_bytes=rd, stretch=stretch)
    bus, world = await i2c_setup(dut, slave, div=(div, 0))
    limit = 60000 * div + 40000 * (1 if stretch else 0)
    if kind == "write":
        await i2c_run(dut, bus, pio_i2c.write_transaction(addr, wr), slave, 1, len(wr) + 1, limit=limit)
        assert slave.rx == [("A", addr << 1)] + [("D", b) for b in wr], slave.rx
        assert slave.starts == 1 and slave.stops == 1, (slave.starts, slave.stops)
    elif kind == "read":
        got = await i2c_run(dut, bus, pio_i2c.read_transaction(addr, nrd), slave, 1, nrd + 1, limit=limit)
        assert got[-nrd:] == rd, "read %s expected %s" % ([hex(g) for g in got], [hex(b) for b in rd])
        assert slave.rx[0] == ("A", (addr << 1) | 1), slave.rx
        assert slave.starts == 1 and slave.stops == 1, (slave.starts, slave.stops)
    else:
        reg = r.randrange(256)
        words = pio_i2c.start() + pio_i2c.write_byte(addr << 1) + pio_i2c.write_byte(reg) + pio_i2c.repeated_start()
        words += pio_i2c.write_byte((addr << 1) | 1)
        for i in range(nrd):
            words += pio_i2c.read_byte(i == nrd - 1)
        words += pio_i2c.stop()
        got = await i2c_run(dut, bus, words, slave, 1, nrd + 1, limit=limit)
        assert got[-nrd:] == rd, "read %s expected %s" % ([hex(g) for g in got], [hex(b) for b in rd])
        assert slave.rx[:3] == [("A", addr << 1), ("D", reg), ("A", (addr << 1) | 1)], slave.rx
        assert slave.starts == 2 and slave.stops == 1, (slave.starts, slave.stops)
    assert not slave.violations, slave.violations
    assert (await bus.read(R_IRQ)) & 1 == 0, "no NAK expected"
    # timing: the usual SCL period is 32 * CLKDIV clocks (+1 at CLKDIV 1, pin synchroniser), high time 16 * CLKDIV (+1);
    # slave stretching only lengthens a few cycles, so the most common values stay put
    gaps, highs = scl_period_and_high(slave)
    if len(gaps) >= 6:
        gap = Counter(gaps).most_common(1)[0][0]
        high = Counter(highs).most_common(1)[0][0]
        assert 32 * div <= gap <= 32 * div + 1, "SCL period %d clocks, expected %d" % (gap, 32 * div)
        assert 16 * div <= high <= 16 * div + 1, "SCL high time %d clocks, expected %d" % (high, 16 * div)


@cocotb.test()
@cocotb.parametrize(it=range(max(4, N // 3)))
async def test_random_i2c_wrong_address_naks(dut, it):
    """The master addresses a device that is not there: nobody may receive data, and IRQ 0 (NAK)
    must be raised for the host."""
    r = rng_for(dut, "i2c_nak", it)
    present = r.randint(0x08, 0x77)
    target = r.choice([a for a in range(0x08, 0x78) if a != present])
    data = rand_bytes(r, 1, 3)
    dut._log.info("device 0x%02x present, master calls 0x%02x" % (present, target))
    slave = I2cSlave(addr=present)
    bus, world = await i2c_setup(dut, slave)
    await i2c_run(dut, bus, pio_i2c.write_transaction(target, data), slave, 1, 1, limit=6000)
    assert not any(k == "D" for k, _ in slave.rx), slave.rx
    assert (await bus.read(R_IRQ)) & 1, "NAK must raise IRQ flag 0"


# ============================================================================= PS/2 receiver
@cocotb.test()
@cocotb.parametrize(it=range(N))
async def test_random_ps2_rx(dut, it):
    """Random CLOCK half period 40..250 clocks (the program drops a frame when CLOCK stays high longer than
    its 288-clock idle timeout, so the half period must stay below that), random inter-frame gap above the timeout, 1..8 random bytes: one RX word per frame, odd parity ok, data in order."""
    r = rng_for(dut, "ps2", it)
    half = r.randint(40, 250)
    data = rand_bytes(r, 1, 8)
    dut._log.info("half=%d data=%s" % (half, [hex(b) for b in data]))
    bus, world, kb = await ps2_setup(dut)
    for b in data:
        kb.add_frame(b, half=half, gap=r.randint(400, 1500))
    words = await collect_ps2(bus, len(data), timeout=400000)
    got = [decode_ps2_word(w) for w in words]
    assert [d for d, _ in got] == data, [hex(d) for d, _ in got]
    assert all(ok for _, ok in got), got
    assert await bus.rx_level(0) == 0
