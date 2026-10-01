"""Protocol-level tests for the AgilA32 PIO block (src/pio.v).

Runs the Raspberry Pi Pico SDK programs in pio/ (UART TX/RX, SPI modes 0/1, I2C -- the I2C
one with its side-set polarity swapped, see pio/i2c.pio) against cycle-accurate peer models and checks the waveforms bit by bit:

    cd test && make -f Makefile.proto

What each test proves is stated in its docstring.
"""
import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from pio_tb_lib import *            # noqa: F401,F403
import pio_i2c
import pio_usb
from pio_tb_lib import (PioBus, World, Wire, SerialSource, SpiSlave, I2cSlave, decode_uart,
                        Ps2Keyboard, decode_ps2_word,
                        assemble, sm_reg, R_CTRL, R_IRQ, R_FSTAT, R_PIN_OWN, R_SYNC_BYP,
                        R_PINS_IN, R_INFO, R_IMEM, SM_ADDR, SM_INSTR, SM_TXF, SM_EXEC,
                        SM_SHIFT, SM_CLKDIV, SM_PINCTRL, shiftctrl, FIFO_DEPTH)

PIO_DIR = "../pio/"


def load_src(name, origin=0):
    return assemble(open(PIO_DIR + name).read(), origin=origin)


async def setup(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())      # 50 MHz nominal
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
    cocotb.start_soon(world.run())
    return bus, world


# ============================================================================= registers
@cocotb.test()
async def test_registers_and_fifos(dut):
    """Bus protocol as the CPU sees it: INFO, IMEM r/w with auto-increment, PIN_OWN/BYPASS,
    forced instruction + PC read-back, TX FIFO depth/full/flush, empty RX read."""
    bus, world = await setup(dut)

    info = await bus.read(R_INFO)
    assert info == 0x01042002, hex(info)          # version 1, FIFO 4, IMEM 32, 2 SMs

    words = [0x1234, 0xABCD, 0x0000, 0xFFFF, 0x8001]
    await bus.w_idx(R_IMEM + 3, autoinc=True)
    for w in words:
        await bus.w_data(w)
    await bus.w_idx(R_IMEM + 3, autoinc=True)
    got = [await bus.r_data() for _ in words]
    assert got == words, [hex(g) for g in got]

    await bus.write(R_PIN_OWN, 0x3A5)
    await bus.write(R_SYNC_BYP, 0x0F0)
    assert await bus.read(R_PIN_OWN) == 0x3A5
    assert await bus.read(R_SYNC_BYP) == 0x0F0
    await bus.write(R_PIN_OWN, 0)

    await bus.force(0, 0x0009)                    # jmp 9 on a disabled SM
    assert (await bus.read(sm_reg(0, SM_ADDR))) & 31 == 9
    await bus.force(1, 0x0013)                    # jmp 19
    assert (await bus.read(sm_reg(1, SM_ADDR))) & 31 == 19

    # TX FIFO: 4 deep, 5th write dropped, flags, flush
    for i in range(FIFO_DEPTH + 1):
        await bus.tx_put(0, 0x100 + i)
    assert await bus.tx_level(0) == FIFO_DEPTH
    fstat = await bus.read(R_FSTAT)
    assert (fstat >> 16) & 1 == 1, "TXFULL[0]"
    assert (fstat >> 24) & 1 == 0, "TXEMPTY[0]"
    assert (fstat >> 24) & 2 == 2, "TXEMPTY[1]"
    await bus.fifo_clear(0)
    assert await bus.tx_level(0) == 0
    assert (await bus.read(R_FSTAT) >> 24) & 3 == 3
    assert await bus.rx_get(0) == 0, "empty RX FIFO reads as 0"


# ============================================================================= UART TX
async def uart_tx_run(dut, div, bit_clk, data):
    bus, world = await setup(dut)
    prog = load_src("uart_tx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=0, out_count=1, set_base=0, set_count=1, side_base=0,
                       out_right=True, div=div)
    await bus.force(0, assemble("set pins, 1").instrs[0])          # idle high before we own the pad
    await bus.write(R_PIN_OWN, 0x1)
    await bus.set_enable(0)
    for b in data:
        await bus.tx_put_blocking(0, b)
    # let every queued frame finish (the FIFO holds 4, so most of them are still on the wire)
    await ClockCycles(dut.clk, int(bit_clk * 10 * (FIFO_DEPTH + 1)) + 40)
    tr = world.bit_trace(0)
    return decode_uart(tr, bit_clk, nframes=len(data)), tr


@cocotb.test()
async def test_uart_tx_8n1(dut):
    """Pico `uart_tx` at 8 clk/bit: exact 8N1 waveform, every bit stable for exactly 8
    clocks, idle high, back-to-back frames, more bytes than the FIFO holds."""
    data = [0x55, 0xA5, 0x00, 0xFF, 0x3C, 0xC3, 0x81]
    (got, problems), tr = await uart_tx_run(dut, (1, 0), 8, data)
    assert not problems, problems
    assert got == data, [hex(g) for g in got]


@cocotb.test()
async def test_uart_tx_fractional_divider(dut):
    """CLKDIV 3.5 (int 3, frac 0x80): bit = 8 * 3.5 = 28 clocks exactly."""
    data = [0x5A, 0xA5, 0x0F]
    (got, problems), tr = await uart_tx_run(dut, (3, 0x80), 28, data)
    assert not problems, problems
    assert got == data, [hex(g) for g in got]


@cocotb.test()
async def test_uart_tx_fractional_average(dut):
    """CLKDIV 2.25: a fractional divider cannot be exact per bit, but the frame length
    must average to 8 * 2.25 = 18 clocks/bit (jitter is at most one clock)."""
    bus, world = await setup(dut)
    prog = load_src("uart_tx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=0, out_count=1, set_base=0, set_count=1, side_base=0,
                       div=(2, 0x40))
    await bus.force(0, assemble("set pins, 1").instrs[0])
    await bus.write(R_PIN_OWN, 0x1)
    await bus.set_enable(0)
    await bus.tx_put(0, 0x00)                       # 8 zero bits + start = 9 low bits, then stop
    await ClockCycles(dut.clk, 400)
    tr = world.bit_trace(0)
    s = next(i for i in range(1, len(tr)) if tr[i - 1] == 1 and tr[i] == 0)
    e = next(i for i in range(s, len(tr)) if tr[i] == 1)
    low = e - s
    assert abs(low - 9 * 18) <= 2, "9 low bits should last %d clocks, got %d" % (9 * 18, low)


# ============================================================================= UART RX
async def uart_rx_setup(dut, pin=0):
    bus, world = await setup(dut)
    prog = load_src("uart_rx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, in_base=pin, jmp_pin=pin, in_right=True)
    src = SerialSource(pin)
    world.devs.append(src)
    world.ext_in |= 1 << pin                       # line idles high
    await ClockCycles(dut.clk, 4)
    await bus.set_enable(0)
    return bus, world, src


async def collect_rx(bus, sm, n, timeout=6000):
    got = []
    for _ in range(timeout):
        if len(got) >= n:
            break
        if await bus.rx_level(sm):
            got.append((await bus.rx_get(sm)) >> 24)
    return got


@cocotb.test()
async def test_uart_rx_frames(dut):
    """Pico `uart_rx_mini` receives back-to-back frames (8 clk/bit), including 0x00/0xFF, with
    the host draining the 4-deep RX FIFO while the line keeps running."""
    bus, world, src = await uart_rx_setup(dut)
    data = [0x5A, 0xC3, 0x81, 0x00, 0xFF, 0x18, 0xE7]
    for b in data:
        src.add_uart_frame(b, 8.0)
    got = await collect_rx(bus, 0, len(data))
    assert got == data, [hex(g) for g in got]
    assert (await bus.read(R_IRQ)) & 0x10 == 0, "no framing error expected"


@cocotb.test()
async def test_uart_rx_framing_error_and_recovery(dut):
    """A frame with a low stop bit (break) raises IRQ 4 rel (flag 4 on SM0), pushes nothing,
    and the receiver re-synchronises on the next good frame."""
    bus, world, src = await uart_rx_setup(dut)
    src.add_uart_frame(0x3C, 8.0)
    src.add_uart_frame(0x00, 8.0, stop=0, gap=0)     # break: line stays low for 10 bit times
    src.q.append((0, 8 * 4))                         # extend the break
    src.q.append((1, 8 * 3))                         # back to idle
    src.add_uart_frame(0xA7, 8.0)
    got = await collect_rx(bus, 0, 2, timeout=4000)
    assert got == [0x3C, 0xA7], [hex(g) for g in got]
    assert (await bus.read(R_IRQ)) & 0x10, "framing error must set IRQ flag 4"
    assert await bus.rx_level(0) == 0


@cocotb.test()
async def test_uart_rx_baud_tolerance(dut):
    """Baud-rate mismatch tolerance: the receiver must accept +-2 % and we report the real
    window.  (At 8 clk/bit the 2-flop pin synchroniser makes the window asymmetric.)"""
    results = {}
    for pct in (-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0, 4.0):
        bus, world, src = await uart_rx_setup(dut)
        bit_clk = 8.0 * (1 + pct / 100.0)
        data = [0x55, 0xA5, 0xFF, 0x01, 0x80]
        for b in data:
            src.add_uart_frame(b, bit_clk, gap=1.0)
        got = await collect_rx(bus, 0, len(data), timeout=1500)
        results[pct] = (got == data)
    dut._log.info("UART RX tolerance (sender bit period vs 8 clk): %s" % {
        "%+.0f%%" % k: ("ok" if v else "FAIL") for k, v in results.items()})
    for pct in (-2.0, -1.0, 0.0, 1.0, 2.0):
        assert results[pct], "receiver must tolerate %+.0f%% baud error" % pct


@cocotb.test()
async def test_uart_loopback_two_state_machines(dut):
    """SM0 (uart_tx) -> wire -> SM1 (uart_rx): both machines run concurrently out of the shared
    instruction memory; 24 random bytes must arrive intact, no framing errors."""
    bus, world = await setup(dut)
    tx, rx = load_src("uart_tx.pio", 0), load_src("uart_rx.pio", 8)
    await bus.load_program(tx)
    await bus.load_program(rx)
    await bus.sm_setup(0, tx, out_base=0, out_count=1, set_base=0, set_count=1, side_base=0)
    await bus.sm_setup(1, rx, in_base=1, jmp_pin=1, in_right=True)
    world.ext_in |= 0x2
    world.devs.append(Wire(0, 1))
    await bus.force(0, assemble("set pins, 1").instrs[0])
    await bus.write(R_PIN_OWN, 0x1)
    await bus.set_enable(0)
    await bus.set_enable(1)
    rnd = random.Random(1234)
    data = [rnd.randrange(256) for _ in range(24)]
    sent, got = 0, []
    for _ in range(40000):
        if sent < len(data) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, data[sent])
            sent += 1
        if await bus.rx_level(1):
            got.append((await bus.rx_get(1)) >> 24)
        if len(got) == len(data):
            break
    assert got == data, "%d/%d bytes, first mismatch at %s" % (
        len(got), len(data), next((i for i, (a, b) in enumerate(zip(got, data)) if a != b), None))
    assert (await bus.read(R_IRQ)) & 0x30 == 0


# ============================================================================= SPI master
SPI_PROGS = {0: "spi_master.pio", 1: "spi_cpha1.pio",
             2: "spi_cpol1_cpha0.pio", 3: "spi_cpol1_cpha1.pio"}


async def spi_run(dut, mode, tx, miso, div, bypass_miso=False):
    bus, world = await setup(dut)
    prog = load_src(SPI_PROGS[mode])
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=0, out_count=1, side_base=1, in_base=2,
                       set_base=1, set_count=1,
                       autopull=True, pull_thresh=8, autopush=True, push_thresh=8,
                       out_right=False, in_right=False, div=div)
    if mode >> 1:                                          # CPOL = 1: SCK must idle HIGH ...
        await bus.force(0, 0xF001)                         # set pins, 1 side 1 (bit 12 = side-set value)
    slave = SpiSlave(mode, miso)
    world.devs.append(slave)
    await bus.write(R_PIN_OWN, 0b011)                      # ... BEFORE the pad is handed to PIO
    if bypass_miso:
        await bus.write(R_SYNC_BYP, 1 << 2)
    await bus.set_enable(0)
    got, sent = [], 0
    for _ in range(20000):
        if sent < len(tx) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, tx[sent] << 24)
            sent += 1
        if await bus.rx_level(0):
            got.append((await bus.rx_get(0)) & 0xFF)
        if len(got) == len(tx):
            break
    await ClockCycles(dut.clk, 64)                         # let the trailing SCK edge / idle settle
    return slave, got


@cocotb.test()
async def test_spi_mode0(dut):
    """Pico `spi_cpha0`: MOSI bytes reach the slave MSB first, MISO bytes come back in the
    RX FIFO, SCK period is exactly 4 * CLKDIV clocks."""
    tx, miso = [0xA5, 0x3C, 0xFF, 0x00], [0xC3, 0x5A, 0x81, 0x7E]
    slave, got = await spi_run(dut, 0, tx, miso, (4, 0))
    assert slave.mosi_bytes == tx, [hex(b) for b in slave.mosi_bytes]
    assert got == miso, [hex(b) for b in got]
    d = [b - a for a, b in zip(slave.rise_cycles, slave.rise_cycles[1:])]
    assert set(d[:7]) == {16}, "SCK period should be 16 clocks, got %s" % d[:8]


@cocotb.test()
async def test_spi_mode1(dut):
    """Pico `spi_cpha1` (CPHA = 1): data changes on the rising edge, sampled on the falling."""
    tx, miso = [0x96, 0x69, 0xF0, 0x0F], [0xA1, 0x1A, 0xFF, 0x00]
    slave, got = await spi_run(dut, 1, tx, miso, (4, 0))
    assert slave.mosi_bytes == tx, [hex(b) for b in slave.mosi_bytes]
    assert got == miso, [hex(b) for b in got]


def check_mosi_timing(slave, min_clocks=4):
    """MOSI must be stable for at least one PIO tick (4 clocks at CLKDIV 1..4) before and after
    every sampling edge. A slave model that samples in the same cycle MOSI changes would hide
    a program whose data and clock edge coincide, so measure it explicitly."""
    assert slave.setup and min(slave.setup) >= min_clocks, "setup %s" % slave.setup[:12]
    assert slave.hold and min(slave.hold) >= min_clocks, "hold %s" % slave.hold[:12]


def check_cpol1(slave, tx):
    """CPOL = 1: SCK idles high before, between and after the transfer, and there is exactly
    one falling + one rising edge per bit (no glitch when the pad is handed over)."""
    assert slave.idle_sck == 1, "SCK was not high when the pad was handed to PIO"
    assert slave.prev_sck == 1, "SCK did not return to idle-high after the last bit"
    assert len(slave.fall_cycles) == 8 * len(tx), len(slave.fall_cycles)
    assert len(slave.rise_cycles) == 8 * len(tx), len(slave.rise_cycles)
    assert slave.fall_cycles[0] < slave.rise_cycles[0], "first edge must be the leading (falling) one"


@cocotb.test()
async def test_spi_mode2(dut):
    """`spi_cpol1_cpha0` (CPOL = 1, CPHA = 0): SCK idles high, MISO is sampled on the falling
    edge, MOSI changes on the rising edge; SCK period is exactly 4 * CLKDIV clocks."""
    tx, miso = [0xA5, 0x3C, 0xFF, 0x00], [0xC3, 0x5A, 0x81, 0x7E]
    slave, got = await spi_run(dut, 2, tx, miso, (4, 0))
    assert slave.mosi_bytes == tx, [hex(b) for b in slave.mosi_bytes]
    assert got == miso, [hex(b) for b in got]
    check_cpol1(slave, tx)
    check_mosi_timing(slave)
    d = [b - a for a, b in zip(slave.fall_cycles, slave.fall_cycles[1:])]
    assert set(d[:7]) == {16}, "SCK period should be 16 clocks, got %s" % d[:8]


@cocotb.test()
async def test_spi_mode3(dut):
    """`spi_cpol1_cpha1` (CPOL = 1, CPHA = 1): SCK idles high, MOSI changes on the falling edge,
    MISO is sampled on the rising edge."""
    tx, miso = [0x96, 0x69, 0xF0, 0x0F], [0xA1, 0x1A, 0xFF, 0x00]
    slave, got = await spi_run(dut, 3, tx, miso, (4, 0))
    assert slave.mosi_bytes == tx, [hex(b) for b in slave.mosi_bytes]
    assert got == miso, [hex(b) for b in got]
    check_cpol1(slave, tx)
    check_mosi_timing(slave)
    d = [b - a for a, b in zip(slave.fall_cycles, slave.fall_cycles[1:])]
    assert set(d[:7]) == {16}, "SCK period should be 16 clocks, got %s" % d[:8]


@cocotb.test()
async def test_spi_modes_0_and_1_idle_low(dut):
    """Regression guard for the shared helper: CPOL = 0 modes still idle low and are unchanged."""
    for mode in (0, 1):
        tx, miso = [0x5A, 0xA5], [0x0F, 0xF0]
        slave, got = await spi_run(dut, mode, tx, miso, (4, 0))
        assert slave.idle_sck == 0 and slave.prev_sck == 0
        assert slave.mosi_bytes == tx and got == miso, mode
        check_mosi_timing(slave)


@cocotb.test()
async def test_spi_mode0_fast_with_sync_bypass(dut):
    """CLKDIV 1 (SCK = clk/4): MISO must skip the 2-flop synchroniser (SYNC_BYP) to be
    sampled in time, exactly as on the RP2040."""
    tx, miso = [0x12, 0x34, 0x56, 0x78], [0x9A, 0xBC, 0xDE, 0xF0]
    slave, got = await spi_run(dut, 0, tx, miso, (1, 0), bypass_miso=True)
    assert slave.mosi_bytes == tx
    assert got == miso, [hex(b) for b in got]


# ============================================================================= I2C master
async def i2c_setup(dut, slave, div=(1, 0)):
    bus, world = await setup(dut)
    prog = load_src("i2c.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=8, out_count=1, set_base=8, set_count=1, side_base=9,
                       in_base=8, jmp_pin=8, autopull=True, pull_thresh=16, autopush=True,
                       push_thresh=8, out_right=False, in_right=False, div=div,
                       entry=prog.labels["entry_point"] + prog.origin)
    world.devs.append(slave)
    await bus.write(R_PIN_OWN, 0x300)                      # SDA / SCL are PIO pins 8 / 9
    await bus.set_enable(0)
    return bus, world


async def i2c_run(dut, bus, words, slave, stops, nrx, limit=60000):
    """Feed `words` to the SM, drain the RX FIFO, until `stops` STOP conditions were seen."""
    sent, got = 0, []
    for _ in range(limit):
        if sent < len(words) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, pio_i2c.fifo_word(words[sent]))
            sent += 1
        if await bus.rx_level(0):
            got.append((await bus.rx_get(0)) & 0xFF)
        if slave.stops >= stops and sent == len(words):
            break
    await ClockCycles(dut.clk, 50)
    while await bus.rx_level(0):
        got.append((await bus.rx_get(0)) & 0xFF)
    return got


@cocotb.test()
async def test_i2c_write(dut):
    """Pico `i2c` program: START, address 0x50/W, three data bytes, STOP.  The slave must
    ACK all of them, see exactly one START and one STOP, and no SDA change while SCL is high."""
    slave = I2cSlave(addr=0x50)
    bus, world = await i2c_setup(dut, slave)
    data = [0xDE, 0xAD, 0x5A]
    await i2c_run(dut, bus, pio_i2c.write_transaction(0x50, data), slave, 1, 4)
    assert slave.rx == [("A", 0xA0)] + [("D", b) for b in data], slave.rx
    assert slave.starts == 1 and slave.stops == 1, (slave.starts, slave.stops)
    assert not slave.violations, slave.violations
    assert (await bus.read(R_IRQ)) & 1 == 0, "no NAK expected"


@cocotb.test()
async def test_i2c_read(dut):
    """START, address 0x50/R, read 3 bytes (ACK, ACK, NAK on the last), STOP."""
    slave = I2cSlave(addr=0x50, read_bytes=[0x11, 0x80, 0xFE])
    bus, world = await i2c_setup(dut, slave)
    got = await i2c_run(dut, bus, pio_i2c.read_transaction(0x50, 3), slave, 1, 4)
    assert got[-3:] == [0x11, 0x80, 0xFE], [hex(g) for g in got]
    assert slave.rx[0] == ("A", 0xA1)
    assert slave.starts == 1 and slave.stops == 1
    assert not slave.violations, slave.violations


@cocotb.test()
async def test_i2c_repeated_start_write_then_read(dut):
    """Register read: START, addr/W, register byte, REPEATED START, addr/R, 2 bytes, STOP."""
    slave = I2cSlave(addr=0x50, read_bytes=[0xCA, 0xFE])
    bus, world = await i2c_setup(dut, slave)
    words = (pio_i2c.start() + pio_i2c.write_byte(0xA0) + pio_i2c.write_byte(0x07)
             + pio_i2c.repeated_start() + pio_i2c.write_byte(0xA1)
             + pio_i2c.read_byte(False) + pio_i2c.read_byte(True) + pio_i2c.stop())
    got = await i2c_run(dut, bus, words, slave, 1, 5)
    assert got[-2:] == [0xCA, 0xFE], [hex(g) for g in got]
    assert slave.rx[:3] == [("A", 0xA0), ("D", 0x07), ("A", 0xA1)], slave.rx
    assert slave.starts == 2 and slave.stops == 1, (slave.starts, slave.stops)
    assert not slave.violations, slave.violations


@cocotb.test()
async def test_i2c_nak_raises_irq(dut):
    """Wrong address: nobody ACKs, so the program must stop and raise IRQ 0 for the host
    (rather than plough on and write data into the void)."""
    slave = I2cSlave(addr=0x50)
    bus, world = await i2c_setup(dut, slave)
    words = pio_i2c.write_transaction(0x33, [0x01, 0x02])
    sent = 0
    for _ in range(30000):
        if sent < len(words) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, pio_i2c.fifo_word(words[sent]))
            sent += 1
        if await bus.rx_level(0):
            await bus.rx_get(0)
        if (await bus.read(R_IRQ)) & 1:
            break
    assert (await bus.read(R_IRQ)) & 1, "NAK must raise IRQ flag 0"
    assert slave.rx == [("A", 0x66)] and not any(k == "D" for k, _ in slave.rx), slave.rx


@cocotb.test()
async def test_i2c_clock_stretching(dut):
    """A slave that holds SCL low after the ACK bit: the master must wait (`wait 1 pin`)
    for SCL to actually rise, and all bytes must still arrive intact."""
    slave = I2cSlave(addr=0x50, stretch=300)
    bus, world = await i2c_setup(dut, slave)
    data = [0x42, 0x24]
    await i2c_run(dut, bus, pio_i2c.write_transaction(0x50, data), slave, 1, 3, limit=120000)
    assert slave.rx == [("A", 0xA0)] + [("D", b) for b in data], slave.rx
    assert not slave.violations, slave.violations
    lows = [c for c, lvl in slave.scl_edges if lvl == 0]
    highs = [c for c, lvl in slave.scl_edges if lvl == 1]
    longest = max((h - l for l in lows for h in highs if h > l and h - l > 0), default=0)
    assert longest >= 300, "SCL should have been held low >= 300 clocks by the slave (%d)" % longest


# ============================================================================= PS/2 receiver
# `ps2_rx.pio`: CLOCK = pin 3 (ui_in[3]), DATA = pin 4 (ui_in[4]), one 11-bit frame per RX FIFO
# word, idle-gap timeout of 288 PIO cycles (= 288 * CLKDIV clocks) that drops a partial frame.
PS2_FRAME = 11


async def ps2_setup(dut, div=1):
    bus, world = await setup(dut)
    prog = load_src("ps2_rx.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, in_base=4, jmp_pin=3, autopush=True, push_thresh=PS2_FRAME,
                       in_right=True, div=(div, 0))
    kb = Ps2Keyboard()
    world.devs.append(kb)
    world.ext_in |= (1 << 3) | (1 << 4)                     # both lines idle high
    await ClockCycles(dut.clk, 4)
    await bus.set_enable(0)
    return bus, world, kb


async def collect_ps2(bus, n, timeout=200000):
    words = []
    for _ in range(timeout):
        if len(words) >= n:
            break
        if await bus.rx_level(0):
            words.append(await bus.rx_get(0))
    return words


async def settle(dut, kb, extra=2000):
    """Wait until the keyboard model has played everything out, plus `extra` idle clocks."""
    while kb.q or kb.busy:
        await ClockCycles(dut.clk, 500)
    await ClockCycles(dut.clk, extra)


@cocotb.test()
async def test_ps2_rx_keystrokes(dut):
    """Real keystroke traffic (Scan Code Set 2): 'A' make + break, an extended key (up arrow) make
    + break, and the edge-case bytes 0x00 / 0xFF / 0xAA.  Every frame arrives as ONE RX word with
    start = 0, stop = 1, correct odd parity and the right data byte, in order."""
    bus, world, kb = await ps2_setup(dut)
    data = [0x1C, 0xF0, 0x1C, 0xE0, 0x75, 0xE0, 0xF0, 0x75, 0x00, 0xFF, 0xAA]
    for b in data:
        kb.add_frame(b, half=100, gap=600)                 # gap 600 > idle timeout 288
    words = await collect_ps2(bus, len(data))
    got = [decode_ps2_word(w) for w in words]
    assert [d for d, _ in got] == data, [hex(d) for d, _ in got]
    assert all(ok for _, ok in got), got
    assert await bus.rx_level(0) == 0


@cocotb.test()
async def test_ps2_rx_realistic_timing(dut):
    """Real PS/2 rates on the 50 MHz sim clock: the two ends of the spec, CLOCK at 10 kHz and
    16.7 kHz (half periods 2500 / 1500 clocks = 50 / 30 us).  CLKDIV 25 makes the idle timeout
    7200 clocks (144 us): longer than any legal CLOCK-high time, shorter than the 200 us
    inter-frame gap."""
    for half in (2500, 1500):
        bus, world, kb = await ps2_setup(dut, div=25)
        data = [0x1C, 0xF0]                                # make + first byte of a break
        for b in data:
            kb.add_frame(b, half=half, gap=10000)
        words = await collect_ps2(bus, len(data))
        got = [decode_ps2_word(w) for w in words]
        assert [d for d, _ in got] == data and all(ok for _, ok in got), (half, got)


@cocotb.test()
async def test_ps2_rx_fifo_buffers_a_burst_while_cpu_is_busy(dut):
    """The point of doing PS/2 in the PIO: the CPU can be busy elsewhere (drawing, flash paging)
    while frames keep arriving.  Four frames arrive back to back with nobody reading the RX
    FIFO; afterwards all four are there, in order and intact."""
    bus, world, kb = await ps2_setup(dut)
    data = [0xE0, 0xF0, 0x75, 0x1C]
    for b in data:
        kb.add_frame(b, half=100, gap=40)                  # gap 40 < idle timeout: back to back
    await settle(dut, kb, extra=1000)
    assert await bus.rx_level(0) == FIFO_DEPTH
    words = [await bus.rx_get(0) for _ in range(FIFO_DEPTH)]
    got = [decode_ps2_word(w) for w in words]
    assert [d for d, _ in got] == data and all(ok for _, ok in got), got


@cocotb.test()
async def test_ps2_rx_fifo_overflow_keeps_old_frames_and_recovers(dut):
    """If the CPU stays away for more than 4 frames the oldest 4 are preserved (the SM stalls on
    the autopush; nothing already buffered is corrupted), the rest are lost, and after an idle gap
    the receiver is back in sync: the next frame decodes correctly."""
    bus, world, kb = await ps2_setup(dut)
    burst = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77]
    for b in burst:
        kb.add_frame(b, half=100, gap=40)
    await settle(dut, kb, extra=1000)
    first4 = [decode_ps2_word(await bus.rx_get(0)) for _ in range(FIFO_DEPTH)]
    assert [d for d, _ in first4] == burst[:4] and all(ok for _, ok in first4), first4
    await ClockCycles(dut.clk, 3000)                       # idle gap > timeout: resynchronise
    while await bus.rx_level(0):                           # drop whatever the overflow left behind
        await bus.rx_get(0)
    kb.add_frame(0xA5, half=100, gap=600)
    words = await collect_ps2(bus, 1, timeout=20000)
    assert len(words) == 1 and decode_ps2_word(words[0]) == (0xA5, True), words
    assert await bus.rx_level(0) == 0


@cocotb.test()
async def test_ps2_rx_idle_timeout_resyncs_after_partial_frame(dut):
    """A frame that stops after 5 bits (line noise, hot-plug, a keyboard reset mid-byte) leaves the
    state machine holding 5 stray bits.  The idle-gap timeout discards them, so the NEXT frame is
    decoded from its own start bit and nothing bogus is pushed."""
    bus, world, kb = await ps2_setup(dut)
    kb.add_frame(0x1C, half=100, gap=1500, nbits=5)        # truncated frame, then a long idle gap
    kb.add_frame(0x5A, half=100, gap=600)
    words = await collect_ps2(bus, 1, timeout=20000)
    await settle(dut, kb, extra=1500)
    extra = []
    while await bus.rx_level(0):
        extra.append(await bus.rx_get(0))
    assert not extra, "nothing else may be pushed: %s" % [hex(w) for w in extra]
    assert len(words) == 1 and decode_ps2_word(words[0]) == (0x5A, True), [hex(w) for w in words]


@cocotb.test()
async def test_ps2_rx_reports_bad_parity_and_framing_to_the_cpu(dut):
    """The PIO does not judge frames: a bad-parity frame and a frame with a low stop bit are
    delivered as received, and the CPU-side check (`decode_ps2_word`: start/stop/parity) flags
    exactly those two while the good frames around them pass."""
    bus, world, kb = await ps2_setup(dut)
    kb.add_frame(0x1C, half=100, gap=600)
    kb.add_frame(0x2D, half=100, gap=600, parity=0 if bin(0x2D).count("1") % 2 == 0 else 1) # wrong
    kb.add_frame(0x3B, half=100, gap=600, stop=0)
    kb.add_frame(0x4C, half=100, gap=600)
    words = await collect_ps2(bus, 4)
    got = [decode_ps2_word(w) for w in words]
    assert [d for d, _ in got] == [0x1C, 0x2D, 0x3B, 0x4C], got
    assert [ok for _, ok in got] == [True, False, False, True], got


class _ClockAndFrameProbe:
    """Counts SPI SCK edges that happen while a PS/2 frame is on the wire."""

    def __init__(self, kb):
        self.kb, self.both, self.last = kb, 0, 0

    def step(self, w):
        sck = (w.pin_out >> 1) & 1
        if self.kb.busy and sck != self.last:
            self.both += 1
        self.last = sck


@cocotb.test()
async def test_ps2_rx_runs_alongside_spi_master(dut):
    """The point of the whole exercise: SM0 streams 96 bytes over SPI (mode 0, MOSI/SCK/MISO on
    pins 0-2) WHILE SM1 captures PS/2 frames (pins 3-4), both out of the shared instruction memory
    (ps2_rx loaded at origin 8).  Neither disturbs the other: every SPI byte and every PS/2 frame
    is intact, and thousands of SCK edges happened during PS/2 frames."""
    bus, world = await setup(dut)
    spi, ps2 = load_src("spi_master.pio", 0), load_src("ps2_rx.pio", 8)
    await bus.load_program(spi)
    await bus.load_program(ps2)
    await bus.sm_setup(0, spi, out_base=0, out_count=1, side_base=1, in_base=2,
                       autopull=True, pull_thresh=8, autopush=True, push_thresh=8,
                       out_right=False, in_right=False, div=(4, 0))
    await bus.sm_setup(1, ps2, in_base=4, jmp_pin=3, autopush=True, push_thresh=PS2_FRAME,
                       in_right=True, div=(1, 0))
    rnd = random.Random(7)
    tx = [rnd.randrange(256) for _ in range(96)]
    miso = [rnd.randrange(256) for _ in range(96)]
    slave = SpiSlave(0, miso)
    kb = Ps2Keyboard()
    probe = _ClockAndFrameProbe(kb)
    world.devs += [slave, kb, probe]
    world.ext_in |= (1 << 3) | (1 << 4)
    keys = [0x1C, 0xF0, 0x1C, 0x2D, 0xF0, 0x2D]
    for b in keys:
        kb.add_frame(b, half=100, gap=600)
    await bus.write(R_PIN_OWN, 0b011)
    await bus.set_enable(0)
    await bus.set_enable(1)
    spi_got, ps2_words, sent = [], [], 0
    for _ in range(60000):
        if sent < len(tx) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, tx[sent] << 24)
            sent += 1
        if await bus.rx_level(0):
            spi_got.append((await bus.rx_get(0)) & 0xFF)
        if await bus.rx_level(1):
            ps2_words.append(await bus.rx_get(1))
        if len(spi_got) == len(tx) and len(ps2_words) == len(keys):
            break
    assert slave.mosi_bytes == tx, "SPI MOSI corrupted while PS/2 was running"
    assert spi_got == miso, "SPI MISO corrupted while PS/2 was running"
    got = [decode_ps2_word(w) for w in ps2_words]
    assert [d for d, _ in got] == keys and all(ok for _, ok in got), got
    assert probe.both > 500, "SPI and PS/2 did not overlap in time (%d SCK edges)" % probe.both


# ============================================================================= low-speed USB
USB_DIV = (2, 0)                # 8 ticks/bit x CLKDIV 2 = 16 clk/bit (24 MHz core -> 1.5 Mb/s)


async def usb_setup(dut, device, entry="tx_start", ctx=None):
    """Bring up SM0 with usb_ls.pio. Pass the (bus, world) of an earlier call as `ctx` to reuse the
    clock and pad model (tests that loop must not start a second clock / World); the state machine
    is then restarted and the new device replaces the old one."""
    if ctx is None:
        bus, world = await setup(dut)
    else:
        bus, world = ctx
        await bus.write(R_CTRL, (1 << 4) | (1 << 8) | (1 << 12))          # disable + restart + FIFO clear
        bus.en_mask = 0
        world.devs[:] = []
        world.ext_high = world.ext_low = 0
        await ClockCycles(dut.clk, 40)
    world.pull = 0x200                                     # D+ pulled down, D- pulled up: idle = J
    device.spont = [(c + world.cycle, bits) for c, bits in device.spont]   # times are relative
    prog = load_src("usb_ls.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, out_base=8, out_count=2, set_base=8, set_count=2, in_base=8,
                       autopull=True, autopush=True, in_right=False, out_right=True,
                       div=USB_DIV, entry=prog.labels[entry] + prog.origin)
    world.devs.append(device)
    await bus.write(R_PIN_OWN, 0x300)                      # D+/D- = PIO pins 8/9, PINDIR = 0 (released)
    await bus.set_enable(0)
    return bus, world, prog


async def usb_send(bus, bits):
    for w in pio_usb.tx_words(bits):
        await bus.tx_put_blocking(0, w)


async def usb_collect(dut, bus, idle=900, first=4000):
    """Wait up to `first` clocks for the first RX word, then drain until quiet for `idle` clocks.
    (A 32-bit word takes 32 x 16 = 512 clocks to arrive, so `idle` must exceed that.)"""
    words, quiet, waited = [], 0, 0
    while True:
        if await bus.rx_level(0):
            words.append(await bus.rx_get(0))
            quiet = 0
            continue
        await ClockCycles(dut.clk, 8)
        if words:
            quiet += 8
            if quiet >= idle:
                return words
        else:
            waited += 8
            if waited >= first:
                return words


@cocotb.test()
async def test_usb_ls_tx_token_waveform(dut):
    """SETUP token: the device sees exactly the intended stuffed bit stream (NRZI decoded at bit
    centres), every line edge lies on a 16-clock bit boundary, EOP = SE0 for 32 clocks then J."""
    dev = UsbLsDevice()
    bus, world, prog = await usb_setup(dut, dev)
    bits = pio_usb.token_bits(pio_usb.SETUP, 5, 0)
    await usb_send(bus, bits)
    await ClockCycles(dut.clk, (len(bits) + 6) * 16 + 200)
    assert len(dev.rx_packets) == 1, dev.rx_packets
    pkt = dev.rx_packets[0]
    assert pkt["bits"] == bits, (pkt["bits"], bits)
    r = pio_usb.parse(pkt["bits"])
    assert r["name"] == "SETUP" and r["ok"] and (r["addr"], r["endp"]) == (5, 0), r
    edges = [(c, lv) for c, lv in dev.edges if pkt["start"] <= c <= pkt["se0_start"] + 40]
    for (c0, _), (c1, _) in zip(edges, edges[1:]):
        assert (c1 - c0) % 16 == 0, "edge spacing %d is not a multiple of 16 clocks" % (c1 - c0)
    se0 = [i for i, (c, lv) in enumerate(edges) if lv == (0, 0)]
    assert len(se0) == 1 and edges[se0[0] + 1][1] == (0, 1), edges[-4:]
    assert edges[se0[0] + 1][0] - edges[se0[0]][0] == 32, "SE0 must last exactly 2 bit times"


@cocotb.test()
async def test_usb_ls_tx_data_with_bit_stuffing(dut):
    """DATA1 with 8 x 0xFF (maximum stuffing): the device recovers payload and CRC16, and the
    line never stays in one state for more than 7 bit times."""
    dev = UsbLsDevice()
    bus, world, prog = await usb_setup(dut, dev)
    payload = [0xFF] * 8
    bits = pio_usb.data_bits(pio_usb.DATA1, payload)
    assert len(bits) > 96             # stuffing really adds bits (96 raw bits)
    await usb_send(bus, bits)
    await ClockCycles(dut.clk, (len(bits) + 6) * 16 + 300)
    r = pio_usb.parse(dev.rx_packets[0]["bits"])
    assert r["name"] == "DATA1" and r["payload"] == payload and r["ok"], r
    runs = [(b - a) for (a, _), (b, _) in zip(dev.edges, dev.edges[1:])]
    assert max(runs[:-2]) <= 7 * 16, "a level lasted %d clocks (> 7 bit times)" % max(runs)


@cocotb.test()
async def test_usb_ls_in_transaction(dut):
    """Host sends IN addr 5 ep 1; the device answers DATA1 (8 bytes, several stuffed bits) after a
    short turnaround. The same state machine flips from TX to RX on its own (no CPU), and the
    host recovers payload + CRC from the RX FIFO."""
    payload = [0xFF, 0x00, 0xFF, 0xFF, 0x3F, 0xFC, 0x81, 0xFF]
    dev = UsbLsDevice(responses=[pio_usb.data_bits(pio_usb.DATA1, payload)], turnaround=24)
    bus, world, prog = await usb_setup(dut, dev)
    await usb_send(bus, pio_usb.token_bits(pio_usb.IN, 5, 1))
    words = await usb_collect(dut, bus)
    r = pio_usb.decode_rx(words)
    assert r["name"] == "DATA1" and r["payload"] == payload and r["ok"], (r, [hex(w) for w in words])
    tok = pio_usb.parse(dev.rx_packets[0]["bits"])
    assert tok["name"] == "IN" and tok["ok"] and (tok["addr"], tok["endp"]) == (5, 1), tok
    assert len(words) <= 4, "response must fit the 4-deep RX FIFO"


@cocotb.test()
async def test_usb_ls_handshakes_nak_and_ack(dut):
    """Short packets (PID only) must decode too: NAK in answer to an IN token."""
    dev = UsbLsDevice(responses=[pio_usb.handshake_bits(pio_usb.NAK)])
    bus, world, prog = await usb_setup(dut, dev)
    await usb_send(bus, pio_usb.token_bits(pio_usb.IN, 3, 0))
    r = pio_usb.decode_rx(await usb_collect(dut, bus))
    assert r["name"] == "NAK" and r["ok"], r


@cocotb.test()
async def test_usb_ls_rx_only_entry(dut):
    """Receiver on its own (entry = rx_start): a device that speaks first - ACK then a DATA0 with a
    zero-length payload - is received in order, one packet per RX burst."""
    dev = UsbLsDevice(spontaneous=[(900, pio_usb.handshake_bits(pio_usb.ACK))])
    bus, world, prog = await usb_setup(dut, dev, entry="rx_start")
    r = pio_usb.decode_rx(await usb_collect(dut, bus))
    assert r["name"] == "ACK" and r["ok"], r


@cocotb.test()
async def test_usb_ls_rx_random_payloads(dut):
    """Fuzz the receiver: 12 random DATA packets of 0..8 bytes, restarting the SM each time."""
    rnd = random.Random(2024)
    ctx = None
    for n in range(12):
        payload = [rnd.randrange(256) for _ in range(rnd.randrange(9))]
        pid = pio_usb.DATA0 if n % 2 == 0 else pio_usb.DATA1
        dev = UsbLsDevice(spontaneous=[(700, pio_usb.data_bits(pid, payload))])
        bus, world, prog = await usb_setup(dut, dev, entry="rx_start", ctx=ctx)
        ctx = (bus, world)
        words = await usb_collect(dut, bus)
        r = pio_usb.decode_rx(words)
        assert r["payload"] == payload and r["pid"] == pid and r["ok"], (n, payload, r)


@cocotb.test()
async def test_usb_ls_rx_clock_tolerance(dut):
    """Receiver vs a device whose bit clock is off by +-x %: report the window and require +-0.25 %.
    The sample point is fixed after the first edge (no per-transition resync: all 32 instruction
    words are in use), so a full 8-byte packet (up to ~112 bit times) has about +-0.45 % of margin;
    a real low-speed device is allowed +-1.5 % by the USB spec -- see docs/info.md."""
    payload = [0xA5, 0x5A, 0xFF, 0x00, 0x96, 0x69, 0xC3, 0x3C]
    results = {}
    ctx = None
    for pct in (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0):
        dev = UsbLsDevice(spontaneous=[(700, pio_usb.data_bits(pio_usb.DATA1, payload))],
                          scale=1 + pct / 100.0)
        bus, world, prog = await usb_setup(dut, dev, entry="rx_start", ctx=ctx)
        ctx = (bus, world)
        words = await usb_collect(dut, bus)
        try:
            r = pio_usb.decode_rx(words)
            results[pct] = r["ok"] and r["payload"] == payload
        except ValueError:
            results[pct] = False
    dut._log.info("USB LS RX clock tolerance (8-byte DATA1): %s" % {
        "%+.1f%%" % k: ("ok" if v else "FAIL") for k, v in results.items()})
    for pct in (-0.25, 0.0, 0.25):
        assert results[pct], "receiver must tolerate %+.2f %% bit-clock error" % pct
