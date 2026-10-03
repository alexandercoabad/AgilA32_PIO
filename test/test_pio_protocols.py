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
from pio_tb_lib import I2cMaster
from pio_i2c_slave import (I2cSlaveRx, I2cSlaveTx, PIN_OWN_MASK as SL_OWN, INIT_INSTRS as SL_INIT, tx_word)
from pio_i2c_mm import (I2cMM, SM_CONFIG as MM_CONFIG, PIN_OWN_MASK as MM_OWN, IRQ_LOST, IRQ_NAK,
                        shift_ctrl as mm_shift, MOV_ISR_NULL as MM_MOV_ISR_NULL)
from pio_tb_lib import (OneWireSlave, make_rom, crc8_maxim, PioBus, World, Wire, SerialSource, SpiSlave, I2cSlave, decode_uart,
                        Ps2Keyboard, decode_ps2_word, Ws2812Strip,
                        assemble, sm_reg, R_CTRL, R_IRQ, R_FSTAT, R_PIN_OWN, R_SYNC_BYP,
                        R_PINS_IN, R_INFO, R_IMEM, SM_ADDR, SM_INSTR, SM_TXF, SM_EXEC,
                        SM_SHIFT, SM_CLKDIV, SM_PINCTRL, shiftctrl, FIFO_DEPTH)

PIO_DIR = "../pio/"


def load_src(name, origin=0):
    return assemble(open(PIO_DIR + name).read(), origin=origin)


async def setup(dut, period_ns=20):
    cocotb.start_soon(Clock(dut.clk, period_ns, unit="ns").start())      # 50 MHz nominal unless told
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

# ============================================================================= I2C multi-master
def scl_period_and_high(slave):
    """(rise-to-rise gaps, high times) in clocks, from the resolved SCL edges the slave saw."""
    e = slave.scl_edges
    rises = [c for c, lvl in e if lvl == 1]
    gaps = [b - a for a, b in zip(rises, rises[1:])]
    highs = [e[k + 1][0] - e[k][0] for k in range(len(e) - 1) if e[k][1] == 1]
    return gaps, highs


async def mm_setup(dut, devs, div=(1, 0)):
    bus, world = await setup(dut)
    mm = I2cMM()
    for d in devs:
        world.devs.append(d)
    await bus.load_program(mm.prog)
    await bus.sm_setup(0, mm.prog, div=div, entry=mm.entry, **MM_CONFIG)
    await bus.write(R_PIN_OWN, MM_OWN)
    await bus.set_enable(0)
    await bus.write(sm_reg(0, SM_SHIFT), mm_shift(False))
    await bus.force(0, MM_MOV_ISR_NULL)
    return bus, world, mm


async def mm_run(bus, mm, words, stop_on_irq=True, timeout=30000, settle=3, until=None):
    """Feed `words` (TX FIFO is 4 deep); return "done", "lost" or "nak".  "done" = all words sent
    and the SM parked at entry_point; the IRQ flags are checked on every iteration."""
    i, parked = 0, 0
    for _ in range(timeout):
        flags = await bus.read(R_IRQ)
        if flags & (1 << IRQ_LOST):
            return "lost"
        if flags & (1 << IRQ_NAK):
            return "nak"
        if i < len(words) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, words[i])
            i += 1
        if i == len(words):
            pc = (await bus.read(sm_reg(0, SM_ADDR))) & 31
            parked = parked + 1 if (pc == mm.entry and await bus.tx_level(0) == 0) else 0
            if parked >= settle:
                return "done"
    raise TimeoutError("multi-master transfer never finished (%d/%d words)" % (i, len(words)))


async def mm_recover(bus, mm, flag):
    """Host recovery after IRQ 0 (NAK) or IRQ 1 (lost): flush, restart the SM, back to entry."""
    await bus.fifo_clear(0)
    await bus.write(R_CTRL, bus.en_mask | (1 << 4))
    await bus.write(R_CTRL, bus.en_mask)
    await bus.force(0, mm.entry)
    await bus.write(R_IRQ, 1 << flag)


@cocotb.test()
async def test_mm_single_master_write_and_read_no_false_arbitration(dut):
    """With nobody else on the bus the multi-master program is an ordinary master: a 3-byte write
    (slave ACKs pull SDA low while we released it -- that must NOT look like lost arbitration)
    and a 3-byte read (the slave pulls SDA low for every 0 bit while we release -- ditto)."""
    slave = I2cSlave(addr=0x50, read_bytes=[0xA5, 0x00, 0xFF])
    bus, world, mm = await mm_setup(dut, [slave])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x12, 0x34, 0xC8])) == "done"
    assert slave.rx == [("A", 0xA0), ("D", 0x12), ("D", 0x34), ("D", 0xC8)], slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations
    assert (await bus.read(R_IRQ)) & 3 == 0, "no NAK / arbitration flag expected"
    slave.rx.clear()
    assert await mm_run(bus, mm, mm.start() + mm.address(0x50, read=True)) == "done"
    await bus.write(sm_reg(0, SM_SHIFT), mm_shift(True))       # autopush only for the data bytes
    await bus.force(0, MM_MOV_ISR_NULL)
    words = mm.read_byte() + mm.read_byte() + mm.read_byte(last=True) + mm.stop()
    got, i = [], 0
    for _ in range(30000):
        if i < len(words) and await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, words[i])
            i += 1
        if await bus.rx_level(0):
            got.append((await bus.rx_get(0)) & 0xFF)
        if len(got) == 3 and i == len(words) and slave.stops == 2:
            break
    assert got == [0xA5, 0x00, 0xFF], [hex(g) for g in got]
    assert (await bus.read(R_IRQ)) & 3 == 0, "reading must not raise the arbitration flag"


@cocotb.test()
async def test_mm_pio_wins_arbitration(dut):
    """Both masters START together and send the same address; data 0x11 (PIO) vs 0x33 (model)
    first differs at bit 2, where the PIO sends 0 and the model 1.  The model must lose and let go
    of BOTH lines; the slave must receive the PIO's bytes intact; the PIO must not flag anything."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x33), ("write", 0x77), ("stop",)],
                   join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x11, 0x55])) == "done"
    assert m2.lost and m2.lost_at == (1, 2), "model must lose at data byte 1, bit 2: %s" % (m2.lost_at,)
    assert m2.low == 0, "the loser must have released SDA and SCL"
    assert slave.rx == [("A", 0xA0), ("D", 0x11), ("D", 0x55)], slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations
    assert (await bus.read(R_IRQ)) & 3 == 0
    assert world.bus_trace[-1] == 0b11


@cocotb.test()
async def test_mm_pio_loses_arbitration_releases_bus_and_retries(dut):
    """Roles swapped: the PIO sends 0x33 against the model's 0x11.  At bit 2 the PIO releases SDA
    for a 1 and reads 0 -> IRQ 1, and it must let go of SDA *and* SCL at once (the model's byte
    completes untouched: slave sees 0x11, 0x55).  After the documented recovery the PIO retries and
    its own 0x33 gets through."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x11), ("write", 0x55), ("stop",)],
                   join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    words = mm.write_transaction(0x50, [0x33])
    assert await mm_run(bus, mm, words) == "lost"
    assert (await bus.read(sm_reg(0, SM_ADDR))) & 31 == 4, "SM must be parked on `irq wait 1`"
    for _ in range(2000):                                # let the winner finish its transaction
        await ClockCycles(dut.clk, 1)
        if m2.done:
            break
    assert m2.done and not m2.lost and m2.acks == [0, 0, 0], (m2.done, m2.lost, m2.acks)
    assert slave.rx == [("A", 0xA0), ("D", 0x11), ("D", 0x55)], "winner's data corrupted: %s" % slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations
    assert world.mlow == 0, "the loser must not be driving either line"
    await mm_recover(bus, mm, IRQ_LOST)
    slave.rx.clear()
    assert await mm_run(bus, mm, words) == "done", "retry after recovery must work"
    assert slave.rx == [("A", 0xA0), ("D", 0x33)], slave.rx
    assert slave.stops == 2 and not slave.violations, slave.violations


@cocotb.test()
async def test_mm_address_phase_arbitration(dut):
    """Arbitration can be lost inside the ADDRESS byte too: the PIO addresses 0x50 (1010000x), the
    model 0x48 (1001000x); the first difference is address bit 3 (PIO releases, model drives 0) so
    the PIO loses there.  The slave only knows 0x48 -- it must ACK the model, and the PIO must
    have flagged the loss before sending any data."""
    slave = I2cSlave(addr=0x48)
    m2 = I2cMaster([("start",), ("write", 0x90), ("write", 0x5A), ("stop",)], join_start=True)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x99])) == "lost"
    for _ in range(2000):
        await ClockCycles(dut.clk, 1)
        if m2.done:
            break
    assert slave.rx == [("A", 0x90), ("D", 0x5A)], slave.rx
    assert m2.acks == [0, 0] and not slave.violations, (m2.acks, slave.violations)


@cocotb.test()
async def test_mm_clock_synchronisation_with_slower_master(dut):
    """A second master with a 2x slower clock (period 128) writes the SAME bytes: the wired-AND SCL
    is low until the slow one releases, and the PIO's `wait 1 pin, 1` must hold its high phase off
    until SCL is really high.  Both complete without arbitration loss, and the slave sees clean
    data with no violations."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x5A), ("stop",)], join_start=True, period=128)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x5A]), timeout=60000) == "done"
    assert not m2.lost and m2.acks == [0, 0], (m2.lost, m2.acks)
    assert slave.rx[:2] == [("A", 0xA0), ("D", 0x5A)], slave.rx
    assert not slave.violations, slave.violations
    gaps, highs = scl_period_and_high(slave)
    # I2C clock synchronisation: SCL low lasts as long as the SLOWEST master's low phase, high as
    # long as the FASTEST master's high phase.  Here that is ~82 clocks per bit (the PIO alone
    # does 33), and it must be stable from bit to bit.
    assert min(gaps[:8]) >= 70, "SCL must slow down to the slower master: %s" % gaps[:8]
    assert max(gaps[:8]) - min(gaps[:8]) <= 2, "SCL period must be stable: %s" % gaps[:8]


async def mm_wait_model(dut, m2, limit=4000):
    for _ in range(limit):
        await ClockCycles(dut.clk, 1)
        if m2.done:
            break
    await ClockCycles(dut.clk, 10)


@cocotb.test()
async def test_mm_identical_frames_neither_loses(dut):
    """Two masters that send exactly the same frame never see a difference on the wire, so neither
    may lose: no arbitration flag, the model does not lose either, and the slave sees ONE START and
    one transaction with the right bytes (the textbook multi-master property).  Guards against a
    false 'lost' when another master's SDA drive matches ours."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x12), ("write", 0x34), ("stop",)],
                   join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x12, 0x34])) == "done"
    await mm_wait_model(dut, m2)
    assert m2.done and not m2.lost and m2.acks == [0, 0, 0], (m2.done, m2.lost, m2.acks)
    assert slave.rx == [("A", 0xA0), ("D", 0x12), ("D", 0x34)], slave.rx
    assert slave.starts == 1 and not slave.violations, (slave.starts, slave.violations)
    assert (await bus.read(R_IRQ)) & 3 == 0, "no NAK / arbitration flag expected"
    assert world.bus_trace[-1] == 0b11


@cocotb.test()
async def test_mm_pio_loses_on_the_last_bit_of_a_byte(dut):
    """The two data bytes differ only in bit 0 (PIO 0x13, model 0x12): the loss is decided on the
    LAST bit of the byte, right before the ACK slot.  The PIO must notice it, let go of both lines
    at once and NOT clock an ACK slot of its own: the model's byte completes untouched, the slave
    ACKs it (model sees ACK), and the slave data is 0x12."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x12), ("stop",)], join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x13])) == "lost"
    await mm_wait_model(dut, m2)
    assert m2.done and not m2.lost and m2.acks == [0, 0], (m2.done, m2.lost, m2.acks)
    assert slave.rx == [("A", 0xA0), ("D", 0x12)], slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations
    assert world.mlow == 0, "the loser must not be driving either line"


@cocotb.test()
async def test_mm_model_loses_on_the_last_bit_of_a_byte(dut):
    """Mirror: PIO 0x12 vs model 0x13.  The PIO sends the 0 in bit 0 and wins; the model must lose
    exactly there (byte 1, bit 7) and let go; the slave receives the PIO's 0x12."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x13), ("stop",)], join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x12])) == "done"
    await mm_wait_model(dut, m2)
    assert m2.lost and m2.lost_at == (1, 7), m2.lost_at
    assert m2.low == 0
    assert slave.rx == [("A", 0xA0), ("D", 0x12)], slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations
    assert (await bus.read(R_IRQ)) & 3 == 0


@cocotb.test()
async def test_mm_read_versus_write_to_the_same_address(dut):
    """Same address, different direction: the PIO READS (0xA1), the model WRITES (0xA0).  The R/W bit
    is the last bit of the address byte; the PIO releases SDA for the 1 and the model drives the 0,
    so the PIO loses.  Its flag is raised, it lets go, and the model's write completes (the slave
    sees a WRITE of 0x5A, never a read)."""
    slave = I2cSlave(addr=0x50, read_bytes=[0xEE])
    m2 = I2cMaster([("start",), ("write", 0xA0), ("write", 0x5A), ("stop",)], join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.start() + mm.address(0x50, read=True)) == "lost"
    await mm_wait_model(dut, m2)
    assert m2.done and not m2.lost and m2.acks == [0, 0], (m2.done, m2.lost, m2.acks)
    assert slave.rx == [("A", 0xA0), ("D", 0x5A)], slave.rx
    assert world.mlow == 0 and not slave.violations, slave.violations


@cocotb.test()
async def test_mm_write_versus_read_to_the_same_address(dut):
    """Mirror: the PIO WRITES (0xA0), the model READS (0xA1).  The PIO drives the 0 in the R/W bit,
    so the MODEL loses there (address byte, bit 7) and lets go; the PIO's write completes."""
    slave = I2cSlave(addr=0x50, read_bytes=[0xEE])
    m2 = I2cMaster([("start",), ("write", 0xA1), ("read", False), ("stop",)], join_start=True, period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    assert await mm_run(bus, mm, mm.write_transaction(0x50, [0x77])) == "done"
    await mm_wait_model(dut, m2)
    assert m2.lost and m2.lost_at == (0, 7), m2.lost_at
    assert m2.low == 0 and m2.reads == []
    assert slave.rx == [("A", 0xA0), ("D", 0x77)], slave.rx
    assert (slave.starts, slave.stops) == (1, 1) and not slave.violations, slave.violations


# ============================================================================= I2C slave
async def slave_setup(dut, slave, master):
    bus, world = await setup(dut)
    world.devs.append(master)
    await bus.load_program(slave.prog)
    await bus.sm_setup(0, slave.prog, div=(1, 0), entry=slave.idle, **slave.config)
    await bus.write(R_PIN_OWN, SL_OWN)
    await bus.tx_put(0, slave.addr7)                       # own address -> Y (forced pull + mov y, osr)
    for ins in SL_INIT:
        await bus.force(0, ins)
    await bus.force(0, slave.idle)
    await bus.set_enable(0)
    return bus, world


async def drain_rx(bus, n, master, extra=6000, timeout=30000):
    """Collect up to `n` RX bytes; return once the master script is done (plus `extra` clocks)."""
    got, tail = [], extra
    for _ in range(timeout):
        if await bus.rx_level(0):
            got.append((await bus.rx_get(0)) & 0xFF)
        if master.done:
            tail -= 1
            if tail <= 0 or len(got) >= n:
                break
    return got


@cocotb.test()
async def test_i2c_slave_rx_write(dut):
    """An external master writes 0x42 + [0xA5, 0x3C].  The slave ACKs the address and both bytes (the
    master sees ACK, ACK, ACK on the wire), delivers exactly those two bytes to the RX FIFO, and never
    drives SCL."""
    m = I2cMaster([("start",), ("write", 0x84), ("write", 0xA5), ("write", 0x3C), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=2), m)
    got = await drain_rx(bus, 2, m)
    assert got == [0xA5, 0x3C], [hex(g) for g in got]
    assert m.acks == [0, 0, 0], "master must see ACK ACK ACK, got %s" % m.acks
    assert not m.lost and world.mlow & 0x200 == 0, "slave must never pull SCL low"


@cocotb.test()
async def test_i2c_slave_rx_ignores_other_addresses_and_reads(dut):
    """Not our address -> no ACK and nothing in the RX FIFO.  A READ request (R/W = 1) to our own address
    is NAKed too (this program is write-only).  The slave is not wedged by either: a proper write afterwards
    goes through."""
    m = I2cMaster([("start",), ("write", 0x86), ("stop",),              # 0x43 write: not us
                   ("start",), ("write", 0x85), ("stop",),              # 0x42 READ: unsupported
                   ("start",), ("write", 0x84), ("write", 0x11), ("write", 0x22), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=2), m)
    got = await drain_rx(bus, 2, m)
    assert m.acks == [1, 1, 0, 0, 0], "NAK, NAK, then ACK x3: %s" % m.acks
    assert got == [0x11, 0x22], [hex(g) for g in got]


@cocotb.test()
async def test_i2c_slave_rx_buffer_full_naks_extra_bytes(dut):
    """The program accepts exactly WRITE_BYTES (here 2) per transaction: a third byte is NAKed, nothing more
    reaches the FIFO, and the next transaction is unaffected ('buffer full' semantics)."""
    m = I2cMaster([("start",), ("write", 0x84), ("write", 0x01), ("write", 0x02), ("write", 0x03), ("stop",),
                   ("start",), ("write", 0x84), ("write", 0x04), ("write", 0x05), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=2), m)
    got = await drain_rx(bus, 4, m)
    assert got == [1, 2, 4, 5], got
    assert m.acks == [0, 0, 0, 1, 0, 0, 0], "third byte NAKed: %s" % m.acks


@cocotb.test()
async def test_i2c_slave_rx_repeated_start_and_back_to_back(dut):
    """STOP+START and repeated START both re-arm the slave: START addr 2 bytes REPSTART addr 2 bytes STOP."""
    m = I2cMaster([("start",), ("write", 0x84), ("write", 0x10), ("write", 0x20),
                   ("repstart",), ("write", 0x84), ("write", 0x30), ("write", 0x40), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=2), m)
    got = await drain_rx(bus, 4, m)
    assert got == [0x10, 0x20, 0x30, 0x40], [hex(g) for g in got]
    assert m.acks == [0] * 6, m.acks


@cocotb.test()
async def test_i2c_slave_rx_single_byte_mode_and_all_values(dut):
    """WRITE_BYTES = 1 (the register-pointer pattern) with every bit pattern that could confuse the
    START detector or the address compare: 0x00, 0xFF, 0x01, 0x80 -- and an address differing in one bit."""
    m = I2cMaster([("start",), ("write", 0x84), ("write", 0x00), ("stop",),
                   ("start",), ("write", 0x84), ("write", 0xFF), ("stop",),
                   ("start",), ("write", 0x84), ("write", 0x01), ("stop",),
                   ("start",), ("write", 0x84), ("write", 0x80), ("stop",),
                   ("start",), ("write", 0x94), ("write", 0x77), ("stop",)])      # 0x4A: one bit off
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=1), m)
    got = await drain_rx(bus, 4, m)
    assert got == [0x00, 0xFF, 0x01, 0x80], [hex(g) for g in got]
    assert m.acks[:8] == [0, 0] * 4 and m.acks[8] == 1, m.acks


async def feed_tx(bus, slave_words, master, timeout=30000, extra=4000):
    """Keep the 4-deep TX FIFO topped up with `slave_words` until the master script is done."""
    i, tail = 0, extra
    for _ in range(timeout):
        level = await bus.tx_level(0)              # every iteration must consume bus cycles
        if i < len(slave_words) and level < FIFO_DEPTH:
            await bus.tx_put(0, slave_words[i])
            i += 1
        if master.done:
            tail -= 1
            if tail <= 0:
                return
    raise TimeoutError("master script never finished (slave hung or never answered; %d words queued)" % len(slave_words))


@cocotb.test()
async def test_i2c_slave_tx_read(dut):
    """The master reads 4 bytes from us (ACK, ACK, ACK, then NAK on the last) and gets exactly the bytes the
    host queued -- including 0x00 and 0xFF, which are all-released / all-driven on the wire.  The address ACK
    shows on the wire; after the NAK the slave is idle and a second read transaction works."""
    data = [0x5A, 0x00, 0xFF, 0xC3]
    m = I2cMaster([("start",), ("write", 0x85), ("read", True), ("read", True), ("read", True),
                   ("read", False), ("stop",),
                   ("start",), ("write", 0x85), ("read", False), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveTx(0x42), m)
    await feed_tx(bus, [tx_word(b) for b in data + [0x3E]], m)
    assert m.reads == data + [0x3E], [hex(r) for r in m.reads]
    assert m.acks == [0, 0], "address must be ACKed both times: %s" % m.acks
    assert not m.lost


@cocotb.test()
async def test_i2c_slave_tx_stretches_clock_until_host_supplies_data(dut):
    """With the TX FIFO EMPTY the slave must hold SCL low after ACKing the address (clock stretching via
    side-set on the blocking `pull`) for as long as it takes -- here 3000 clocks -- while the MASTER has
    released SCL (so the low is the slave's doing).  When the host finally queues a byte the read completes.
    Data setup: the byte's first bit must be on the wire at least one clock BEFORE the slave lets SCL rise,
    or a master that sees SCL high at once would sample the old level.  The slave is still holding SDA low
    from the address ACK, so the check uses a first bit of 1: SDA must already have been released."""
    m = I2cMaster([("start",), ("write", 0x85), ("read", False), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveTx(0x42), m)
    stretched = 0
    for _ in range(3000):
        await ClockCycles(dut.clk, 1)
        scl = (world.resolve() >> 9) & 1
        if m.acks and not m.reads and scl == 0 and (m.low & 0x200) == 0:
            stretched += 1                                # bus low although the master let go
    assert m.acks == [0] and not m.reads, (m.acks, m.reads)
    assert stretched > 2000, "SCL must be held low by the slave while it waits (%d of 3000 clocks)" % stretched
    mark = len(world.bus_trace)
    await bus.tx_put(0, tx_word(0xA7))                    # first bit 1: SDA must rise before SCL does
    for _ in range(4000):
        await ClockCycles(dut.clk, 1)
        if m.done:
            break
    assert m.reads == [0xA7], [hex(r) for r in m.reads]
    tr = world.bus_trace[mark:]
    k = next(i for i in range(1, len(tr)) if (tr[i] >> 1) & 1 and not (tr[i - 1] >> 1) & 1)   # SCL rises
    assert (tr[k - 1] & 1) == 1, "SDA must already be high one clock before SCL is released (data setup)"
    assert world.bus_trace[-1] == 0b11, "bus must be released after STOP"


@cocotb.test()
async def test_i2c_slave_tx_ignores_writes_and_other_addresses(dut):
    """A write to our address and any other address are NAKed (this program only serves reads) and do not
    consume the queued data byte; the read that follows gets it."""
    m = I2cMaster([("start",), ("write", 0x84), ("stop",),          # write to 0x42: unsupported
                   ("start",), ("write", 0x97), ("stop",),          # read from 0x4B: not us
                   ("start",), ("write", 0x85), ("read", False), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveTx(0x42), m)
    await feed_tx(bus, [tx_word(0x6D)], m)
    assert m.acks == [1, 1, 0], "NAK, NAK, ACK: %s" % m.acks
    assert m.reads == [0x6D], [hex(r) for r in m.reads]


@cocotb.test()
async def test_i2c_slave_tx_long_read_with_fifo_refill(dut):
    """12 random bytes read while the host refills the 4-deep TX FIFO on the fly (the slave stretches SCL
    whenever the host is late).  Every byte must arrive intact and in order."""
    rnd = random.Random(1011)
    data = [rnd.randrange(256) for _ in range(12)]
    m = I2cMaster([("start",), ("write", 0x85)] + [("read", True)] * 11 + [("read", False), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveTx(0x42), m)
    await feed_tx(bus, [tx_word(b) for b in data], m)
    assert m.reads == data, [hex(r) for r in m.reads]
    assert m.acks == [0]


def ack_setup_ok(world, ack_rise_indices):
    """True if SDA was already low in the very clock SCL rose, for each given SCL rising edge (0-based)."""
    tr = world.bus_trace
    rises = [i for i in range(1, len(tr)) if (tr[i] >> 1) & 1 and not (tr[i - 1] >> 1) & 1]
    return all(len(rises) > k and (tr[rises[k]] & 1) == 0 for k in ack_rise_indices)


@cocotb.test()
async def test_i2c_slave_scl_speed_limits(dut):
    """How fast an SCL can the slaves follow?  Sweep the master's SCL period (clocks; its low phase is half of
    it) with both slaves at CLKDIV 1.  A master only sees a valid ACK if SDA is ALREADY low when SCL rises, so
    this checks the ACK on the wire at the rising edge itself (not a late sample).  The slave needs ~12 ticks
    after the 8th SCL fall to drive it, so there is a hard limit; the table is logged."""
    ok = {}
    for period in (16, 24, 32, 40, 48, 64):
        m = I2cMaster([("start",), ("write", 0x84), ("write", 0xA5), ("stop",)], period=period)
        bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=1), m)
        got = await drain_rx(bus, 1, m, extra=2000)
        rx_ok = got == [0xA5] and m.acks == [0, 0] and ack_setup_ok(world, (8, 17))
        m = I2cMaster([("start",), ("write", 0x85), ("read", False), ("stop",)], period=period)
        bus, world = await slave_setup(dut, I2cSlaveTx(0x42), m)
        await bus.tx_put(0, tx_word(0x5C))
        for _ in range(6000):
            await ClockCycles(dut.clk, 1)
            if m.done:
                break
        tx_ok = m.reads == [0x5C] and m.acks == [0] and ack_setup_ok(world, (8,))
        ok[period] = (rx_ok, tx_ok)
    dut._log.info("I2C slave, ACK valid at the SCL rising edge, vs SCL period (clocks): %s" % {
        p: ("rx " + ("ok" if r else "FAIL"), "tx " + ("ok" if t else "FAIL")) for p, (r, t) in ok.items()})
    for p in (32, 40, 48, 64):
        assert ok[p] == (True, True), "documented range (SCL period >= 32 clocks) must work: %s" % ok
    for p in (16, 24):
        assert ok[p] != (True, True), "a %d-clock SCL period leaves too little time to ACK: %s" % (p, ok)


# ============================================================================= multi-master: bus busy
MM_BUSY_SCRIPT = [("start",), ("write", 0xA0), ("write", 0x11), ("write", 0x55), ("write", 0x66), ("stop",)]
MM_BUSY_BYTES = [("A", 0xA0), ("D", 0x11), ("D", 0x55), ("D", 0x66)]


async def mm_busy_run(dut, offset, wait_free):
    """Another master (the model) is already part-way through a 4-byte write when the PIO master is
    asked to write 0x33 to the same slave `offset` clocks after the model's START.  With
    `wait_free` the host first reads PINS_IN until SDA and SCL have both been high for 128 clocks
    (the bus-free time the model itself waits for); without it, nothing checks."""
    slave = I2cSlave(addr=0x50)
    m2 = I2cMaster(list(MM_BUSY_SCRIPT), period=64)
    bus, world, mm = await mm_setup(dut, [slave, m2])
    for _ in range(20000):
        await ClockCycles(dut.clk, 1)
        if slave.starts >= 1:
            break
    await ClockCycles(dut.clk, offset)
    if wait_free:
        free_since = None
        for _ in range(40000):
            pins = await bus.read(R_PINS_IN)
            if (pins >> 8) & 3 == 3:
                free_since = world.cycle if free_since is None else free_since
                if world.cycle - free_since >= 128:
                    break
            else:
                free_since = None
    res = await mm_run(bus, mm, mm.write_transaction(0x50, [0x33]), timeout=12000)
    for _ in range(8000):
        await ClockCycles(dut.clk, 1)
        if m2.done:
            break
    return res, slave, m2


@cocotb.test()
async def test_mm_no_bus_busy_check_start_in_the_address_byte_destroys_the_other_master(dut):
    """LIMITATION (documented in docs/info.md): i2c_mm.pio never checks that the bus is free before
    a START.  The PIO's START lands 30 clocks into the other master's transaction, while SCL is high
    and SDA is released: the slave sees a second START, the other master's address byte is lost
    (every byte NAKed, the slave captured garbage) -- and the PIO still reports 'arbitration lost'.
    If i2c_mm.pio ever gains a bus-free check this test must be changed with the docs."""
    res, slave, m2 = await mm_busy_run(dut, 30, wait_free=False)
    assert res == "lost", res
    assert slave.starts == 2, "the PIO's START must have shown up as a second START (%d)" % slave.starts
    assert slave.rx != MM_BUSY_BYTES and m2.acks == [1, 1, 1, 1], (slave.rx, m2.acks)


@cocotb.test()
async def test_mm_no_bus_busy_check_start_in_a_data_byte_corrupts_it(dut):
    """Same limitation, later in the transaction (1500 clocks in, during data byte 3): the other
    master's byte 0x55 reaches the slave as 0x6C and its last two bytes are NAKed."""
    res, slave, m2 = await mm_busy_run(dut, 1500, wait_free=False)
    assert res == "lost", res
    assert slave.starts == 2, slave.starts
    assert slave.rx[:2] == MM_BUSY_BYTES[:2] and slave.rx[2] != ("D", 0x55), slave.rx
    assert m2.acks[:2] == [0, 0] and m2.acks[2:] == [1, 1], m2.acks


@cocotb.test()
async def test_mm_host_bus_free_wait_protects_the_other_master_early(dut):
    """The mitigation: the host waits for SDA and SCL to be high for 128 clocks before it queues the
    transaction.  Same timing as the address-byte case above: the other master's four bytes arrive
    intact, then the PIO's own write (0x33) goes through: two clean transactions."""
    res, slave, m2 = await mm_busy_run(dut, 30, wait_free=True)
    assert res == "done", res
    assert m2.acks == [0, 0, 0, 0] and not m2.lost, (m2.acks, m2.lost)
    assert slave.rx == MM_BUSY_BYTES + [("A", 0xA0), ("D", 0x33)], slave.rx
    assert (slave.starts, slave.stops) == (2, 2) and not slave.violations, slave.violations


@cocotb.test()
async def test_mm_host_bus_free_wait_protects_the_other_master_late(dut):
    """Same mitigation, same timing as the data-byte case above (1500 clocks in)."""
    res, slave, m2 = await mm_busy_run(dut, 1500, wait_free=True)
    assert res == "done", res
    assert m2.acks == [0, 0, 0, 0] and not m2.lost, (m2.acks, m2.lost)
    assert slave.rx == MM_BUSY_BYTES + [("A", 0xA0), ("D", 0x33)], slave.rx
    assert (slave.starts, slave.stops) == (2, 2) and not slave.violations, slave.violations


# ============================================================================= WS2812 / NeoPixel
# `ws2812.pio`: side-set pin 0 is the data line, 10 PIO ticks per bit, tick = 125 ns = CLKDIV 3 at the
# 24 MHz this chip is constrained to (period 41.667 ns).  Datasheet windows (WS2812B): T0H 400, T0L 850,
# T1H 800, T1L 450 ns, each +-150 ns; bit period 1250 ns.
WS_CLK_NS = 41.666                # 24 MHz (41.667 ns is odd in ps; the simulator wants an even period)
WS_TICK_CLK = 3                      # CLKDIV
WS_BIT_CLK = 10 * WS_TICK_CLK        # 30 clocks = 1.25 us


async def ws_setup(dut, pull_thresh=24, reset_us=50.0):
    bus, world = await setup(dut, period_ns=WS_CLK_NS)
    prog = load_src("ws2812.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, side_base=0, autopull=True, pull_thresh=pull_thresh,
                       out_right=False, div=(WS_TICK_CLK, 0))
    strip = Ws2812Strip(pin=0, clk_ns=WS_CLK_NS, reset_us=reset_us)
    world.devs.append(strip)
    await bus.write(R_PIN_OWN, 0b001)
    return bus, world, strip


def grb(g, r, b):
    return ((g << 16) | (r << 8) | b) << 8       # the 24 bits sent are the TOP 24 of the TX word


async def ws_feed(dut, bus, words, timeout=400000):
    """Push `words` as TX room appears (the CPU-in-a-hurry case: no gap between pixels)."""
    sent = 0
    for _ in range(timeout):
        if sent == len(words):
            return
        if await bus.tx_level(0) < FIFO_DEPTH:
            await bus.tx_put(0, words[sent])
            sent += 1
    raise AssertionError("feed timed out")


def ws_check_windows(strip, tol_ns=150.0):
    """Every pulse of every bit inside the WS2812B datasheet windows, and all bit periods exact."""
    typ = {0: (400.0, 850.0), 1: (800.0, 450.0)}
    bits = [b for fr in strip.frames for b in fr] + strip.cur
    assert len(bits) == len(strip.high_clk)
    for i, (b, hi, lo) in enumerate(zip(bits, strip.high_clk, strip.low_clk)):
        th, tl = typ[b]
        assert abs(hi * strip.clk_ns - th) <= tol_ns, (i, b, "T%dH = %.0f ns" % (b, hi * strip.clk_ns))
        last_of_frame = i + 1 == len(bits) or strip.frame_of_bit[i + 1] != strip.frame_of_bit[i]
        if not last_of_frame:                       # the last low of a frame runs into the reset gap
            assert abs(lo * strip.clk_ns - tl) <= tol_ns, (i, b, "T%dL = %.0f ns" % (b, lo * strip.clk_ns))
    assert strip.glitches == 0


@cocotb.test()
async def test_ws2812_single_pixel_exact_timing(dut):
    """One GRB pixel: data decoded from the pulse widths is right, every bit is exactly 10 ticks
    (30 clocks), and 0 / 1 pulses are 3 / 7 ticks high (9 / 21 clocks) -- inside the datasheet
    windows with room to spare on both sides."""
    bus, world, strip = await ws_setup(dut)
    pixel = (0xC3, 0x5A, 0x96)                       # G R B: both bit values, runs and alternations
    await bus.tx_put(0, grb(*pixel))
    await bus.set_enable(0)
    await ClockCycles(dut.clk, 24 * WS_BIT_CLK + 2500)
    assert len(strip.frames) == 1 and not strip.cur, "frame must be latched after the idle gap"
    assert Ws2812Strip.pixels(strip.frames[0]) == [(0xC3 << 16) | (0x5A << 8) | 0x96]
    assert strip.period_clk == [WS_BIT_CLK] * 23, set(strip.period_clk)
    assert {strip.high_clk[i] for i, b in enumerate(strip.frames[0]) if b == 0} == {3 * WS_TICK_CLK}
    assert {strip.high_clk[i] for i, b in enumerate(strip.frames[0]) if b == 1} == {7 * WS_TICK_CLK}
    ws_check_windows(strip)
    assert strip.level == 0, "the line must idle LOW after the last bit"


@cocotb.test()
async def test_ws2812_eight_pixels_back_to_back(dut):
    """8 pixels streamed as TX room appears: ONE frame of 192 bits with no extra clock anywhere, not even
    at the pixel boundaries (autopull refills the OSR inside the bit loop), then one latch."""
    bus, world, strip = await ws_setup(dut)
    rnd = random.Random(2812)
    pixels = [(rnd.randrange(256), rnd.randrange(256), rnd.randrange(256)) for _ in range(8)]
    await bus.set_enable(0)
    await ws_feed(dut, bus, [grb(*p) for p in pixels])
    await ClockCycles(dut.clk, 8 * 24 * WS_BIT_CLK + 2500)
    assert len(strip.frames) == 1, "a gap inside the frame split it into %d frames" % len(strip.frames)
    want = [(g << 16) | (r << 8) | b for g, r, b in pixels]
    assert Ws2812Strip.pixels(strip.frames[0]) == want
    assert strip.period_clk == [WS_BIT_CLK] * (8 * 24 - 1), set(strip.period_clk)
    ws_check_windows(strip)


@cocotb.test()
async def test_ws2812_two_frames_each_latched(dut):
    """Frame A (3 pixels), > reset time of silence, frame B (2 pixels): both latched, in order, and the
    line is low in between and afterwards."""
    bus, world, strip = await ws_setup(dut)
    a = [(1, 2, 3), (0xFF, 0, 0x80), (0x10, 0x20, 0x30)]
    b = [(0xAA, 0x55, 0xAA), (0, 0, 0)]
    await bus.set_enable(0)
    await ws_feed(dut, bus, [grb(*p) for p in a])
    await ClockCycles(dut.clk, 3 * 24 * WS_BIT_CLK + 2500)           # idle 2500 clocks = 104 us > 50 us
    assert len(strip.frames) == 1
    await ws_feed(dut, bus, [grb(*p) for p in b])
    await ClockCycles(dut.clk, 2 * 24 * WS_BIT_CLK + 2500)
    px = lambda frame: Ws2812Strip.pixels(frame)
    conv = lambda ps: [(g << 16) | (r << 8) | bb for g, r, bb in ps]
    assert len(strip.frames) == 2 and not strip.cur
    assert px(strip.frames[0]) == conv(a) and px(strip.frames[1]) == conv(b)
    assert strip.level == 0


@cocotb.test()
async def test_ws2812_feed_gap_longer_than_reset_splits_the_frame(dut):
    """The hazard this chip's CPU has to respect, caught by the model: if the feeder leaves the TX FIFO
    empty for longer than the strip's reset time between two pixels, the strip latches after the FIRST pixel
    and the second pixel starts a new frame -- on a real chain it would land on LED 0 again."""
    bus, world, strip = await ws_setup(dut)
    await bus.set_enable(0)
    await bus.tx_put(0, grb(0x12, 0x34, 0x56))
    await ClockCycles(dut.clk, 24 * WS_BIT_CLK + 2500)               # 104 us of silence > 50 us reset
    await bus.tx_put(0, grb(0x78, 0x9A, 0xBC))
    await ClockCycles(dut.clk, 24 * WS_BIT_CLK + 2500)
    assert len(strip.frames) == 2, "expected two separate frames, got %d" % len(strip.frames)
    assert [Ws2812Strip.pixels(f)[0] for f in strip.frames] == [0x123456, 0x789ABC]
    # ... and a longer-reset strip (280 us) would NOT have split it:
    assert 2500 * WS_CLK_NS / 1000.0 < 280.0


@cocotb.test()
async def test_ws2812_rgbw_32_bit_pixels(dut):
    """SK6812 RGBW: autopull threshold 32 makes the same program send 32-bit pixels."""
    bus, world, strip = await ws_setup(dut, pull_thresh=32)
    words = [0x11223344, 0xFF00FF00, 0x80000001]
    await bus.set_enable(0)
    await ws_feed(dut, bus, words)
    await ClockCycles(dut.clk, 3 * 32 * WS_BIT_CLK + 2500)
    assert len(strip.frames) == 1
    assert Ws2812Strip.pixels(strip.frames[0], nbits=32) == words
    assert strip.period_clk == [WS_BIT_CLK] * (3 * 32 - 1)


# ============================================================================= 1-Wire
US = 24                                                    # clocks per microsecond (24 MHz; CLKDIV 24 = 1 us tick)


def ow_xfer(nbits, data):
    """Command word: op 1, n-1 in [5:1], TX data (LSB first) from bit 6."""
    assert 1 <= nbits <= 26
    return 1 | ((nbits - 1) << 1) | ((data & ((1 << nbits) - 1)) << 6)


def ow_read(nbits):
    return ow_xfer(nbits, (1 << nbits) - 1)               # a read slot is a write-1 slot


OW_RESET = 0


async def ow_setup(dut, slave=None, div=(24, 0)):
    """SM0 = pio/onewire.pio on pad 8: SET and IN both based at pin 8, 1 PIO tick = div clocks."""
    bus, world = await setup(dut)
    prog = load_src("onewire.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, set_base=8, set_count=1, in_base=8, div=div)
    await bus.write(R_PIN_OWN, 0x100)
    if slave is not None:
        world.devs.append(slave)
    await bus.set_enable(0)
    return bus, world, prog


async def ow_cmd(bus, word, limit=400000):
    """Push one command and return the single RX word it produces."""
    await bus.tx_put_blocking(0, word)
    for _ in range(limit):
        if await bus.rx_level(0):
            return await bus.rx_get(0)
    raise TimeoutError("no RX word for command %#x" % word)


async def ow_session(bus, steps):
    """Run (word, rx_shift) steps; returns the list of decoded results (None for writes)."""
    out = []
    for word, sh in steps:
        w = await ow_cmd(bus, word)
        out.append(None if sh is None else w >> sh)
    return out


@cocotb.test()
async def test_onewire_reset_and_presence(dut):
    """Reset pulse is 480..960 us, a present device is seen (bit 31 = 0), and the controller leaves
    >= 480 us of recovery before the next slot. Device answering as LATE as 60 us after the release and
    as SHORT as 60 us (the guaranteed-overlap window is 60..75 us) and the earliest/shortest, 15 us..75 us,
    are both detected."""
    for delay, length in ((30, 120), (60, 60), (15, 60), (60, 240)):
        slave = OneWireSlave(us=US, presence_delay=delay, presence_len=length)
        bus, world, prog = await ow_setup(dut, slave)
        await bus.tx_put(0, OW_RESET)
        await bus.tx_put(0, ow_xfer(1, 1))                  # one slot right behind the reset
        await bus.set_enable(0)
        w0 = await bus.rx_get(0) if await bus.rx_level(0) else None
        while w0 is None:
            if await bus.rx_level(0):
                w0 = await bus.rx_get(0)
        assert (w0 >> 31) == 0, "presence not seen for device answering at %d us for %d us" % (delay, length)
        for _ in range(100000):
            if await bus.rx_level(0):
                await bus.rx_get(0)
                break
        t_fall, low = slave.resets[0]
        assert 480 * US <= low <= 960 * US, "reset low %.1f us" % (low / US)
        gap = slave.slots[0][0] - (t_fall + low)
        assert gap >= 480 * US, "only %.1f us between the reset release and the next slot" % (gap / US)
        assert not slave.violations, slave.violations
        world.devs.clear()
        await bus.set_enable(0, False)                        # next iteration builds a fresh SM state below
        await bus.fifo_clear(0)
        await bus.write(sm_reg(0, SM_INSTR), 0)               # jmp 0 (program start)
        await ClockCycles(dut.clk, 4)


@cocotb.test()
async def test_onewire_absent_device(dut):
    """No device: the line stays high, so presence reads 1 and a read returns all ones."""
    bus, world, prog = await ow_setup(dut, OneWireSlave(us=US, present=False))
    await bus.set_enable(0)
    r = await ow_session(bus, [(OW_RESET, 31), (ow_read(8), 24)])
    assert r == [1, 0xFF], r


@cocotb.test()
async def test_onewire_write_slot_timing(dut):
    """Write 0x33 after a reset. The device decodes it LSB first (bits 1,1,0,0,1,1,0,0), and every slot
    is within the spec windows: write-1 low 3 us, write-0 low 65 us, slots 60..80 us apart, with no
    low pulse the device could misread."""
    slave = OneWireSlave(us=US)
    bus, world, prog = await ow_setup(dut, slave)
    await bus.set_enable(0)
    r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(8, 0x33), 24)])
    await ow_cmd(bus, ow_read(1))                              # one more slot: waits until the write is done
    assert r[0] == 0
    # every slot shifts exactly ONE bit into the RX word, write-0 slots included (the dummy IN after the 0-slot's
    # recovery); the released line reads 1 in all eight: without the dummy IN only the four 1-slots would shift
    assert r[1] == 0xFF, "write returned %#x, expected 0xFF (one sample per slot)" % r[1]
    assert slave.cmds == [[0x33]], slave.cmds
    assert [b for _, _, b in slave.slots[:8]] == [1, 1, 0, 0, 1, 1, 0, 0]
    for t_fall, low, bit in slave.slots[:8]:
        want = 3 if bit else 65
        assert abs(low - want * US) <= 2, "write-%d low pulse %.2f us, wanted %d" % (bit, low / US, want)
    falls = [t for t, _, _ in slave.slots[:8]]      # the 9th slot follows a command boundary
    gaps = [(b - a) / US for a, b in zip(falls, falls[1:])]
    assert all(60 <= g <= 80 for g in gaps), "slot spacing out of 60..80 us: %s" % gaps
    assert not slave.violations, slave.violations


@cocotb.test()
async def test_onewire_read_rom(dut):
    """The classic READ ROM (0x33): the device returns family code, 6 serial bytes and the CRC. Read 8
    bytes one command each, then again as 3 bytes in a single 24-bit transfer + 5 in 8-bit ones; the CRC-8
    of all 8 bytes must be 0. The device holds each 0 for only 15 us (the spec minimum), so this also
    proves the controller samples early enough."""
    rom = make_rom(0x28, (0x6B, 0x13, 0x9C, 0x00, 0xA4, 0x07))
    slave = OneWireSlave({0x33: rom}, us=US)
    bus, world, prog = await ow_setup(dut, slave)
    await bus.set_enable(0)
    r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(8, 0x33), None)] + [(ow_read(8), 24)] * 8)
    got = r[2:]
    assert got == rom, [hex(x) for x in got]
    assert crc8_maxim(got) == 0
    assert slave.cmds == [[0x33]]
    # same ROM again: 24-bit transfer (word >> 8), then five single bytes
    r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(8, 0x33), None), (ow_read(24), 8)]
                         + [(ow_read(8), 24)] * 5)
    assert r[2] == rom[0] | (rom[1] << 8) | (rom[2] << 16), hex(r[2])
    assert r[3:] == rom[3:], [hex(x) for x in r[3:]]
    assert await bus.rx_level(0) == 0, "exactly one RX word per command"
    assert not slave.violations, slave.violations


@cocotb.test()
async def test_onewire_partial_bit_counts(dut):
    """Transfers need not be bytes: 3 bits then 5 bits make one command byte at the device (LSB first
    across the two commands), and a 5-bit read lands in the top 5 bits of the RX word."""
    slave = OneWireSlave({0x5E: [0b10110]}, us=US)
    bus, world, prog = await ow_setup(dut, slave)
    await bus.set_enable(0)
    r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(3, 0b110), None), (ow_xfer(5, 0b01011), None),
                               (ow_read(5), 27)])
    assert slave.cmds == [[0x5E]], [hex(c) for c in slave.cmds[0]]
    assert r[3] == 0b10110, bin(r[3])


@cocotb.test()
async def test_onewire_read_hold_margin(dut):
    """How early may a device release a read-0 and still be read correctly? The spec guarantees 15 us;
    the controller samples at 13 us after the fall. Reports the window and requires only the
    guaranteed 15 us (and 14 us) to work."""
    ok = {}
    for hold in (10, 12, 13, 14, 15, 30):
        slave = OneWireSlave({0x33: [0x00, 0x00]}, us=US, read_hold=hold)
        bus, world, prog = await ow_setup(dut, slave)
        await bus.set_enable(0)
        r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(8, 0x33), None), (ow_read(16), 16)])
        ok[hold] = (r[2] == 0)
        world.devs.clear()
        await bus.set_enable(0, False)
        await bus.fifo_clear(0)
    dut._log.info("1-Wire read-0 hold (device releases N us after the fall): %s" % {
        k: ("ok" if v else "FAIL") for k, v in ok.items()})
    assert ok[14] and ok[15] and ok[30], ok


@cocotb.test()
async def test_onewire_clock_tolerance(dut):
    """One PIO tick should be 1 us. Sweep the fractional CLKDIV around 24 and report which tick lengths
    still do reset + presence + READ ROM (first two bytes) with no timing violation at the device.
    Limits seen by a typical device (presence 30..150 us): FAST end, CLKDIV ~22.15 (-7.7 %): the write-0
    low pulse (65 ticks) drops under the 60 us minimum; SLOW end, CLKDIV ~27.7 (+15 %): the read sample
    (13 ticks after the fall) passes the 15 us a device holds a 0. A worst-case device whose presence
    pulse is only guaranteed over 60..75 us would cut the slow end to ~26.5 (+10 %). The window must
    include +-5 %."""
    rom = make_rom()
    results = {}
    for d in (22.0, 22.8, 24.0, 25.2, 27.0, 28.0):
        div = (int(d), int(round((d - int(d)) * 256)))
        slave = OneWireSlave({0x33: rom}, us=US)
        bus, world, prog = await ow_setup(dut, slave, div=div)
        await bus.set_enable(0)
        try:
            r = await ow_session(bus, [(OW_RESET, 31), (ow_xfer(8, 0x33), None), (ow_read(16), 16)])
            good = (r[0] == 0 and r[2] == (rom[0] | rom[1] << 8) and slave.cmds == [[0x33]]
                    and not slave.violations)
        except TimeoutError:
            good = False
        results[d] = good
        world.devs.clear()
        await bus.set_enable(0, False)
        await bus.fifo_clear(0)
    dut._log.info("1-Wire clock tolerance (CLKDIV, 24.0 = exactly 1 us/tick): %s" % {
        "%.1f" % k: ("ok" if v else "FAIL") for k, v in results.items()})
    for d in (22.8, 24.0, 25.2):
        assert results[d], "must work at CLKDIV %.1f (%+.1f %%): %s" % (d, (d / 24 - 1) * 100, results)


@cocotb.test()
async def test_i2c_slave_rx_fast_data_change_after_scl_fall(dut):
    """A legal master may change SDA only ONE clock after SCL falls (t_HD;DAT is allowed to be tiny). The
    slave must sample on the RISING edge, so every bit pattern -- here fully alternating 0x55/0xAA/0x00/0xFF --
    still arrives intact.  (A slave that sampled just after the fall would read the NEXT bit.)"""
    m = I2cMaster([("start",), ("write", 0x84), ("write", 0x55), ("write", 0xAA),
                   ("write", 0x00), ("write", 0xFF), ("stop",)], period=96, hold=1)
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=4), m)
    got = await drain_rx(bus, 4, m)
    assert got == [0x55, 0xAA, 0x00, 0xFF], [hex(g) for g in got]
    assert m.acks == [0, 0, 0, 0, 0], m.acks


@cocotb.test()
async def test_i2c_slave_rx_stays_silent_during_other_devices_traffic(dut):
    """Bystander: another master talks to ANOTHER slave (0x55) with data full of zero bits, which pulls SDA
    low while SCL is high-or-rising.  Our slave must neither push anything, nor drive SDA/SCL, nor lose its
    place: START detection must really be 'SDA FALLS while SCL is high', not merely 'SDA is low'.  Its own
    transaction right afterwards must still work."""
    other = I2cSlave(addr=0x55)
    m = I2cMaster([("start",), ("write", 0xAA), ("write", 0x00), ("write", 0x00), ("write", 0x00),
                   ("write", 0xFF), ("write", 0x00), ("stop",),
                   ("start",), ("write", 0x84), ("write", 0x11), ("write", 0x22), ("stop",)])
    bus, world = await slave_setup(dut, I2cSlaveRx(0x42, write_bytes=2), m)
    world.devs.append(other)
    drove = []

    async def watch():                       # was the PIO ever driving the bus during the bystander phase?
        while len(m.acks) < 6:
            drove.append(world.mlow & 0x300)
            await ClockCycles(dut.clk, 1)
    cocotb.start_soon(watch())
    got = await drain_rx(bus, 2, m)
    assert not any(drove), "slave drove the bus during somebody else's transaction (%d cycles)" % sum(1 for d in drove if d)
    assert [b for k, b in other.rx if k == "D"] == [0, 0, 0, 0xFF, 0], other.rx
    assert got == [0x11, 0x22], "only OUR transaction may reach the FIFO: %s" % [hex(g) for g in got]


# ============================================================================= WS2812 repeat-colour program
# `ws2812_repeat.pio`: one pair of TX words (N-1, pixel) paints a run of N identical pixels.  Ticks of 3 clocks.
async def wsr_setup(dut, pull_thresh=24, reset_us=50.0):
    bus, world = await setup(dut, period_ns=WS_CLK_NS)
    prog = load_src("ws2812_repeat.pio")
    await bus.load_program(prog)
    await bus.sm_setup(0, prog, side_base=0, autopull=False, pull_thresh=pull_thresh,
                       out_right=False, div=(WS_TICK_CLK, 0))
    strip = Ws2812Strip(pin=0, clk_ns=WS_CLK_NS, reset_us=reset_us)
    world.devs.append(strip)
    await bus.write(R_PIN_OWN, 0b001)
    return bus, world, strip


def wsr_words(runs):
    """TX words of a list of (count, pixel_word) runs."""
    out = []
    for n, px in runs:
        out += [n - 1, px]
    return out


def wsr_check(strip, runs, nbits=24):
    """One frame holding exactly the runs; every HIGH pulse exact (3 ticks = 0, 7 ticks = 1) and every LOW pulse
    exactly the number of ticks documented in ws2812_repeat.pio (inside a pixel / between pixels / between runs)."""
    T = WS_TICK_CLK
    assert len(strip.frames) == 1 and not strip.cur, (len(strip.frames), len(strip.cur))
    frame = strip.frames[0]
    want_px = []
    for n, px in runs:
        want_px += [(px >> (32 - nbits)) & ((1 << nbits) - 1)] * n if nbits == 24 else [px] * n
    assert Ws2812Strip.pixels(frame, nbits=nbits) == want_px
    # position class of every bit
    cls, k = [], 0
    for n, _ in runs:
        for p in range(n):
            for b in range(nbits):
                if b < nbits - 1:
                    cls.append("in")
                else:
                    cls.append("run" if p == n - 1 else "px")
    low_ticks = {("in", 1): 3, ("in", 0): 7, ("px", 1): 4, ("px", 0): 8, ("run", 1): 9, ("run", 0): 12}
    for i, bit in enumerate(frame[:-1]):
        assert strip.high_clk[i] == (7 if bit else 3) * T, (i, bit, strip.high_clk[i])
        want = low_ticks[(cls[i], bit)] * T
        assert strip.low_clk[i] == want, "bit %d (%s, bit %d): low %d clocks, expected %d" % (
            i, cls[i], bit, strip.low_clk[i], want)
    assert strip.high_clk[len(frame) - 1] == (7 if frame[-1] else 3) * T
    assert strip.glitches == 0


@cocotb.test()
async def test_ws2812_repeat_single_run(dut):
    """One command, 60 identical pixels: a single frame of 60 equal pixels, no CPU feed after the two words;
    all pulses exact."""
    bus, world, strip = await wsr_setup(dut)
    runs = [(60, grb(0x12, 0xA5, 0x3C))]
    await bus.set_enable(0)
    for w in wsr_words(runs):
        await bus.tx_put(0, w)
    await ClockCycles(dut.clk, 60 * 24 * WS_BIT_CLK + 6 * 1200)
    wsr_check(strip, runs)


@cocotb.test()
async def test_ws2812_repeat_one_pixel_and_extremes(dut):
    """N = 1 (count word 0) and the all-zero / all-one pixels, three commands in one frame."""
    bus, world, strip = await wsr_setup(dut)
    runs = [(1, grb(0, 0, 0)), (2, grb(0xFF, 0xFF, 0xFF)), (1, grb(0x80, 0x01, 0xFE))]
    await bus.set_enable(0)
    await ws_feed(dut, bus, wsr_words(runs))
    await ClockCycles(dut.clk, 4 * 24 * WS_BIT_CLK + 6 * 1200)
    wsr_check(strip, runs)


@cocotb.test()
async def test_ws2812_repeat_runs_back_to_back(dut):
    """Three commands (20 red, 15 green, 10 blue) fed as FIFO room appears: ONE frame of 45 pixels, the strip
    never latches between runs, and the borders are exactly the documented 9 / 12 tick lows."""
    bus, world, strip = await wsr_setup(dut)
    runs = [(20, grb(0, 0xFF, 0)), (15, grb(0xFF, 0, 0)), (10, grb(0, 0, 0xFF))]
    await bus.set_enable(0)
    await ws_feed(dut, bus, wsr_words(runs))
    await ClockCycles(dut.clk, 45 * 24 * WS_BIT_CLK + 6 * 1200)
    wsr_check(strip, runs)


@cocotb.test()
async def test_ws2812_repeat_long_strip_without_feeding(dut):
    """The point of the program: 300 pixels (a 5 m strip) from ONE pair of words. The 4-deep FIFO would last 4
    pixels with the plain program; here the FIFO is empty from the second after the start and the frame still
    streams without a single gap longer than 1.5 us."""
    bus, world, strip = await wsr_setup(dut)
    runs = [(300, grb(0x20, 0x40, 0x60))]
    await bus.set_enable(0)
    for w in wsr_words(runs):
        await bus.tx_put(0, w)
    await ClockCycles(dut.clk, 20)
    assert await bus.tx_level(0) == 0, "FIFO must be empty while the strip is still being written"
    await ClockCycles(dut.clk, 300 * 24 * WS_BIT_CLK + 6 * 1200)
    wsr_check(strip, runs)
    assert max(strip.low_clk[:-1]) <= 12 * WS_TICK_CLK                 # 1.5 us


@cocotb.test()
async def test_ws2812_repeat_rgbw_and_latch_between_frames(dut):
    """Pull threshold 32 gives SK6812 RGBW pixels (the pixel word is all 32 bits), and an empty FIFO for longer than
    the reset time between two commands latches the frame: two frames of one run each."""
    bus, world, strip = await wsr_setup(dut, pull_thresh=32)
    a, b = (4, 0x11223344), (3, 0xF0E0D0C0)
    await bus.set_enable(0)
    for w in wsr_words([a]):
        await bus.tx_put(0, w)
    await ClockCycles(dut.clk, 4 * 32 * WS_BIT_CLK + 3000)           # frame 1 done and > 50 us of silence
    for w in wsr_words([b]):
        await bus.tx_put(0, w)
    await ClockCycles(dut.clk, 3 * 32 * WS_BIT_CLK + 3000)
    assert len(strip.frames) == 2, len(strip.frames)
    assert Ws2812Strip.pixels(strip.frames[0], nbits=32) == [a[1]] * a[0]
    assert Ws2812Strip.pixels(strip.frames[1], nbits=32) == [b[1]] * b[0]
