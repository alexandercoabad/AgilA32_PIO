"""test_pio_gl.py -- a small PIO test that also runs against the POST-LAYOUT (gate-level) netlist.

Everything here goes through the chip's pins only (ui_in / uo_out / uio), so it works the same on the RTL and on the
flattened gate-level netlist that .github/workflows/gds.yaml's gl_test job simulates (`make GATES=yes`):

  1. the GPIO bootloader loads the 4-byte FLASH_MODE hand-off stub (flash_handoff_stub.hex),
  2. the CPU runs tools/build_pio_gl_smoke.py's image from a simulated QSPI flash (served on CS0 below),
  3. that firmware loads two tiny PIO programs (pio/gl_echo.pio, pio/gl_pulses.pio), pushes three words through
     TX FIFO -> state machine -> RX FIFO and shows each result on uo_out[6:0]; SM1 answers every echo with a burst of
     four pulses on uo_out[7] (PIN_OWN hand-over, IRQ between the machines, CLKDIV 2, set pins, delays, jmp x--).

The test checks the three echoed words (bit_reverse(~word): FIFOs, mov with invert and bit-reverse, push/pull),
exactly 12 pulses of 4 clocks high / 10 clocks period, an idle-low pad before and after, and that uio_oe never moved.

RTL:  cd test && make                (runs this module together with test.py)
GL:   the tt-gds-action gl_test job runs `make GATES=yes` in test/ and so picks this module up too.
Rebuild the image after changing the programs:  cd tools && python3 build_pio_gl_smoke.py && cp pio_gl_smoke_flash_image.hex ../test/
"""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

from test import (qspi_ram_slave, reset_dut, speed_up_qspi, boot_send_program, safe_int,   # noqa: F401
                  UIO_CS0, UIO_CS1, UIO_MOSI, UIO_MISO, UIO_SCK)

HERE = os.path.dirname(os.path.abspath(__file__))
WORDS = [0x00000055, 0x00000001, 0x000000F0]            # keep in sync with tools/build_pio_gl_smoke.py


def bitrev32(v):
    return int("{:032b}".format(v)[::-1], 2)


def expected_byte(w):
    return (bitrev32(~w & 0xFFFFFFFF) >> 24) & 0x7F       # bit 7 of uo_out belongs to the PIO


def read_hex_bytes(name):
    return bytes(int(l.strip(), 16) for l in open(os.path.join(HERE, name)) if l.strip())


class QspiDevice:
    """One single-line SPI memory (CMD 0x03 read / 0x02 write, 24-bit address, MSB first), stepped once per clock
    like test.py's qspi_ram_slave.  `mem` is the contents; `mask` wraps the address (RAM: 0xFF); `writable`."""

    def __init__(self, mem, mask, writable):
        self.mem, self.mask, self.writable = mem, mask, writable
        self.prev_sck = 0
        self.reset()

    def reset(self):
        self.phase, self.bitcnt, self.addr_byte, self.addr = 0, 0, 0, 0
        self.we, self.shift_in, self.cur = False, 0, 0

    def byte_at(self, a):
        a &= self.mask
        return self.mem[a] if a < len(self.mem) else 0xFF

    def step(self, cs, sck, mosi):
        """Returns the MISO bit this device wants to drive after this clock, or None for 'leave it'."""
        out = None
        if cs == 1:
            self.reset()
        elif self.prev_sck == 0 and sck == 1:
            self.shift_in = ((self.shift_in << 1) | mosi) & 0xFF
            self.bitcnt += 1
            if self.bitcnt == 8:
                self.bitcnt = 0
                if self.phase == 0:
                    self.we = self.writable and self.shift_in == 0x02
                    self.phase = 1
                elif self.phase == 1:
                    self.addr = ((self.addr << 8) | self.shift_in) & 0xFFFFFF
                    self.addr_byte += 1
                    if self.addr_byte == 3:
                        self.phase = 2
                        self.cur = self.byte_at(self.addr)
                else:
                    if self.we:
                        self.mem[self.addr & self.mask] = self.shift_in
                    self.addr = (self.addr + 1) & 0xFFFFFF
                    self.cur = self.byte_at(self.addr)
        elif self.prev_sck == 1 and sck == 0 and self.phase == 2 and not self.we:
            out = (self.cur >> (7 - self.bitcnt)) & 1
        self.prev_sck = sck
        return out


async def qspi_pmod(dut, flash_image):
    """The QSPI Pmod: flash on CS0 (read-only image), PSRAM on CS1 (256 bytes), one shared MISO line."""
    flash = QspiDevice(bytearray(flash_image), 0xFFFFFF, False)
    ram = QspiDevice(bytearray(256), 0xFF, True)
    while True:
        await FallingEdge(dut.clk)
        uio = safe_int(dut.uio_out.value)
        cs0, cs1 = (uio >> UIO_CS0) & 1, (uio >> UIO_CS1) & 1
        sck, mosi = (uio >> UIO_SCK) & 1, (uio >> UIO_MOSI) & 1
        f, r = flash.step(cs0, sck, mosi), ram.step(cs1, sck, mosi)
        bit = f if cs0 == 0 and f is not None else r if cs1 == 0 and r is not None else None
        cur = safe_int(dut.uio_in.value)
        if cs0 == 1 and cs1 == 1:
            dut.uio_in.value = cur & ~(1 << UIO_MISO)
        elif bit is not None:
            dut.uio_in.value = (cur & ~(1 << UIO_MISO)) | (bit << UIO_MISO)


@cocotb.test()
async def test_pio_postlayout_echo_and_pulses(dut):
    """CPU-driven PIO smoke test through the pins only (see the module docstring)."""
    image = read_hex_bytes("pio_gl_smoke_flash_image.hex")
    stub = read_hex_bytes("flash_handoff_stub.hex")
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_pmod(dut, image))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)                       # a no-op on the gate-level netlist (see test.py): budgets below allow for it

    st = dict(seen=[], pulses=[], oe=set(), hi_since=None, level=0, clk=0, active=False, bad_idle=0)

    async def monitor():
        while True:
            await FallingEdge(dut.clk)
            st["clk"] += 1
            if not st["active"]:
                continue
            v = safe_int(dut.uo_out.value)
            st["oe"].add(safe_int(dut.uio_oe.value))
            lvl = (v >> 7) & 1
            if lvl and not st["level"]:
                st["hi_since"] = st["clk"]
            if st["level"] and not lvl:
                st["pulses"].append((st["hi_since"], st["clk"]))
            st["level"] = lvl
            low = v & 0x7F
            if not st["seen"] or st["seen"][-1] != low:
                st["seen"].append(low)

    cocotb.start_soon(monitor())

    # same start as test_bootloader_loads_and_runs_program: let the boot ROM's self-test finish first
    await ClockCycles(dut.clk, 15000)
    st["active"] = True
    await boot_send_program(dut, stub)

    want = [expected_byte(w) for w in WORDS]

    def done():
        s = st["seen"]
        it = iter(s)
        return all(any(x == w for x in it) for w in want)      # the three results appeared in order

    max_clocks = int(os.environ.get("GL_SMOKE_MAX_CLOCKS", "20000000"))   # default: enough for the slow GL QSPI
    waited = 0
    while waited < max_clocks:
        await ClockCycles(dut.clk, 100)
        waited += 100
        if done():
            break
    dut._log.info("echo results complete after %d clocks of polling", waited)
    assert done(), f"the three echoed words never appeared on uo_out[6:0]: wanted {[hex(w) for w in want]}, saw {[hex(x) for x in st['seen']]}"
    await ClockCycles(dut.clk, 400)                              # let the last burst finish

    # 1. echoed words in order (bit_reverse(~word), top byte, low 7 bits)
    s = st["seen"]
    it = iter(s)
    assert all(any(x == w for x in it) for w in want), f"echo results {[hex(x) for x in s]}, wanted {[hex(w) for w in want]} in order"

    # 2. exactly 3 bursts x 4 pulses; each high 4 clocks; period 10 inside a burst
    p = st["pulses"]
    assert len(p) == 12, f"expected 12 pulses on uo_out[7] (3 bursts of 4), got {len(p)}: {p}"
    widths = [f - r for r, f in p]
    assert widths == [4] * 12, f"pulse widths (clocks) {widths}, expected 4 each (set pins, 1 [1] at CLKDIV 2)"
    for b in range(3):
        rises = [p[4 * b + i][0] for i in range(4)]
        assert [rises[i + 1] - rises[i] for i in range(3)] == [10] * 3, f"burst {b} periods {[rises[i+1]-rises[i] for i in range(3)]}, expected 10 clocks"

    # 3. pad idles low after the last burst; PIO only owned uo_out[7]: uio_oe never changed
    assert st["level"] == 0, "uo_out[7] did not return low after the bursts"
    assert st["oe"] == {0b1111_1011}, f"uio_oe changed: {[bin(x) for x in st['oe']]}"
