"""test_pio_gl_sig.py -- the PIO *signature* test: a second post-layout test that covers what the smoke test does not.

test_pio_gl.py proves the CPU -> TX FIFO -> state machine -> RX FIFO -> CPU path on the netlist with one tiny program.
It never uses `in`, most `out` / `mov` / `jmp` forms, side-set, PINDIRS, autopush / autopull, the input synchroniser,
or the two open-drain pads (pins 8 and 9).  This test runs a fixed mix of those through two state machines and compares
everything that can be seen on the pins with golden values.  Pins only (ui_in / uo_out / uio), so it runs unchanged on
the RTL and on the gate-level netlist (`make GATES=yes`).

What is checked
  * uo_out[7:0] (all eight bits; PIN_OWN hands every pin to the PIO) goes through exactly the golden sequence:
    SM0 (pio/gl_sig0.pio) shows each result of  set / mov (plain, ~, ::) / in pins, x (left shift) / out pins, y
    (left shift) / pull / out exec / irq set rel / push / jmp x-- , plus four exec'd instructions from the CPU
    (mov pins ~y / isr / status, set pins, in y, jmp x!=y / pin / y-- taken, jmp !x not taken, wait gpio / pin;
    the `set pins` words after a taken jump are sentinels that would show up on the pins if the jump were not taken).
  * the two pads go through the golden (output-enable, output-value) sequence: SM1 (pio/gl_sig1.pio) pulls pad 8 / pad 9
    low with side-set PINDIRS (open-drain, pad released = pulled high by this testbench) and reads them back.
  * the RX words the CPU collected (SM0 push of ISR; SM1 autopush of the four pad reads = 0x93, then 0xC3, 0x5A taken
    from an autopull word) and the IRQ flags (flag 0 raised by SM0 and consumed by SM1's `wait 1 irq 0`; flag 1 raised
    by SM1's `irq set 0 rel`) are shown by the CPU on uo_out, one byte at a time, after PIN_OWN is cleared.

Golden values: worked out by hand from the instruction semantics (comments below) and confirmed on the RTL simulation.
The static input pattern is ui_in = 0x6A (pins 0-7 = 0,1,0,1,0,1,1,0 -> `in pins, 8` from IN_BASE 2 sees
pins 2..9 = 0xDA with both pads released and pulled up; jmp pin on pin 6 is taken).

Rebuild the image after changing the programs:
    cd tools && python3 build_pio_gl_sig.py && cp pio_gl_sig_flash_image.hex ../test/
"""
import os

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge

from test import reset_dut, speed_up_qspi, boot_send_program, safe_int, UIO_CS0, UIO_CS1, UIO_MOSI, UIO_MISO, UIO_SCK  # noqa: F401
from test_pio_gl import QspiDevice, read_hex_bytes

UI_PATTERN = 0x6A

# uo_out[7:0] after every change, from the first PIO write until the CPU's last display byte.
#   0x00  PIN_OWN set, nothing driven yet
#   0x15  set pins, 21 (pins 0-4 only)                0xEC  mov pins, ~x   (x = 19)
#   0xCA  out pins, 8 of 0xCAD80000 = bitrev(0x1B53)  0xD8  mov pins, y    (y = 0xD8)
#   0x27  exec: mov pins, ~y                          0x2A  exec: set pins, 10 (pins 5-7 keep 001)
#   0x53  exec: mov pins, isr (isr = 0x1B53)          (exec: in y, 4 -> isr = 0x1B538)
#   0xFF  exec: mov pins, status (RX level 0 < 1)     0xD7  exec: mov pins, y after jmp y-- (0xD8 -> 0xD7)
#   0xC5  exec: set pins, 5 (pins 5-7 keep 110)
#         (exec: jmp x!=y, jmp pin, jmp y-- taken and jmp !x not taken: a taken jump skips a `set pins` sentinel that
#          would show up here; wait gpio / wait pin already true: no stall)
# then the PIN_OWN release (0x00) and the CPU's display bytes:
#   0x38 0xB5 0x01 0x00   SM0 push: isr = 0x0001B538, low byte first
#   0x93 0xC3 0x5A        SM1 autopush words (4 pad reads, x, y)
#   0x02                  IRQ flags (flag 1 set, flag 0 consumed)
UO_GOLDEN = [0x00, 0x15, 0xEC, 0xCA, 0xD8, 0x27, 0x2A, 0x53, 0xFF, 0xD7, 0xC5,
             0x00, 0x38, 0xB5, 0x01, 0x00, 0x93, 0xC3, 0x5A, 0x02]

# pads 8 / 9: (uio_oe[5:4], uio_out[5:4]) after every change.  Before the hand-over both pads are driven high by the
# top level (3,3); owned with PINDIR 0 they float (0,0); then side-set PINDIRS 1, 2, 3, 0; PIN_OWN = 0 drives high again.
PAD_GOLDEN = [(3, 3), (0, 0), (1, 0), (2, 0), (3, 0), (0, 0), (3, 3)]


async def qspi_pmod_with_pads(dut, flash_image):
    """The QSPI Pmod (flash on CS0, PSRAM on CS1) plus the pull-ups on the two PIO pads (uio[4], uio[5]): a pad reads
    low only while the chip drives it low, otherwise high.  One coroutine owns every write to uio_in."""
    flash = QspiDevice(bytearray(flash_image), 0xFFFFFF, False)
    ram = QspiDevice(bytearray(256), 0xFF, True)
    while True:
        await FallingEdge(dut.clk)
        uio = safe_int(dut.uio_out.value)
        oe = safe_int(dut.uio_oe.value)
        cs0, cs1 = (uio >> UIO_CS0) & 1, (uio >> UIO_CS1) & 1
        sck, mosi = (uio >> UIO_SCK) & 1, (uio >> UIO_MOSI) & 1
        f, r = flash.step(cs0, sck, mosi), ram.step(cs1, sck, mosi)
        bit = f if cs0 == 0 and f is not None else r if cs1 == 0 and r is not None else None
        cur = safe_int(dut.uio_in.value)
        new = cur
        if cs0 == 1 and cs1 == 1:
            new &= ~(1 << UIO_MISO)
        elif bit is not None:
            new = (new & ~(1 << UIO_MISO)) | (bit << UIO_MISO)
        for p in (4, 5):
            low = ((oe >> p) & 1) and not ((uio >> p) & 1)
            new = (new & ~(1 << p)) | ((0 if low else 1) << p)
        if new != cur:
            dut.uio_in.value = new


@cocotb.test()
async def test_pio_postlayout_signature(dut):
    """CPU-driven PIO signature test through the pins only (see the module docstring)."""
    image = read_hex_bytes("pio_gl_sig_flash_image.hex")
    stub = read_hex_bytes("flash_handoff_stub.hex")
    cocotb.start_soon(Clock(dut.clk, 10, unit="us").start())
    dut.ena.value = 1
    dut.ui_in.value = UI_PATTERN
    dut.uio_in.value = 0x30                       # pads pulled up from the start
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_pmod_with_pads(dut, image))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)                            # a no-op on the gate-level netlist

    st = dict(uo=[], pads=[], clk=0, active=False)

    async def monitor():
        while True:
            await FallingEdge(dut.clk)
            st["clk"] += 1
            if not st["active"]:
                continue
            uo = safe_int(dut.uo_out.value)
            pad = ((safe_int(dut.uio_oe.value) >> 4) & 3, (safe_int(dut.uio_out.value) >> 4) & 3)
            if not st["uo"] or st["uo"][-1] != uo:
                st["uo"].append(uo)
            if not st["pads"] or st["pads"][-1] != pad:
                st["pads"].append(pad)

    cocotb.start_soon(monitor())

    await ClockCycles(dut.clk, 15000)             # let the boot ROM's self-test finish first
    st["active"] = True
    await boot_send_program(dut, stub)
    dut.ui_in.value = UI_PATTERN                  # the bootloader used ui_in; restore the static input pattern

    max_clocks = int(os.environ.get("GL_SIG_MAX_CLOCKS", "20000000"))
    waited = 0
    while waited < max_clocks:
        await ClockCycles(dut.clk, 100)
        waited += 100
        if st["uo"][-1:] == [UO_GOLDEN[-1]] and len(st["uo"]) >= len(UO_GOLDEN):
            break
    dut._log.info("signature complete after %d clocks of polling (%d clocks in all)", waited, st["clk"])
    await ClockCycles(dut.clk, 400)

    # the monitor starts before the boot hand-over, so drop anything recorded before PIN_OWN was set
    uo = st["uo"]
    start = max(i for i in range(len(uo)) if uo[i:i + 2] == [0x00, 0x15]) if any(uo[i:i + 2] == [0x00, 0x15] for i in range(len(uo))) else 0
    got = uo[start:]
    assert got == UO_GOLDEN, f"uo_out sequence {[hex(x) for x in got]}, golden {[hex(x) for x in UO_GOLDEN]}"
    pads = st["pads"]
    pstart = max((i for i in range(len(pads)) if pads[i:i + 2] == [(3, 3), (0, 0)]), default=0)
    assert pads[pstart:] == PAD_GOLDEN, f"pad (oe,out) sequence {pads[pstart:]}, golden {PAD_GOLDEN}"
