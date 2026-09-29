# SPDX-FileCopyrightText: © 2024 Tiny Tapeout
# SPDX-License-Identifier: Apache-2.0

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge, Timer


# uio pin mapping (see src/tt_um_agila32.v):
#   uio[0]=qspi_cs0  uio[1]=qspi_mosi  uio[2]=qspi_miso (input to chip)
#   uio[3]=qspi_sck  uio[6]=qspi_cs1   (used by the external RAM window)
UIO_CS0, UIO_MOSI, UIO_MISO, UIO_SCK, UIO_CS1 = 0, 1, 2, 3, 6

# ui_in pin mapping for the GPIO bootloader protocol (see docs/info.md
# and tools/build_boot_rom.py):
#   ui_in[0]=DATA  ui_in[1]=CLOCK  ui_in[2]=START
UI_DATA, UI_CLOCK, UI_START = 0, 1, 2

CMD_READ = 0x03
CMD_WRITE = 0x02
DEBUG = False


def safe_int(val):
    """Safely resolve cocotb logic values that may contain X or Z into integers."""
    try:
        return int(val)
    except (ValueError, TypeError):
        return 0


async def qspi_ram_slave(dut, corrupt_reads=False, log=None):
    """Behavioral single-line SPI RAM slave, driven by sampling
    dut.uio_out every system clock edge and comparing against the
    previous sample to detect sck/cs edges -- mirrors
    test/spi_ram_model.v's protocol (same CMD_READ/CMD_WRITE, 24-bit
    address, MSB-first cmd+addr, byte-at-a-time data), just implemented
    in Python instead of Verilog so it can run against the gate-level
    netlist via cocotb. Runs forever as a background task; persists
    written bytes for the lifetime of one test.

    corrupt_reads: if True, every read returns the bitwise complement
    of the stored byte instead of the real value -- used to prove the
    self-test's mismatch detection actually inspects the data.

    log: optional list. If provided, a dict is appended to it each
    time an address phase completes: {"we": bool, "addr": int}.
    """
    mem = bytearray(256)
    prev_sck = 0
    bitcnt = 0
    phase = 0  # 0=cmd, 1=addr, 2=data
    addr = 0
    addr_byte = 0
    we = False
    shift_in = 0
    cur_out_bit = 0

    while True:
        await FallingEdge(dut.clk)
        uio = safe_int(dut.uio_out.value)
        cs1 = (uio >> UIO_CS1) & 1
        sck = (uio >> UIO_SCK) & 1
        mosi = (uio >> UIO_MOSI) & 1

        if cs1 == 1:
            phase = 0
            bitcnt = 0
            addr_byte = 0
            dut.uio_in.value = safe_int(dut.uio_in.value) & ~(1 << UIO_MISO)
        elif prev_sck == 0 and sck == 1:
            shift_in = ((shift_in << 1) | mosi) & 0xFF
            bitcnt += 1
            if bitcnt == 8:
                bitcnt = 0
                if phase == 0:
                    we = (shift_in == CMD_WRITE)
                    phase = 1
                    if DEBUG:
                        print(f"  [slave] CMD byte = 0x{shift_in:02x} we={we}")
                elif phase == 1:
                    addr = ((addr << 8) | shift_in) & 0xFFFFFF
                    addr_byte += 1
                    if addr_byte == 3:
                        phase = 2
                        raw = mem[addr & 0xFF]
                        cur_out_bit = (~raw) & 0xFF if corrupt_reads else raw
                        if log is not None:
                            log.append({"we": we, "addr": addr & 0xFF})
                else:  # phase == 2, data
                    if we:
                        mem[addr & 0xFF] = shift_in
                        if DEBUG:
                            print(f"  [slave] WROTE mem[0x{addr&0xFF:02x}] = 0x{shift_in:02x}")
                    addr = (addr + 1) & 0xFFFFFF
                    raw = mem[addr & 0xFF]
                    cur_out_bit = (~raw) & 0xFF if corrupt_reads else raw
        elif prev_sck == 1 and sck == 0 and phase == 2 and not we:
            bit_idx = 7 - bitcnt
            bitval = (cur_out_bit >> bit_idx) & 1
            cur = safe_int(dut.uio_in.value)
            dut.uio_in.value = (cur & ~(1 << UIO_MISO)) | (bitval << UIO_MISO)

        prev_sck = sck


async def reset_dut(dut):
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)
    await ClockCycles(dut.clk, 1)


def speed_up_qspi(dut):
    """Forces QSPI_CTRL's clock divider to 2'd0 (fastest) right after
    reset -- QSPI_CTRL (0xFB) now resets to 2'd3 (sys_clk/128, the
    slowest setting -- see docs/info.md's "Variable SPI clock divider"
    section), so every self-test QSPI transaction the boot ROM runs
    takes ~64x longer than these tests' cycle budgets were written
    for. Standing in for "hardware whose real SPI timing has already
    been confirmed safe" (exactly QSPI_CTRL's intended real-world
    usage) rather than inflating every wait/timeout here by 64x, which
    is the same fix CHANGES_feature3.md already applied to the
    standalone Icarus testbenches (tb_check.v etc.) -- this closes the
    gap where test.py itself was missed.

    Gate-level netlists (this same test.py also runs under GATES=yes
    against a synthesized, normally-flattened netlist -- see
    .github/workflows/gds.yaml's gl_test job) don't preserve the
    `user_project.u_mem` hierarchy this force relies on as a walkable
    Python scope -- confirmed against a real gate-level run: the
    flattened netlist still contains the register (as literal escaped
    wire names `u_mem.qspi_div_sel[1]`/`[0]` directly inside
    `user_project`'s scope, visible in that run's waveform dump), but
    `dut.user_project.u_mem` isn't a nested scope to attribute-access
    into anymore, so the force raises AttributeError and this
    try/except turns it into a silent no-op. Poking the flattened
    wire name directly was considered and rejected: it depends on
    synthesis happening not to rename or optimize away that specific
    register, which isn't a safe assumption to build on across
    synthesis runs.

    So under GATES=yes this is a no-op, and GL runs genuinely execute
    at QSPI_CTRL's real, slow reset-default speed -- every cycle
    budget in this file (wait_for_first_led_write's default and every
    test that waits before or without it) is now sized to tolerate
    that directly, the same fix CHANGES_feature3.md already applied to
    the standalone Icarus testbenches, rather than depending on this
    force succeeding for correctness. RTL sim still benefits from the
    force where it works (self-test finishes in a few hundred cycles
    instead of ~10,340, so RTL runs finish faster within the same,
    now-larger budgets) -- it's a speed optimization there, not a
    correctness dependency anywhere anymore.
    """
    try:
        dut.user_project.u_mem.qspi_div_sel.value = 0
    except AttributeError:
        pass


async def wait_for_first_led_write(dut, max_cycles=15000):
    """Waits for the self-test execution to complete and write to uo_out.

    max_cycles' default of 15000 is sized for the case speed_up_qspi's
    own docstring flags as unverified: gate-level runs (GATES=yes)
    where the hierarchical force silently no-ops, leaving QSPI_CTRL at
    its real ~64x-slower reset default and pushing self-test out to
    ~10,340 cycles (same figure CHANGES_feature3.md measured for
    tb_check.v against the identical RTL) -- not the few hundred
    cycles self-test takes when the force actually lands under RTL
    sim. 15000 matches the margin already proven sufficient there.
    """
    for _ in range(max_cycles):
        await ClockCycles(dut.clk, 1)
        val = safe_int(dut.uo_out.value)
        if val != 0 and (val & 0x0F) != 0:
            return
    assert False, "counter never wrote anything -- self-test/boot prefix may be stuck"


# ---------------------------------------------------------------------
# GPIO bootloader helpers: DATA/CLOCK/START on ui_in[0:2] (see
# docs/info.md and tools/build_boot_rom.py's module docstring for the
# full protocol).
# ---------------------------------------------------------------------
BOOT_BIT_HOLD_CYCLES = 200


async def boot_send_bit(dut, bit):
    dut.ui_in.value = (safe_int(dut.ui_in.value) & ~((1 << UI_DATA) | (1 << UI_CLOCK))) | (bit << UI_DATA)
    await ClockCycles(dut.clk, BOOT_BIT_HOLD_CYCLES)
    dut.ui_in.value = safe_int(dut.ui_in.value) | (1 << UI_CLOCK)
    await ClockCycles(dut.clk, BOOT_BIT_HOLD_CYCLES)
    dut.ui_in.value = safe_int(dut.ui_in.value) & ~(1 << UI_CLOCK)
    await ClockCycles(dut.clk, BOOT_BIT_HOLD_CYCLES)


async def boot_send_byte(dut, byte):
    for i in range(7, -1, -1):
        await boot_send_bit(dut, (byte >> i) & 1)


async def boot_send_program(dut, program_bytes):
    """Asserts START, then streams a length-prefixed program over the
    DATA/CLOCK handshake. Caller is responsible for having already
    reset the DUT and waited past the self-test."""
    dut.ui_in.value = safe_int(dut.ui_in.value) | (1 << UI_START)
    await ClockCycles(dut.clk, 20)
    await boot_send_byte(dut, len(program_bytes))
    for b in program_bytes:
        await boot_send_byte(dut, b)


@cocotb.test()
async def test_counter_wraps(dut):
    """Checks uo_out[3:0] counts 0..15 and wraps, with ui_in held at 0."""
    dut._log.info("Start")

    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    last = safe_int(dut.uo_out.value) & 0x0F
    seen_values = {last}
    wrapped = False

    for _ in range(15000 + 56 * 20):
        await ClockCycles(dut.clk, 1)
        cur = safe_int(dut.uo_out.value) & 0x0F
        if cur != last:
            assert 0 <= cur <= 15, f"uo_out[3:0] left expected 0..15 range: {cur}"
            if cur < last:
                wrapped = True
            seen_values.add(cur)
            last = cur

    assert wrapped, "counter never wrapped from 15 back to 0 in the simulated window"
    assert seen_values == set(range(16)), (
        f"expected to see all values 0..15, saw: {sorted(seen_values)}"
    )


@cocotb.test()
async def test_reset_starts_from_zero(dut):
    """Immediately after reset, uo_out should read 0."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    await ClockCycles(dut.clk, 5)
    val = safe_int(dut.uo_out.value)
    assert val == 0, f"expected uo_out == 0 immediately after reset, got {val}"


@cocotb.test()
async def test_ui_in_upper_bits_do_not_affect_counter(dut):
    """Confirms ui_in[7:3] changing patterns do not disturb counter execution."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)

    last = safe_int(dut.uo_out.value) & 0x0F
    seen_values = {last}
    wrapped = False

    for i in range(15000 + 56 * 20):
        dut.ui_in.value = (i & 0x1F) << 3  # only touch bits [7:3]
        await ClockCycles(dut.clk, 1)
        cur = safe_int(dut.uo_out.value) & 0x0F
        if cur != last:
            assert 0 <= cur <= 15, f"uo_out[3:0] left expected 0..15 range: {cur}"
            if cur < last:
                wrapped = True
            seen_values.add(cur)
            last = cur

    assert wrapped, "counter never wrapped with ui_in[7:3] toggling"
    assert seen_values == set(range(16)), (
        f"expected to see all values 0..15 with ui_in[7:3] toggling, saw: {sorted(seen_values)}"
    )


@cocotb.test()
async def test_selftest_fails_without_pmod(dut):
    """With nothing driving MISO, self-test should set uo_out[7]."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    await reset_dut(dut)
    await wait_for_first_led_write(dut)

    assert (safe_int(dut.uo_out.value) >> 7) & 1 == 1, (
        "expected uo_out[7]=1 (self-test failed) with no QSPI slave attached"
    )

    seen_values = set()
    for _ in range(56 * 20):
        await ClockCycles(dut.clk, 1)
        val = safe_int(dut.uo_out.value)
        assert (val >> 7) & 1 == 1, "uo_out[7] should stay 1 once set"
        seen_values.add(val & 0x0F)
    assert seen_values == set(range(16)), (
        f"expected counter to still visit all 0..15, saw: {sorted(seen_values)}"
    )


@cocotb.test()
async def test_selftest_passes_with_pmod(dut):
    """With a QSPI RAM slave responding, self-test should pass."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)

    await wait_for_first_led_write(dut)

    # Wait up to 50 cycles for hardware bit flags to settle
    for _ in range(50):
        if ((safe_int(dut.uo_out.value) >> 7) & 1) == 0:
            break
        await ClockCycles(dut.clk, 1)

    assert (safe_int(dut.uo_out.value) >> 7) & 1 == 0, (
        f"expected uo_out[7]=0 (self-test passed) with a QSPI RAM slave attached, got uo_out={safe_int(dut.uo_out.value):#010b}"
    )


@cocotb.test()
async def test_selftest_detects_mismatch(dut):
    """Corrupted reads on SPI should set error flag uo_out[7]."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut, corrupt_reads=True))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)

    await wait_for_first_led_write(dut)

    assert (safe_int(dut.uo_out.value) >> 7) & 1 == 1, (
        "expected uo_out[7]=1 (self-test failed) when the QSPI slave echoes back corrupted data"
    )


@cocotb.test()
async def test_selftest_transaction_addresses_match(dut):
    """Log SPI address phases to confirm self-test writes then reads the same address."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    txns = []
    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut, log=txns))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)

    await wait_for_first_led_write(dut)

    assert len(txns) >= 2, f"expected at least 2 QSPI transactions before the loop starts, saw {txns}"
    write_txn, read_txn = txns[0], txns[1]
    assert write_txn["we"] is True, f"expected the first self-test transaction to be a write, got {write_txn}"
    assert read_txn["we"] is False, f"expected the second self-test transaction to be a read, got {read_txn}"
    assert write_txn["addr"] == read_txn["addr"], (
        f"self-test wrote to 0x{write_txn['addr']:02x} but read back from 0x{read_txn['addr']:02x}"
    )


@cocotb.test()
async def test_flash_cs_never_asserted(dut):
    """Flash chip-select uio[0] must remain unasserted during self-test."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    for _ in range(500 + 56 * 5):
        await ClockCycles(dut.clk, 1)
        cs0 = (safe_int(dut.uio_out.value) >> UIO_CS0) & 1
        assert cs0 == 1, "qspi_cs0 (uio[0]) went low during the boot ROM's own execution"


@cocotb.test()
async def test_uio_oe_is_constant(dut):
    """uio_oe should remain constant at 0b1111_1011."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1

    for _ in range(500 + 56 * 5):
        await ClockCycles(dut.clk, 1)
        oe = safe_int(dut.uio_oe.value)
        assert oe == 0b1111_1011, f"uio_oe changed to 0b{oe:08b}, expected constant 0b11111011"


@cocotb.test()
async def test_selftest_passes_again_after_soft_reset(dut):
    """Self-test should pass across consecutive soft resets."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut))
    await ClockCycles(dut.clk, 10)

    for attempt in (1, 2):
        dut.rst_n.value = 0
        await ClockCycles(dut.clk, 10)
        dut.rst_n.value = 1
        speed_up_qspi(dut)  # QSPI_CTRL resets to slow again on every reset

        await wait_for_first_led_write(dut)

        for _ in range(50):
            if ((safe_int(dut.uo_out.value) >> 7) & 1) == 0:
                break
            await ClockCycles(dut.clk, 1)

        assert (safe_int(dut.uo_out.value) >> 7) & 1 == 0, (
            f"expected uo_out[7]=0 (self-test passed) on boot attempt {attempt} after a soft reset, got uo_out={safe_int(dut.uo_out.value):#010b}"
        )


@cocotb.test()
async def test_bootloader_loads_and_runs_program(dut):
    """Bootload a small RV32I program over GPIO and verify execution output."""
    clock = Clock(dut.clk, 10, unit="us")
    cocotb.start_soon(clock.start())

    dut.ena.value = 1
    dut.ui_in.value = 0
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    cocotb.start_soon(qspi_ram_slave(dut))
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    speed_up_qspi(dut)

    # 15000 cycles (not the original 200) to clear self-test's real,
    # slow-QSPI_CTRL-default duration (~10,340 cycles -- see
    # wait_for_first_led_write) before asserting START. Asserting it
    # any earlier starts the bit-banged transfer before the boot ROM
    # has reached MAIN_LOOP's polling loop, desyncing the whole
    # protocol -- confirmed as the actual gate-level failure mode
    # (self-test hadn't finished, so the length/program bytes were
    # sampled against nothing, corrupting the load). This is well
    # under MAIN_LOOP's own ~32,256-cycle give-up timeout (measured
    # from when MAIN_LOOP itself starts, i.e. self-test's ~10,340 plus
    # the shift's own budget), so it's asserted while the boot ROM is
    # still genuinely listening, and once START is recognized MAIN_LOOP
    # is left for good -- the timeout can't fire mid-transfer no matter
    # how long the bit-banging afterward takes.
    await ClockCycles(dut.clk, 15000)

    program = bytes([
        0x93, 0x00, 0x50, 0x00,  # addi x1, x0, 5
        0x13, 0x01, 0x70, 0x00,  # addi x2, x0, 7
        0xb3, 0x81, 0x20, 0x00,  # add  x3, x1, x2
        0x23, 0x28, 0x30, 0x0e,  # sw   x3, 0xF0(x0)   -- LED_OUT
        0x6f, 0x00, 0x00, 0x00,  # jal  x0, 0          -- spin
    ])
    await boot_send_program(dut, program)

    await ClockCycles(dut.clk, 3000)

    res = safe_int(dut.uo_out.value)
    assert res & 0x0F == 12, (
        f"expected uo_out[3:0]=12 (5+7) after the bootloaded program ran, "
        f"got uo_out=0b{res:08b}"
    )
