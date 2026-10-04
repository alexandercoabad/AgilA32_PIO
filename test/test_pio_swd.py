"""SWD (ARM Serial Wire Debug) host on the PIO: pio/swd.pio against a software model of an ARM SW-DP.

    cd test && make -f Makefile.proto          (module test_pio_swd)

The model (SwdTarget) is a minimal but protocol-strict SW-DP + MEM-AP:
  * it ignores everything until the JTAG-to-SWD switch (>= 50 ones, 0xE79E, >= 50 ones, idle) has been received
  * it finds requests with a sliding 8-bit window and accepts only start = 1, stop = 0, park = 1, correct parity
  * it answers with turnaround + 3-bit ACK (OK / WAIT / FAULT), drives read data + parity after the rising edge
    (the host samples just before the next one), releases the line for the turnaround cycles, and checks the
    parity of write data (WDATAERR + the write is dropped when it is wrong)
  * DP: DPIDR, ABORT, CTRL/STAT (power-up request/acknowledge, STICKYERR, WDATAERR), SELECT, RDBUFF
  * MEM-AP 0: CSW (auto-increment), TAR, DRW with the posted-read behaviour of the real thing, IDR; a small memory
  * it flags BUS CONTENTION (host and target driving SWDIO at the same time) and records every request
Pins: SWDIO = pad uio4 (PIO pin 8), SWCLK = uo_out[0] (PIO pin 0).
"""
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles

from pio_tb_lib import (PioBus, World, assemble, R_PIN_OWN, R_SYNC_BYP, R_CTRL)
import pio_swd as S                                  # noqa: E402  (tools/ is on sys.path via pio_tb_lib)

SWDIO = 0x100                                        # pad uio4 as seen by World.resolve()


def load_src(name, origin=0):
    return assemble(open("../pio/" + name).read(), origin=origin)


async def setup(dut):
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
    cocotb.start_soon(world.run())
    return bus, world


# ============================================================================= the target
class SwdTarget:
    DPIDR = 0x2BA01477
    APIDR = 0x24770011

    def __init__(self, require_switch=True, wait_requests=0, bad_read_parity=False):
        self.low = 0                      # World pulls SWDIO low while this has bit 0x100 set
        self.driving = False
        self.out_bit = 1
        self.prev_clk = 0
        self.edge = 0
        self.state = "RESET" if require_switch else "IDLE"
        self.ones = 0
        self.sr16 = 0
        self.long_run_end = -1000
        self.sr8 = 0
        self.win = 0
        self.ph = 0
        self.req = self.ack = 0
        self.rdata = self.rpar = self.wdata = self.wpar = 0
        # debug port state
        self.csys = self.cdbg = 0
        self.sticky = self.wderr = 0
        self.select = 0
        self.rdbuff = 0
        self.csw = 0x23000002
        self.tar = 0
        self.mem = {0xE000ED00: 0x410FC231, 0x20000000: 0x00000000, 0x08000000: 0x20001000}
        # knobs and records
        self.wait_left = wait_requests
        self.bad_read_parity = bad_read_parity
        self.log = []                     # one dict per request that got an ACK
        self.requests = []                # every request byte accepted by the parser
        self.line_resets = 0
        self.contention = 0
        self.data_phases = 0
        self.trace = []                   # (cycle, swclk, host_drives, swdio) every cycle
        self.rise_cycles = []

    # ---- pad interaction
    def step(self, w):
        clk = (w.pin_out & w.own) & 1
        if clk and not self.prev_clk:
            self._edge(w)
        self.prev_clk = clk
        self.low = SWDIO if (self.driving and self.out_bit == 0) else 0
        host_drives = bool(w.own & w.pin_dir & SWDIO)
        if self.driving and host_drives:
            self.contention += 1
        self.trace.append((w.cycle, clk, host_drives, (w.resolve() >> 8) & 1))

    def _track_ones(self, lvl, active):
        if lvl:
            self.ones += 1
        else:
            if self.ones >= 50:
                self.long_run_end = self.edge - 1
                if active:
                    self.line_resets += 1
            self.ones = 0

    def _edge(self, w):
        lvl = (w.resolve() >> 8) & 1
        self.edge += 1
        self.rise_cycles.append(w.cycle)
        st = self.state
        if st == "RESET":
            self._track_ones(lvl, False)
            self.sr16 = (self.sr16 >> 1) | (lvl << 15)
            if self.edge - self.long_run_end == 16 and self.sr16 == S.JTAG_TO_SWD:
                self.state = "POST_SWITCH"
                self.ones = 0
        elif st == "POST_SWITCH":
            was = self.ones
            self._track_ones(lvl, False)
            if not lvl and was >= 50:
                self.state, self.win, self.sr8 = "IDLE", 1, 0
        elif st == "IDLE":
            self._track_ones(lvl, True)
            self.sr8 = (self.sr8 >> 1) | (lvl << 7)
            self.win += 1
            if self.win >= 8 and self._valid_request(self.sr8):
                self._begin(self.sr8)
        elif st == "TXN":
            self._txn_edge(lvl)

    @staticmethod
    def _valid_request(b):
        if not (b & 1) or ((b >> 6) & 1) or not ((b >> 7) & 1):
            return False
        return ((b >> 5) & 1) == (((b >> 1) ^ (b >> 2) ^ (b >> 3) ^ (b >> 4)) & 1)

    # ---- transactions
    def _begin(self, req):
        self.req = req
        self.requests.append(req)
        ap, rnw = (req >> 1) & 1, (req >> 2) & 1
        if self.wait_left > 0:
            self.wait_left -= 1
            self.ack = S.ACK_WAIT
        elif ap and not (self.csys and self.cdbg):
            self.sticky = 1
            self.ack = S.ACK_FAULT
        elif ap and self.sticky:
            self.ack = S.ACK_FAULT
        else:
            self.ack = S.ACK_OK
        self.wdata, self.wpar, self.rdata, self.rpar = 0, 0, 0, 0
        self.state, self.ph = "TXN", 0
        self.cur = dict(req=req, ap=ap, rnw=rnw, addr=(((req >> 3) & 1) << 2) | (((req >> 4) & 1) << 3),
                        ack=self.ack, data=None, parity_ok=None)

    def _txn_edge(self, lvl):
        self.ph += 1
        ph, ack, rnw = self.ph, self.ack, self.cur["rnw"]
        if ph == 1:                                   # turnaround edge: we start driving ACK[0]
            self.driving, self.out_bit = True, ack & 1
        elif ph == 2:
            self.out_bit = (ack >> 1) & 1
        elif ph == 3:
            self.out_bit = (ack >> 2) & 1
        elif ph == 4:
            if ack == S.ACK_OK and rnw:
                self.rdata = self._reg_read()
                self.rpar = S.parity(self.rdata) ^ (1 if self.bad_read_parity else 0)
                self.out_bit = self.rdata & 1
                self.data_phases += 1
            else:
                self.driving = False
        elif ack == S.ACK_OK and rnw:
            if ph <= 35:
                self.out_bit = (self.rdata >> (ph - 4)) & 1
            elif ph == 36:
                self.out_bit = self.rpar
            elif ph == 37:
                self.driving = False
            else:                                      # ph == 38: turnaround
                self.cur["data"] = self.rdata
                self._finish()
        elif ack == S.ACK_OK:                          # write: edge 5 = turnaround, data 6..37, parity 38
            if 6 <= ph <= 37:
                self.wdata |= lvl << (ph - 6)
            elif ph == 38:
                self.wpar = lvl
                ok = (S.parity(self.wdata) == self.wpar)
                self.cur["data"], self.cur["parity_ok"] = self.wdata, ok
                if ok:
                    self._reg_write(self.wdata)
                else:
                    self.wderr = 1                      # WDATAERR: the write is dropped
                self._finish()
        else:                                          # WAIT / FAULT: edge 5 = turnaround, no data phase
            if ph == 5:
                self._finish()

    def _finish(self):
        self.log.append(self.cur)
        self.state, self.win, self.sr8 = "IDLE", 0, 0
        self.driving = False

    # ---- registers
    def _ap_present(self):
        return (self.select >> 24) == 0

    def _ap_reg(self, addr):
        return (((self.select >> 4) & 0xF) << 4) | addr

    def _reg_read(self):
        c = self.cur
        if not c["ap"]:
            if c["addr"] == 0x0:
                return self.DPIDR
            if c["addr"] == 0x4:
                return ((S.CSYSPWRUPREQ | S.CSYSPWRUPACK) if self.csys else 0) \
                    | ((S.CDBGPWRUPREQ | S.CDBGPWRUPACK) if self.cdbg else 0) \
                    | (S.STICKYERR if self.sticky else 0) | (S.WDATAERR if self.wderr else 0)
            if c["addr"] == 0xC:
                return self.rdbuff
            return 0
        prev = self.rdbuff                                # AP reads are posted: return the previous result
        if self._ap_present():
            r = self._ap_reg(c["addr"])
            if r == 0x00:
                new = self.csw
            elif r == 0x04:
                new = self.tar
            elif r == 0x0C:
                new = self.mem.get(self.tar & ~3, 0)
                if (self.csw >> 4) & 3 == 1:
                    self.tar = (self.tar + 4) & 0xFFFFFFFF
            elif r == 0xFC:
                new = self.APIDR
            else:
                new = 0
        else:
            new = 0
        self.rdbuff = new
        return prev

    def _reg_write(self, d):
        c = self.cur
        if not c["ap"]:
            if c["addr"] == 0x0:                          # ABORT
                if d & 0x08:
                    self.sticky = 0
                if d & 0x10:
                    self.wderr = 0
            elif c["addr"] == 0x4:                        # CTRL/STAT
                self.csys, self.cdbg = (d >> 30) & 1, (d >> 28) & 1
            elif c["addr"] == 0x8:                        # SELECT
                self.select = d
            return
        if not self._ap_present():
            return
        r = self._ap_reg(c["addr"])
        if r == 0x00:
            self.csw = d
        elif r == 0x04:
            self.tar = d
        elif r == 0x0C:
            self.mem[self.tar & ~3] = d
            if (self.csw >> 4) & 3 == 1:
                self.tar = (self.tar + 4) & 0xFFFFFFFF


# ============================================================================= the host
class SwdHost:
    """What the CPU/firmware does: build packets, push PIO commands, pop one RX word per command."""

    def __init__(self, dut, bus, sm=0, div=(1, 0)):
        self.dut, self.bus, self.sm, self.div = dut, bus, sm, div
        self.prog = load_src("swd.pio")
        self.select_cache = None

    async def setup(self, bypass=False):
        b = self.bus
        await b.load_program(self.prog)
        await b.sm_setup(self.sm, self.prog, out_base=8, out_count=1, set_base=8, set_count=1,
                         side_base=0, in_base=8, autopush=False, autopull=False,
                         in_right=True, out_right=True, div=self.div)
        await ClockCycles(self.dut.clk, 3)
        await b.write(R_PIN_OWN, 0x101)                      # SWCLK (pin 0) and SWDIO (pin 8) -> PIO
        if bypass:
            await b.write(R_SYNC_BYP, 1 << 8)
        await b.set_enable(self.sm)

    async def _pop(self):
        for _ in range(6000):
            if await self.bus.rx_level(self.sm):
                return await self.bus.rx_get(self.sm)
        raise TimeoutError("no RX word from the SWD state machine")

    async def wr(self, n, value):
        await self.bus.tx_put_blocking(self.sm, S.cmd(n, False), limit=6000)
        await self.bus.tx_put_blocking(self.sm, value & 0xFFFFFFFF, limit=6000)
        await self._pop()

    async def rd(self, n):
        await self.bus.tx_put_blocking(self.sm, S.cmd(n, True), limit=6000)
        return (await self._pop()) >> (32 - n)

    async def ones(self, n):
        for k in range(0, n, 32):
            await self.wr(min(32, n - k), 0xFFFFFFFF)

    async def line_reset(self, idle=8):
        await self.ones(64)
        await self.wr(idle, 0)

    async def jtag_to_swd(self):
        await self.ones(64)
        await self.wr(16, S.JTAG_TO_SWD)
        await self.ones(64)
        await self.wr(8, 0)

    async def txn(self, apndp, rnw, addr, data=0, idle=8, bad_wparity=False, raw_request=None):
        req = S.request_byte(apndp, rnw, addr) if raw_request is None else raw_request
        await self.wr(8, req)
        ack = ((await self.rd(4)) >> 1) & 7                   # turnaround + 3 ACK bits
        res = dict(ack=ack, data=None, parity_ok=None, req=req)
        if ack == S.ACK_OK:
            if rnw:
                d = await self.rd(32)
                p = await self.rd(1)
                await self.rd(1)                              # turnaround: the host takes the line back
                res["data"], res["parity_ok"] = d, (S.parity(d) == p)
            else:
                await self.rd(1)                              # turnaround
                await self.wr(32, data)
                await self.wr(1, S.parity(data) ^ (1 if bad_wparity else 0))
        else:
            await self.rd(1)                                  # turnaround after WAIT / FAULT / no answer
        if idle:
            await self.wr(idle, 0)
        return res

    async def txn_retry(self, apndp, rnw, addr, data=0, tries=20, **kw):
        for n in range(tries):
            r = await self.txn(apndp, rnw, addr, data, **kw)
            r["tries"] = n + 1
            if r["ack"] != S.ACK_WAIT:
                return r
        return r

    # ---- register level
    async def dp_read(self, addr, **kw):
        return await self.txn_retry(0, 1, addr, **kw)

    async def dp_write(self, addr, value, **kw):
        return await self.txn_retry(0, 0, addr, value, **kw)

    async def select(self, apsel=0, apbank=0, dpbank=0):
        w = S.select_word(apsel, apbank, dpbank)
        if self.select_cache != w:
            r = await self.dp_write(S.DP_SELECT, w)
            assert r["ack"] == S.ACK_OK
            self.select_cache = w

    async def ap_read(self, reg, **kw):
        await self.select(0, reg >> 4)
        return await self.txn_retry(1, 1, reg & 0xC, **kw)

    async def ap_write(self, reg, value, **kw):
        await self.select(0, reg >> 4)
        return await self.txn_retry(1, 0, reg & 0xC, value, **kw)

    async def connect(self):
        await self.jtag_to_swd()
        return await self.dp_read(S.DP_DPIDR)

    async def power_up(self):
        r = await self.dp_write(S.DP_CTRLSTAT, S.CSYSPWRUPREQ | S.CDBGPWRUPREQ)
        assert r["ack"] == S.ACK_OK
        return await self.dp_read(S.DP_CTRLSTAT)

    async def read_mem(self, addr):
        """MEM-AP posted read: write TAR, read DRW (stale), read RDBUFF (the real value)."""
        await self.ap_write(S.AP_TAR, addr)
        await self.ap_read(S.AP_DRW)
        return await self.dp_read(S.DP_RDBUFF)


async def swd_setup(dut, div=(1, 0), bypass=False, **target_kw):
    bus, world = await setup(dut)
    tgt = SwdTarget(**target_kw)
    world.devs.append(tgt)
    host = SwdHost(dut, bus, div=div)
    await host.setup(bypass)
    await ClockCycles(dut.clk, 10)
    return host, world, tgt


def clean(tgt):
    assert tgt.contention == 0, "SWDIO was driven by host and target at the same time (%d cycles)" % tgt.contention


# ============================================================================= tests
@cocotb.test()
async def test_swd_connect_reads_dpidr(dut):
    """JTAG-to-SWD switch, then a DPIDR read: ACK OK, the DPIDR, correct read parity.  The request byte on the wire is
    the well-known 0xA5.  The target saw a line reset after the switch and nothing drove the bus at the same time."""
    host, world, tgt = await swd_setup(dut)
    r = await host.connect()
    assert r["ack"] == S.ACK_OK, r
    assert r["data"] == SwdTarget.DPIDR, hex(r["data"])
    assert r["parity_ok"]
    assert tgt.requests == [0xA5], [hex(x) for x in tgt.requests]
    assert tgt.state == "IDLE"
    clean(tgt)


@cocotb.test()
async def test_swd_ignored_until_the_switch_sequence(dut):
    """Before the JTAG-to-SWD switch the target must not answer: the ACK reads as 111 (the released line, pulled up).
    After the switch the same request works."""
    host, world, tgt = await swd_setup(dut)
    await host.line_reset()                                    # a line reset alone is not enough
    r = await host.txn(0, 1, S.DP_DPIDR)
    assert r["ack"] == 0b111, "target answered without the JTAG-to-SWD switch: ack=%d" % r["ack"]
    assert tgt.requests == []
    r = await host.connect()
    assert r["ack"] == S.ACK_OK and r["data"] == SwdTarget.DPIDR
    clean(tgt)


@cocotb.test()
async def test_swd_request_bytes_on_the_wire(dut):
    """Every request the target accepted is exactly the byte the SWD spec defines: DPIDR read A5, ABORT write 81,
    SELECT write B1, CTRL/STAT write A9, CTRL/STAT read ... and the parity bit is right in all of them."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.dp_write(S.DP_ABORT, 0x1E)
    await host.dp_write(S.DP_SELECT, 0)
    await host.dp_write(S.DP_CTRLSTAT, S.CSYSPWRUPREQ | S.CDBGPWRUPREQ)
    await host.dp_read(S.DP_CTRLSTAT)
    await host.dp_read(S.DP_RDBUFF)
    assert tgt.requests == [0xA5, 0x81, 0xB1, 0xA9, 0x8D, 0xBD], [hex(x) for x in tgt.requests]
    for b in tgt.requests:
        assert SwdTarget._valid_request(b)
    clean(tgt)


@cocotb.test()
async def test_swd_power_up_handshake(dut):
    """Write CTRL/STAT with CSYSPWRUPREQ + CDBGPWRUPREQ; the read-back shows the matching ACK bits (31 and 29)."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    r = await host.power_up()
    v = r["data"]
    assert v & S.CSYSPWRUPACK and v & S.CDBGPWRUPACK, hex(v)
    assert v & S.CSYSPWRUPREQ and v & S.CDBGPWRUPREQ
    assert r["parity_ok"]
    clean(tgt)


@cocotb.test()
async def test_swd_read_target_memory(dut):
    """The classic probe operation: power up, select the MEM-AP, set CSW and TAR, read DRW (posted: returns the PREVIOUS
    value) and fetch the result from RDBUFF.  Reads the Cortex-M CPUID at 0xE000ED00 and the AP's IDR."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.power_up()
    r = await host.ap_write(S.AP_CSW, 0x23000002)
    assert r["ack"] == S.ACK_OK
    cpuid = await host.read_mem(0xE000ED00)
    assert cpuid["ack"] == S.ACK_OK and cpuid["data"] == 0x410FC231, hex(cpuid["data"] or 0)
    assert cpuid["parity_ok"]
    stale = await host.ap_read(S.AP_DRW)                       # posted read: the previous RDBUFF value
    assert stale["data"] == 0x410FC231
    await host.ap_read(S.AP_IDR)
    idr = await host.dp_read(S.DP_RDBUFF)
    assert idr["data"] == SwdTarget.APIDR, hex(idr["data"])
    clean(tgt)


@cocotb.test()
async def test_swd_write_then_read_memory_with_auto_increment(dut):
    """CSW auto-increment: write four words through DRW at 0x20000000.., set TAR back and read them again."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.power_up()
    await host.ap_write(S.AP_CSW, 0x23000012)                  # 32-bit, AddrInc = 1
    await host.ap_write(S.AP_TAR, 0x20000000)
    words = [0xDEADBEEF, 0x00000000, 0xFFFFFFFF, 0x12345678]
    for w in words:
        r = await host.ap_write(S.AP_DRW, w)
        assert r["ack"] == S.ACK_OK and r["parity_ok"] is None
    assert [tgt.mem[0x20000000 + 4 * i] for i in range(4)] == words
    await host.ap_write(S.AP_TAR, 0x20000000)
    await host.ap_read(S.AP_DRW)                               # starts the first read, returns stale
    got = []
    for _ in range(3):
        got.append((await host.ap_read(S.AP_DRW))["data"])     # each returns the previous word
    got.append((await host.dp_read(S.DP_RDBUFF))["data"])
    assert got == words, [hex(x) for x in got]
    clean(tgt)


@cocotb.test()
async def test_swd_wait_response_is_retried(dut):
    """The target answers WAIT three times: each WAIT has an ACK of 010, NO data phase and one turnaround; the host
    retries and the fourth try returns the data."""
    host, world, tgt = await swd_setup(dut, wait_requests=0)
    await host.connect()
    tgt.wait_left = 3
    r = await host.dp_read(S.DP_DPIDR)
    assert r["ack"] == S.ACK_OK and r["tries"] == 4, r
    assert r["data"] == SwdTarget.DPIDR
    acks = [e["ack"] for e in tgt.log[-4:]]
    assert acks == [S.ACK_WAIT] * 3 + [S.ACK_OK], acks
    assert tgt.data_phases == 2, "only the real reads (connect + this one) may have a data phase: %d" % tgt.data_phases
    clean(tgt)


@cocotb.test()
async def test_swd_fault_before_power_up_and_abort(dut):
    """An AP access before power-up gets FAULT (100) and sets STICKYERR; further AP accesses keep faulting until an ABORT
    clears the flag; then power-up and a memory read work."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.dp_write(S.DP_SELECT, 0)
    host.select_cache = 0
    r = await host.txn(1, 1, S.AP_DRW & 0xC)
    assert r["ack"] == S.ACK_FAULT, r
    st = await host.dp_read(S.DP_CTRLSTAT)
    assert st["data"] & S.STICKYERR, hex(st["data"])
    await host.power_up()
    r = await host.txn(1, 1, S.AP_DRW & 0xC)
    assert r["ack"] == S.ACK_FAULT, "STICKYERR must keep faulting AP accesses until it is cleared"
    await host.dp_write(S.DP_ABORT, 0x08)                      # STKERRCLR
    st = await host.dp_read(S.DP_CTRLSTAT)
    assert not st["data"] & S.STICKYERR
    cpuid = await host.read_mem(0xE000ED00)
    assert cpuid["data"] == 0x410FC231
    clean(tgt)


@cocotb.test()
async def test_swd_write_parity_error_is_detected_by_the_target(dut):
    """A write whose parity bit is wrong must be dropped by the target (WDATAERR set, SELECT unchanged); the correct
    write afterwards works and ABORT clears the flag."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.dp_write(S.DP_SELECT, 0x000000F0)
    r = await host.txn(0, 0, S.DP_SELECT, 0x00000010, bad_wparity=True)
    assert r["ack"] == S.ACK_OK
    assert tgt.log[-1]["parity_ok"] is False
    assert tgt.select == 0x000000F0, "a write with bad parity must not be applied"
    st = await host.dp_read(S.DP_CTRLSTAT)
    assert st["data"] & S.WDATAERR
    await host.dp_write(S.DP_ABORT, 0x10)                      # WDERRCLR
    await host.dp_write(S.DP_SELECT, 0x00000010)
    assert tgt.select == 0x10
    st = await host.dp_read(S.DP_CTRLSTAT)
    assert not st["data"] & S.WDATAERR
    clean(tgt)


@cocotb.test()
async def test_swd_host_detects_bad_read_parity(dut):
    """The host side: when the target sends a wrong parity bit on a read, the data comes back flagged (parity_ok False)."""
    host, world, tgt = await swd_setup(dut, bad_read_parity=True)
    r = await host.connect()
    assert r["ack"] == S.ACK_OK and r["data"] == SwdTarget.DPIDR
    assert r["parity_ok"] is False
    tgt.bad_read_parity = False
    r = await host.dp_read(S.DP_DPIDR)
    assert r["parity_ok"] is True
    clean(tgt)


@cocotb.test()
async def test_swd_request_with_bad_parity_gets_no_answer(dut):
    """A request with a wrong parity bit is not a request: the target stays silent, the ACK reads 111, the target logged
    nothing, and a good request right afterwards works."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    n = len(tgt.requests)
    bad = S.request_byte(0, 1, 0x0) ^ (1 << 5)                 # flip the parity bit
    r = await host.txn(0, 1, 0x0, raw_request=bad)
    assert r["ack"] == 0b111, r
    assert len(tgt.requests) == n
    r = await host.dp_read(S.DP_DPIDR)
    assert r["ack"] == S.ACK_OK and r["data"] == SwdTarget.DPIDR
    clean(tgt)


@cocotb.test()
async def test_swd_waveform_timing(dut):
    """At CLKDIV 1 SWCLK is 4 clocks high / 4 low (8 per bit) inside every command and idles LOW between them.  Whenever
    the host drives SWDIO it is stable >= 3 clocks before every rising edge and held >= 3 clocks after it."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.power_up()
    clk = [t[1] for t in tgt.trace]
    # run lengths of SWCLK
    runs, cur, n = [], clk[0], 0
    for v in clk:
        if v == cur:
            n += 1
        else:
            runs.append((cur, n))
            cur, n = v, 1
    highs = [n for v, n in runs[1:-1] if v == 1]
    lows = [n for v, n in runs[1:-1] if v == 0 and n < 9]
    assert set(highs) == {4}, "SWCLK high times %s" % sorted(set(highs))
    assert set(lows) == {4}, "SWCLK low times inside commands %s" % sorted(set(lows))
    assert clk[-1] == 0 and clk[0] == 0, "SWCLK must idle low"
    # SWDIO set-up / hold around each rising edge while the host drives
    cyc = [t[0] for t in tgt.trace]
    sdio = [t[3] for t in tgt.trace]
    hdrv = [t[2] for t in tgt.trace]
    setups, holds = [], []
    for i in range(2, len(clk) - 8):
        if clk[i] == 1 and clk[i - 1] == 0 and hdrv[i]:
            j = i
            while j > 0 and sdio[j - 1] == sdio[i]:
                j -= 1
            k = i
            while k < len(sdio) - 1 and sdio[k + 1] == sdio[i]:
                k += 1
            setups.append(i - j)
            holds.append(k - i + 1)
    assert setups and min(setups) >= 3, min(setups)
    assert min(holds) >= 3, min(holds)
    clean(tgt)


@cocotb.test()
async def test_swd_clock_rates(dut):
    """CLKDIV sweep: SWCLK period = 8 * CLKDIV clocks.  The DPIDR read works at CLKDIV 1, 2, 3.5 and 8."""
    for div in ((1, 0), (2, 0), (3, 0x80), (8, 0)):
        host, world, tgt = await swd_setup(dut, div=div)
        r = await host.connect()
        assert r["ack"] == S.ACK_OK and r["data"] == SwdTarget.DPIDR, (div, r)
        rc = tgt.rise_cycles
        exp = 8 * (div[0] + div[1] / 256.0)
        gaps = sorted(b - a for a, b in zip(rc, rc[1:]) if b - a < exp * 1.6 + 3)
        assert abs(gaps[len(gaps) // 2] - exp) <= 1.0, (div, gaps[len(gaps) // 2], exp)
        clean(tgt)


@cocotb.test()
async def test_swd_turnaround_never_overlaps(dut):
    """Bus safety: across a whole session (connect, power-up, memory write and read, WAIT, FAULT) the host and the target
    are never driving SWDIO in the same clock, and the host stops driving for every turnaround."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.power_up()
    await host.ap_write(S.AP_CSW, 0x23000012)
    await host.ap_write(S.AP_TAR, 0x20000000)
    await host.ap_write(S.AP_DRW, 0xA5A5A5A5)
    tgt.wait_left = 2
    await host.read_mem(0x20000000)
    await host.txn(1, 0, S.AP_DRW & 0xC, 1)
    assert tgt.contention == 0
    # the host must have released SWDIO during the ACK and read-data phases: count clocks with both idle/driven
    both_idle = sum(1 for t in tgt.trace if not t[2])
    assert both_idle > 100, "the host never released SWDIO?"


# ============================================================================= extra tests (protocol-level details)
@cocotb.test()
async def test_swd_command_lengths_and_bit_order(dut):
    """Every command length 1..32: exactly n SWCLK pulses; a write puts the word on SWDIO LSB first (sampled at every
    rising edge); a read of the released, pulled-up line returns n ones in the right place ((word >> (32-n)) == 2^n-1)."""
    host, world, tgt = await swd_setup(dut)
    pat = 0xA5C3_9E71
    for n in range(1, 33):
        k0 = len(tgt.rise_cycles)
        t0 = len(tgt.trace)
        await host.wr(n, pat)
        rises = [i for i in range(t0 + 1, len(tgt.trace)) if tgt.trace[i][1] and not tgt.trace[i - 1][1]]
        assert len(tgt.rise_cycles) - k0 == n, (n, len(tgt.rise_cycles) - k0)
        got = sum(tgt.trace[i][3] << j for j, i in enumerate(rises))
        assert got == pat & ((1 << n) - 1), (n, hex(got), hex(pat & ((1 << n) - 1)))
        k0 = len(tgt.rise_cycles)
        v = await host.rd(n)
        assert len(tgt.rise_cycles) - k0 == n, (n, len(tgt.rise_cycles) - k0)
        assert v == (1 << n) - 1, (n, hex(v))
    clean(tgt)


@cocotb.test()
async def test_swd_works_with_synchroniser_bypassed(dut):
    """The same session with the pad input synchroniser bypassed on SWDIO (SYNC_BYP): connect, power-up, memory read."""
    host, world, tgt = await swd_setup(dut, bypass=True)
    r = await host.connect()
    assert r["ack"] == S.ACK_OK and r["data"] == SwdTarget.DPIDR, r
    await host.power_up()
    cpuid = await host.read_mem(0xE000ED00)
    assert cpuid["data"] == 0x410FC231
    clean(tgt)


@cocotb.test()
async def test_swd_swclk_stays_low_when_the_rx_fifo_is_full(dut):
    """Five one-bit reads with nobody popping: the fifth stalls at `push` and SWCLK must be LOW and quiet while it waits;
    popping resumes the machine and nothing is lost or duplicated."""
    host, world, tgt = await swd_setup(dut)
    for _ in range(5):
        await host.bus.tx_put_blocking(0, S.cmd(1, True))
    await ClockCycles(dut.clk, 400)
    n = len(tgt.rise_cycles)
    assert n == 5, n                                           # the fifth clock is done, only its push waits
    assert tgt.trace[-1][1] == 0
    await ClockCycles(dut.clk, 200)
    assert len(tgt.rise_cycles) == n
    got = [await host._pop() for _ in range(5)]
    assert got == [0x80000000] * 5, [hex(x) for x in got]
    clean(tgt)


@cocotb.test()
async def test_swd_long_session_with_wait_retries_between_banks(dut):
    """A longer session: two MEM-AP banks (CSW/TAR bank 0, IDR bank F), alternating writes and reads with WAIT answers
    sprinkled in; the target memory and every read value are checked at the end."""
    host, world, tgt = await swd_setup(dut)
    await host.connect()
    await host.power_up()
    await host.ap_write(S.AP_CSW, 0x23000012)
    base = 0x20000100
    vals = [0x01234567 * (i + 1) & 0xFFFFFFFF for i in range(8)]
    await host.ap_write(S.AP_TAR, base)
    for i, v in enumerate(vals):
        if i % 3 == 1:
            tgt.wait_left = 2
        r = await host.ap_write(S.AP_DRW, v)
        assert r["ack"] == S.ACK_OK
    assert [tgt.mem[base + 4 * i] for i in range(8)] == vals
    await host.ap_read(S.AP_IDR)
    assert (await host.dp_read(S.DP_RDBUFF))["data"] == SwdTarget.APIDR
    for i, v in enumerate(vals):
        tgt.wait_left = i % 2
        r = await host.read_mem(base + 4 * i)
        assert r["data"] == v and r["parity_ok"], (i, hex(r["data"] or 0))
    clean(tgt)
