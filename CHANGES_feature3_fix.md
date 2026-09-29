# Feature #3 fix: cocotb RTL test 3/11 -> 11/11

## Root cause

`docs/info.md`/`CHANGES_feature3.md` already documented the cause:
`QSPI_CTRL` (`0xFB`) now resets to `2'd3` (slowest, sys_clk/128)
instead of running fixed-fast, so every self-test QSPI transaction the
boot ROM runs now takes ~64x longer. The standalone Icarus
testbenches (`tb_check.v` and friends) were already patched to force
the divider back to fast right after reset — **but `test/test.py`
(the actual cocotb suite) was missed**, so 8 of its 11 tests were
timing out waiting on cycle budgets sized for the old fast-by-default
timing (self-test alone now takes ~10,340 cycles): the demo counter
never got a chance to run, `wait_for_first_led_write`'s 2000-cycle
budget wasn't enough, and the bootloader test's START pulse landed
before the boot ROM had even reached its polling loop.

## Fix

Added a `speed_up_qspi(dut)` helper to `test/test.py` that forces
`QSPI_CTRL`'s divider to `2'd0` (fastest) immediately after every
reset — the same fix already applied to the standalone testbenches,
just extended to cover this file too. Called it after all 9 reset
sites (`reset_dut()` plus 8 tests that reset the DUT manually instead
of calling it); the 2 tests that don't need it (they only watch
signals during the early portion of self-test, already passing) were
left untouched.

**Path correction along the way:** the hierarchical path is
`dut.user_project.u_mem.qspi_div_sel` — `dut` in this testbench is the
`tb` wrapper module, and `user_project` is `tb.v`'s fixed instance
name for the DUT (confirmed against `tb.v` itself), not `dut.u_mem`
directly.

**Gate-level safety:** this same `test.py` also runs against a
synthesized (normally flattened) netlist in CI's `gl_test` job
(`.github/workflows/gds.yaml`), which won't preserve the
`user_project.u_mem` hierarchy the force relies on. Wrapped the force
in `try/except AttributeError` so gate-level runs degrade to a silent
no-op instead of crashing — they'll fall back to running at
`QSPI_CTRL`'s real slow reset-default speed. **Not yet verified**
whether GL's own cycle budgets tolerate that; flagged as a follow-up
before calling feature #3 gate-level-clean.

## Verification

- **Full cocotb regression (`make SIM=icarus`): 11/11 tests pass**
  (was 3/11).
- **Full standalone suite (`make standalone-tests`): 14/14
  testbenches, 112 PASS assertions, 0 failures** — unaffected,
  re-run as a regression check.

## Still open

- Gate-level (`GATES=yes`) cocotb run not yet exercised against a real
  synthesized netlist in this sandbox (none present locally) — worth
  running once a `gate_level_netlist.v` is available, to confirm the
  no-op fallback's timing assumption actually holds.
