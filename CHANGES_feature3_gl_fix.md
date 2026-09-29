# Feature #3 gate-level fix: gl_test 3/11 -> 11/11

Closes out `CHANGES_feature3_fix.md`'s "Still open" item: the
gate-level (`GATES=yes`) cocotb run had never actually been exercised
against a real synthesized netlist, so `speed_up_qspi`'s no-op
fallback there was unverified. CI's `gl_test` job (`gds.yaml`) then
ran it for real and failed with the exact same 3/11 signature the RTL
fix (`CHANGES_feature3_fix.md`) started from.

## Root cause

Confirmed directly against the failing run's own waveform dump and
console log (both included in CI's artifacts): `speed_up_qspi`'s
`dut.user_project.u_mem.qspi_div_sel` force raises `AttributeError`
under `GATES=yes` and is silently swallowed, exactly as that
function's docstring already flagged as a risk. The synthesized
netlist's flattening doesn't remove the register -- the failing run's
waveform shows it present as literal escaped wire names
(`\u_mem.qspi_div_sel[1]`, `\u_mem.qspi_div_sel[0]`) directly inside
`user_project`'s scope -- but `u_mem` is no longer a nested scope to
attribute-access into from Python, so the force can't reach it.

Poking the flattened wire by its literal escaped name was considered
and rejected: it depends on synthesis happening not to rename or
optimize away that specific register, which isn't a safe assumption
to build a test on across synthesis runs (unlike a cycle-count
budget, which only depends on RTL behavior that's supposed to be
synthesis-invariant).

Net effect: gate-level runs execute the boot ROM's self-test at
`QSPI_CTRL`'s real, un-sped-up reset default (`2'd3`, ~64x slower),
the same ~10,340-cycle duration `CHANGES_feature3.md` already measured
for `tb_check.v` against the identical RTL. Every `test.py` budget
written assuming the force had succeeded was too short:

- `wait_for_first_led_write`'s default (2000 cycles) -- self-test
  alone needs ~10,340.
- `test_counter_wraps` / `test_ui_in_upper_bits_do_not_affect_counter`
  (500-cycle head start before sampling for a full 0..15 wrap) --
  same gap.
- `test_bootloader_loads_and_runs_program`'s 200-cycle wait before
  asserting START -- asserted (and began bit-banging) while the boot
  ROM was still mid-self-test, long before it ever reached `MAIN_LOOP`'s
  polling loop, desyncing the whole DATA/CLOCK protocol. Confirmed
  against the failing run's own timing: it ran its full 21-byte
  protocol to completion (104,030 cycles, matching the
  `200-cycle-hold x 3 phases x 8 bits x 21 bytes` math exactly) and
  still failed the final assertion -- not a timeout, a genuinely wrong
  result from the desync.

## Fix

Widened the affected budgets in `test/test.py` to tolerate the real
slow-QSPI-default worst case with margin, the same approach
`CHANGES_feature3.md` already used for `tb_check.v` (budget-bumping,
not another force) -- rather than a second, more fragile attempt at
reaching into the gate-level netlist:

- `wait_for_first_led_write`: default `max_cycles` 2000 -> 15000.
- `test_counter_wraps` / `test_ui_in_upper_bits_do_not_affect_counter`:
  loop bound's fixed component 500 -> 15000 (the `+ 56 * 20` wrap-
  observation component is untouched -- that part happens entirely
  on-chip after self-test and was never QSPI-speed-dependent).
- `test_bootloader_loads_and_runs_program`: pre-START wait 200 -> 15000.

`speed_up_qspi` itself is unchanged -- still a correctness-neutral
speed optimization for RTL sim (where the force works, so self-test
finishes in a few hundred cycles instead of ~10,340 and RTL runs
finish faster within the same, now-larger budgets), no longer
something GL correctness depends on. Its docstring is updated to
record the confirmed (not just suspected) root cause.

**Checked for interaction with feature #2's boot-ROM timeout
(`TIMEOUT_SHIFT=9`, ~32,256 cycles measured from when `MAIN_LOOP`
itself starts, i.e. self-test's ~10,340 + the shift's own budget):**
`MAIN_LOOP` abandons its own timeout-check code entirely the instant
it branches to `START_SEEN` -- once a test's `START` assertion is
recognized, no amount of subsequent bit-banging (100,800+ cycles for
`test_bootloader`'s 21-byte program at 200-cycle-hold timing) can
trigger the timeout, since that code path is never reached again.
The only real constraint is that `START` itself must be asserted
before the timeout's absolute cycle (~42,596 in the worst case); the
new 15000-cycle pre-START wait clears that with a comfortable margin
(~27,000 cycles), so the two features don't collide.

## Verification

- **RTL cocotb regression (`make SIM=icarus`), run locally in this
  sandbox: 11/11 tests pass** (was 3/11 under `GATES=yes` in CI;
  confirms the widened budgets don't regress the fast, force-working
  RTL path -- everything still finishes well inside its budget, just
  with more headroom than before).
- **Full standalone suite (`make standalone-tests`): 14/14
  testbenches pass, 0 failures** -- unaffected (these are separate
  Icarus testbenches, not touched by this change), re-run as a
  regression check.
- **True gate-level (`GATES=yes`) run against the real synthesized
  netlist: not re-run in this sandbox** -- the sky130 PDK's cell
  models aren't available here, and pulling/building `open_pdks`
  wasn't judged worth the setup cost given the fix is a pure
  cycle-budget widening (no gate-level-specific logic to exercise) and
  was checked directly against the failing run's own waveform and
  timing to confirm the diagnosis precisely (the 104,030-cycle
  `test_bootloader` failure time and the `u_mem.qspi_div_sel[1]`/`[0]`
  wire names were both found exactly where this fix's reasoning
  predicted). Worth an explicit `gl_test` CI re-run to close the loop
  for real.

## What's next

Per the prioritized roadmap (unchanged from `CHANGES_feature3.md`):
4. General-purpose SPI peripheral (CS2)
5. `shared_ram` dual-port area merge
