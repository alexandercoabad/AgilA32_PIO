# Feature #3: Variable SPI clock divider (ported from AgilA8)

Ported AgilA8's `SPI_CTRL` clock-divider field to AgilA32's shared QSPI
engine. In AgilA8 that field only ever gated a standalone generic SPI
peripheral (CS2), never flash/PSRAM, which always ran at a fixed
speed regardless of it. AgilA32 has no generic SPI peripheral yet (see
the porting roadmap's feature #4, not started), so `req_div_sel` here
gates `qspi_shared_engine.v`'s actual flash/PSRAM SCK rate directly
instead -- the thing AgilA8's own roadmap entry for this field says
it's really for: de-risking validation of the external memory path
against real hardware by starting slow and speeding up only once a
real device's timing has been confirmed safe.

The clock-divider FSM itself (`req_div_sel` port, `half_period_for()`
lookup, latch-at-accept-time `half_period_r`/`div_cnt`) came in
already complete. This change was mostly about wiring it up the rest
of the way and dealing with the fallout of its new, much slower reset
default.

## RTL

- **`src/mem.v`**: wired up what was missing -- `QSPI_CTRL` (`0xFB`)
  reset value (`2'd3`, slowest), read path, write path, header doc
  entry, and the `req_div_sel` connection into the
  `qspi_shared_engine` instantiation (previously declared but not
  connected to anything).

## Tests

- **`test/tb_qspi_clkdiv.v`** (new) -- the testbench
  `qspi_shared_engine.v`'s own header comment already refers to
  ("bit-for-bit identical cycle count per transaction, confirmed in
  test/tb_qspi_clkdiv.v"); `tb_qspi_engine.v` covers the engine's
  bit-level protocol/byte-order correctness at a single fixed speed
  (`req_div_sel` pinned to `2'd0` there), this file is specifically
  about the divider itself. Part 1 drives `qspi_shared_engine`
  directly: measures each `req_div_sel` setting's SCK half-period
  (via `pin_sck` edge timing) and whole-transaction cycle count (via
  `req_valid`-to-`req_ready` timing) against a golden formula derived
  from the engine's own structure (`130 + 128*(half_period-1)` clk
  cycles for a 64-bit word transaction), confirms `req_div_sel=0`
  reproduces the engine's original 130-cycle fixed timing exactly, and
  confirms a mid-flight `req_div_sel` change never perturbs a
  transaction already in progress (accept-time latching). Part 2
  drives `mem` directly (mirrors `tb_mem_ext.v`'s `do_access` style,
  through the real engine and two behavioral SPI RAM models) and
  confirms `QSPI_CTRL` resets to `2'd3`, reads back what's written,
  reaches the engine end-to-end in both directions (external-access
  wait-cycle counts match the same golden formula), and that the
  same mid-flight-immunity guarantee holds through the register path,
  not just `req_div_sel` directly.
- **`test/mem_extmem_test.v`, `test/tb_qspi_engine.v`**: explicitly
  wire `req_div_sel` (was about to float since these directly
  instantiate `qspi_shared_engine`) -- pinned to `2'd0` (fastest) so
  their existing timing-sensitive checks are unaffected.
- **`test/tb_check.v`**: fixed a regression the new slow reset default
  exposed -- the boot ROM self-test now takes ~10,340 cycles (was a
  few hundred) at `QSPI_CTRL`'s reset default, so scenario1/2's
  3000-cycle wait windows and scenario3's 200-cycle "wait past
  self-test" were too short. Bumped to 15000 cycles (measured margin),
  and the overall watchdog timeout from 5,000,000 to 10,000,000 (ps).
- **`test/tb_flash_handoff.v`, `test/tb_flash_paging.v`,
  `test/tb_st7789_driver.v`, `test/tb_ps2_reader.v`,
  `test/tb_ps2_ascii.v`, `test/tb_boot_timeout.v`, `test/tb_ebreak_halt.v`**:
  every flash-resident instruction fetch now runs at `QSPI_CTRL`'s
  slow reset default unless something speeds it up, which broke these
  testbenches' fixed cycle budgets. For the LCD/PS2 programs in
  particular, just multiplying their existing cycle budgets isn't
  practical (tens of thousands of loop iterations, each iteration's
  fetches all newly ~64x slower -- would balloon into billions of
  simulated cycles). Applied the same fix to all seven instead: each
  now deposits/forces `dut.u_mem.qspi_div_sel = 2'd0` (or
  `top_dut.u_mem.qspi_div_sel` in `tb_ebreak_halt.v`, which shares one
  `rst_n` between a bare-core Part 1 and a real-top-level Part 2)
  right after every reset -- standing in for "hardware whose real SPI
  timing has already been confirmed safe", exactly the real-world
  usage `QSPI_CTRL` is designed for -- rather than reworking the
  tightly-packed `PagedAsm` flash images to spend 2 of their scarce
  page-0 instruction slots on a software speedup, or applying
  `tb_check.v`'s boot-ROM-self-test-focused budget-bumping pattern
  everywhere at a much larger scale.
- **`test/Makefile`**: added `tb_qspi_clkdiv.v` to `standalone-tests`.

## Docs

- **`docs/info.md`**: added the `QSPI_CTRL` (`0xFB`) register-map
  entry and a new "Variable SPI clock divider" section (placed after
  "PSRAM_BANK (RAM B)", the other QSPI-engine-related section); added
  a `tb_qspi_clkdiv.v` paragraph to "How to test".
- **`README.md`**: added a "Variable SPI clock divider" Status bullet;
  bumped the test-suite counts (Fourteen -> Fifteen, thirteen ->
  fourteen standalone, 13 -> 14 standalone tests passing); added
  `tb_qspi_clkdiv.v` to the repo-layout `test/` listing; added the
  clock divider to the `make standalone-tests` comment block.
- **`test/README.md`**: bumped Thirteen -> Fourteen, added the
  variable-SPI-clock-divider description sentence, and appended the
  `tb_qspi_clkdiv.v` command block (with the same walkthrough-comment
  style as the other thirteen) to the standalone-tests listing.

**Known pre-existing staleness, not touched here:** same as
`CHANGES_feature2.md` flagged -- `docs/info.md`'s "## How to test"
section still opens with "Nine test suites," a count that was already
wrong before this change, and `README.md`'s repo-layout `test/`
listing still doesn't mention `tb_timer_pwm.v`, `tb_ebreak_halt.v`, or
`tb_boot_timeout.v` (only `tb_qspi_clkdiv.v`, added by this change,
appears there now). Fixing either is a bigger edit than this feature's
scope.

## Verification

- Full standalone suite (`iverilog`/`vvp`, run manually -- `make
  standalone-tests` itself doesn't work in this sandbox, unrelated to
  this feature: `cocotb-config` isn't installed here, and the
  Makefile's top-level `include $(shell cocotb-config
  -makefiles)/Makefile.sim` fails to parse regardless of target):
  **14/14 testbenches pass, 0 failures**, including the new
  `tb_qspi_clkdiv.v`'s 13/13 checks (8 in Part 1, 5 in Part 2).
- Full cocotb regression not re-run in this sandbox for the same
  reason; unaffected by this change in principle (the cocotb
  testbench drives the DUT the same way regardless of `QSPI_CTRL`'s
  reset value, just more slowly through the self-test) but genuinely
  **not yet confirmed** -- worth an explicit `make SIM=icarus` run
  somewhere `cocotb-config` is available before calling this feature
  fully verified.

## What's next

Per the prioritized roadmap:
4. General-purpose SPI peripheral (CS2)
5. `shared_ram` dual-port area merge
