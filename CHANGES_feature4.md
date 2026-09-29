# Feature #4: General-purpose SPI peripheral (CS2), ported from AgilA8

Ported AgilA8's `spi_ctrl.v`/`qspi_shared_engine.v` CS2 front-end: a
raw 8-bit SPI transfer with no command/address framing at all, for
talking to whatever non-flash/PSRAM SPI device a board has wired to
CS2 -- an ADC, another MCU, an LCD controller not already covered by
a dedicated driver, etc.

Started from a WIP `qspi_shared_engine.v` whose header comments
already laid out the plan (`req_dev==2'd0` = generic SPI peripheral,
sharing CS2 with PSRAM "RAM B"; `SPI_DATA` at mem.v's `0xFD`) but
whose actual RTL body hadn't been touched yet -- this change is that
implementation.

## RTL

- **`src/qspi_shared_engine.v`**: `req_dev==2'd0`'s CS2 mux entry;
  `ST_IDLE`'s framing branches to a raw 8-bit transfer for
  `req_dev==2'd0` instead of calling `build_preload` (no cmd/addr
  phase at all -- the byte is left-justified directly into `sreg`'s
  top 8 bits, `nbits_total`/`data_bits` both hardcoded to 8,
  `req_size` ignored). The existing `data_bits==8` extraction in
  `ST_DONE` already handles the received byte correctly -- no new
  code needed there; it's the same path a byte-sized flash/PSRAM read
  already uses.
- **`src/mem.v`**: `SPI_DATA` (`0xFD`) with AgilA8's spi_ctrl.v split
  faithfully ported -- a WRITE (`in_spi_write`) is the only access
  that reaches the engine at all (`in_ext` widened to include it,
  `req_dev` mux extended); a plain READ bypasses the engine entirely
  and returns a new `spi_last_rx` register instead, latched from
  `ext_rdata` the cycle `ext_ready` pulses for a completed SPI_DATA
  write (confirmed safe against the core's own wait-state protocol --
  `addr`/`we`/`valid` are held constant by `rv32i_core.v`'s
  `ST_MEM_WAIT` for the whole transaction, only clearing the same
  cycle `ready` pulses, not before). Added the register-map doc entry
  and a "Generic SPI peripheral" prose section (placed after
  "Bank-switched flash execution", next to `WINDOW_BYTES`).

No changes needed to `rv32i_core.v` or the address decode outside
`mem.v` -- `SPI_DATA` is just another byte-addressable register from
the core's perspective, same as every other peripheral on this bus.

## Tests

- **`test/tb_spi_periph.v`** (new) -- Part 1 drives
  `qspi_shared_engine` directly: confirms a raw byte gets exactly 8
  bits on the wire (not 40 -- proving it didn't fall through to the
  flash/PSRAM framing by mistake), MOSI-first, matching `req_wdata`
  directly, with only CS2 asserted; and that a write's simultaneously-
  captured MISO byte round-trips correctly through the same byte-read
  extraction path flash/PSRAM already use. Part 2 drives `mem`
  directly (mirroring `tb_mem_ext.v`'s `do_access` style) with a
  standing inline "echo slave" process on `qspi_miso` (not
  `spi_ram_model.v` -- that model speaks the flash/PSRAM cmd/addr/data
  protocol and would misinterpret a raw byte as a command byte and
  hang waiting for address bytes that never come) and confirms:
  `SPI_DATA` resets to `0x00`; a plain read never touches hardware at
  all (zero wait cycles, zero SCK pulses -- the actual point of this
  port, ported from AgilA8's "plain read returns the last byte, no
  retrigger" behavior); a write is a real 8-SCK-pulse transfer under
  CS2 that leaves flash (CS0) and PSRAM RAM A (CS1) untouched; and
  repeated reads after a write keep returning the same captured byte
  for free.
  - Hit and fixed two testbench-only timing bugs while writing this
    (not RTL bugs): a `@(negedge pin_sck)`-first MISO-driving task
    silently skips bit 0 when nothing has consumed a prior `posedge`
    first (SCK starts idle-low, so the first "negedge" doesn't fire
    until AFTER bit 0's own sampling edge has already passed) --
    that's why every existing engine-level MISO-driving task in
    `tb_qspi_engine.v` always calls a `posedge`-based capture task
    first to consume the cmd/addr phase before ever calling
    `drive_miso_bits`, which this test's raw byte-only transfer has no
    equivalent phase for. Fixed with a new
    `drive_miso_bits_from_start` task (Part 1) and by switching the
    Part 2 echo slave's bit-0 drive to trigger on `negedge qspi_cs2`
    (assertion) instead of waiting for the first `qspi_sck` edge.
- **`test/Makefile`**: added `tb_spi_periph.v` to `standalone-tests`;
  updated the file's own header comment block to mention the clock
  divider and generic SPI peripheral (previously listed only through
  feature #2's boot-timeout fallback, already stale before this
  change in a smaller way than the `docs/info.md`/`README.md`
  staleness flagged below).

## Docs

- **`docs/info.md`**: added the `SPI_DATA` (`0xFD`) register-map
  entry; corrected the "Variable SPI clock divider" section's now-
  inaccurate "this core has no generic SPI peripheral yet" framing;
  added a new "Generic SPI peripheral" section (placed right after
  it); added a `tb_spi_periph.v` paragraph to "How to test".
- **`README.md`**: added a "Generic SPI peripheral (CS2)" Status
  bullet; bumped the test-suite counts (Fifteen -> Sixteen, fourteen
  -> fifteen standalone, 14 -> 15 standalone tests passing); added
  `tb_spi_periph.v` to the repo-layout `test/` listing; added the
  generic SPI peripheral to the `make standalone-tests` comment.
- **`test/README.md`**: bumped Fourteen -> Fifteen, added the generic-
  SPI-peripheral description sentence, and appended the
  `tb_spi_periph.v` command block (same walkthrough-comment style as
  the other fourteen) to the standalone-tests listing.

**Known pre-existing staleness, not touched here:** same as every
prior `CHANGES_featureN.md` has flagged -- `docs/info.md`'s "## How to
test" section still opens with "Nine test suites."

## Verification

- Zero-regression check *before* writing any feature-4-specific test:
  full standalone suite (14/14 testbenches, 112 PASS assertions) and
  full cocotb regression (11/11) both re-run against the RTL changes
  alone, confirming `req_dev==2'd0` doesn't disturb flash/PSRAM/RAM-B
  behavior at all.
- Full standalone suite with `tb_spi_periph.v` added: **15/15
  testbenches pass, 124 PASS assertions, 0 failures**.
- Full cocotb regression (`make SIM=icarus`): **11/11 tests pass**,
  re-run as a final regression check (unaffected by this change in
  principle -- the generic SPI peripheral isn't touched by the boot
  ROM or the demo/bootload paths at all).

## What's next

Per the prioritized roadmap:
5. `shared_ram` dual-port area merge
