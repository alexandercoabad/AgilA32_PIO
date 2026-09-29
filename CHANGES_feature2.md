# Feature #2: Boot-timeout -> automatic flash fallback (ported from AgilA8)

Ported AgilA8's boot_rom behavior of giving up on an unbounded wait for
a bootload and falling back to external flash after a fixed number of
iterations, so an unattended AgilA32 chip still boots something useful
instead of listening for a host forever.

Entirely self-contained to `tools/build_boot_rom.py`'s own listen
loop -- no RTL changes were needed. `mem.v`'s `FLASH_MODE`/`LOAD_BASE`
mechanics already existed (built for the bootloaded-stub handoff path),
so the boot ROM just needed to call the same trigger itself.

## Tooling

- **`tools/build_boot_rom.py`** -- `MAIN_LOOP`'s free-running demo
  counter (`x9`) now doubles as the timeout count: it's no longer
  masked to 4 bits in place (which used to throw the count away every
  16 iterations), and a new scratch register (`x3`) holds the
  display-only masked value each iteration instead. Once `x9`'s bit at
  `TIMEOUT_SHIFT` (or higher) goes set -- checked via `SRLI`+`BEQ`,
  since RISC-V branches only compare two registers, never a register
  against an immediate, so this needs no separate threshold register
  -- the loop stops re-branching to `MAIN_LOOP` and instead writes
  `FLASH_MODE` (`0xF8`) itself, then falls into the same absolute jump
  a completed RAM bootload would use (`RUN`, patched to `RAM_BASE`),
  which now resolves to external flash instead of on-chip RAM. Also
  added `Asm.SRLI()` (needed for the threshold check) and folded the
  self-test's pass/fail path into a 2-fewer-instruction
  assume-pass-then-overwrite-on-fail shape, freeing the couple of
  instructions of ROM headroom the timeout logic needed to stay within
  the fixed 176-byte/44-instruction boot ROM window. `TIMEOUT_SHIFT`
  is currently `9` (~32000 cycles, measured), a placeholder sized for
  simulation the same way AgilA8's own threshold of 31 was -- **real
  deployment should pick a much larger shift** based on the desired
  wall-clock timeout at the core's actual clock frequency.
- Regenerated `boot_rom_body.vh`/`boot_rom.hex`; ROM size is unchanged
  at 176 bytes / 44 instructions, so it still fits the fixed `0x00-
  0xAF` window with no changes to `mem.v` or the memory map.

## Tests

- **`test/tb_boot_timeout.v`** (new) -- Part 1 leaves `ui_in[2]`
  (START) low for the whole run (no bootload attempted at all) and
  confirms: flash (CS0) stays deasserted through the first 20000
  cycles of `MAIN_LOOP`; `flash_mode` (`mem.v`'s internal register)
  latches high on its own once the timeout elapses, with no bootload
  involved; and the unattended chip falls through to run a tiny canary
  program preloaded into the flash model (`flash_canary.hex`, the same
  one `tb_flash_handoff.v` uses -- its dead byte-0 NOP just executes
  harmlessly here instead of staying unreachable, since this path
  starts execution at flash byte 0 directly rather than via a
  1-instruction handoff stub). Part 2 confirms a host that DOES assert
  START well before the timeout (same timing and 5-instruction program
  as `tb_check.v`'s scenario 3) still gets a completely normal RAM
  bootload, with `flash_mode` never set -- i.e. the new fallback logic
  doesn't preempt or otherwise disturb a responsive host.
- **`test/tb_flash_paging.v`** -- fixed a timing regression this
  feature exposed: `MAIN_LOOP`'s body got two instructions longer
  (the new `SRLI`+`BEQ` timeout check after every `LED_OUT` store),
  which shifted the loop's phase enough that the test's fixed
  20-cycle gap between asserting START and turning on write-recording
  was no longer reliably past the in-flight iteration's own stale
  `LED_OUT` write -- occasionally prepending a spurious extra entry
  (the demo counter's own value) to the exact 5-write sequence this
  test checks byte-for-byte. Bumped that gap to 100 cycles (comfortably
  past one full `MAIN_LOOP` iteration, measured at ~63-70 cycles) and
  moved `recording <= 1` to after it, so recording never turns on
  until the core has actually left `MAIN_LOOP` for `START_SEEN`.
- **`test/Makefile`** -- added `tb_boot_timeout.v` to
  `standalone-tests`.

## Docs

- **`docs/info.md`** -- rewrote the "Demo / listen loop" boot-ROM step
  and the "Reprogrammability" section for the bounded wait, and added
  a new "Boot-timeout flash fallback" section.
- **`README.md`** -- reworded the "Reprogrammable at runtime" bullet
  (listen loop is bounded now, not indefinite) and added a new
  "Boot-timeout flash fallback" bullet; bumped the test-suite counts
  (Thirteen -> Fourteen test suites, twelve -> thirteen standalone).
- **`test/README.md`** -- added `tb_boot_timeout.v` to the standalone
  list, description, and command block; bumped Twelve -> Thirteen.

**Known pre-existing staleness, not touched here:** `docs/info.md`'s
"## How to test" section still opens with "Nine test suites," a count
that was already wrong before this change (it's missing
`tb_ps2_ascii.v`, `tb_alu_test.v`, `tb_timer_pwm.v`, and
`tb_ebreak_halt.v` from an earlier feature) and lists testbenches by
name only up through `tb_ps2_reader.v`. Fixing that whole paragraph's
count and testbench list is a bigger edit than this feature's scope;
flagging it here rather than silently leaving it more wrong than
before -- `tb_boot_timeout.v` itself IS described there, appended
after the `tb_ps2_reader.v` entry, but the leading count wasn't
touched.

## Verification

- Full standalone suite (`make standalone-tests`): **13/13 testbenches
  pass, 97 PASS assertions, 0 failures**.
- Full cocotb regression (`make SIM=icarus`): **11/11 tests pass**
  (unaffected by this change, run as a regression check -- confirmed
  `test_counter_wraps`, which holds `ui_in` at 0 for its whole ~1630-
  cycle run, finishes well before the ~32000-cycle timeout threshold,
  so it never triggers the new fallback path).

## What's next

Per the prioritized roadmap:
3. Variable SPI clock divider on the shared QSPI engine
4. General-purpose SPI peripheral (CS2)
5. `shared_ram` dual-port area merge
