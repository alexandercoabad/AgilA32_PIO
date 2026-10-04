# Feature #12: I2C slave (both directions) and an arbitrating I2C multi-master

Takes "I2C slave / multi-master" off the README's "Not yet" list. **No RTL change.** 

## New
- `pio/i2c_slave_rx.pio` (29 words), `pio/i2c_slave_tx.pio` (32 words), `pio/i2c_mm.pio` (27 words).
- `tools/pio_i2c_slave.py`, `tools/pio_i2c_mm.py`: host helpers (init sequence, FIFO word builders,
  state-machine configs).
- `test/pio_tb_lib.py`, `test/test_pio_protocols.py`: I2C master/slave/multi-master models and **15 new
  cocotb tests (33 -> 48, all pass)**: 5 multi-master (single master, wins, loses and retries, loses in the
  address byte, clock synchronisation), 5 RX-slave, 4 TX-slave, 1 SCL-speed sweep.
- `tools/pioasm.py`: accepts `jmp pin, label` (comma form) and full expressions for `in/out/set` counts.

## Merge notes
The I2C files were developed against the repo of feature 10, without the USB work. They were merged
with a three-way merge (base = feature-10 repo): `pio_tb_lib.py` and `test_pio_protocols.py` had two
conflicts, both "two blocks appended at the same place" (USB models/tests and I2C models/tests), kept
together. The merged suite was run in full: 48 of 48 pass, including all USB tests.

## Measured limits (from the tests)
- Slave ACK timing: with both slaves at CLKDIV 1 an SCL period of 16 or 24 clocks fails (ACK not on the
  wire when SCL rises); 32, 40, 48, 64 pass. Recommended: 40 or more.
- RX slave accepts exactly `WRITE_BYTES` per transaction (a PIO state machine cannot notice STOP/START
  while waiting for a bit).
- The two slave programs cannot coexist (29 + 32 words in a 32-word memory), so a register-file device
  (write pointer, repeated START, read) is not possible without reloading in between.

## Not done
Gate-level simulation of these tests (still open: the feature 18 smoke test runs post-layout, but it does not touch the I2C paths or
pads 8/9; see `CHANGES_feature18.md`); re-running CI with
this feature (no RTL changed, so the layout should be identical).

## Addendum: CPU-driven slave demo
- `tools/build_pio_i2c_slave.py`, `test/tb_pio_cpu_i2c_slave.v`, `test/pio_i2c_slave_flash_image.hex`
  (1980 bytes, 45 pages); `make standalone-tests` runs it.
- `tools/pio_host.py` gained `wait_rx_ready`, `rx_get`, `byte_plus1_to_i2c_slave_word`, `write_data_reg`
  (RX-FIFO polling loop and the small ALU sequence the CPU needs between the phases).
- Flow: master writes A5 3C -> RX slave (PIO alone) -> CPU pops both, adds 1, swaps in the TX slave,
  halts -> master reads A6 3D. All bench checks pass; mutation-checked (CPU adds 2 -> `a7 3e`, FAIL).
- One thing worth knowing: PIO pin directions are sticky when a state machine is disabled, so the
  switch forces `set pindirs, 0` before the pads change program, and waits 6000 clocks first so the
  master's last ACK and STOP are over (a disabled SM would otherwise leave SDA driven low).

## Addendum: five more multi-master tests (48 -> 53, all pass)
Added to `test/test_pio_protocols.py`, against the existing `i2c_mm` and `I2cMaster` model, no RTL or
program change. They cover what the first five did not:
- `test_mm_identical_frames_neither_loses`: both masters send the same frame; neither may lose, the
  slave sees one START and one transaction (guards against a false "lost" when another master's SDA
  drive matches ours).
- `test_mm_pio_loses_on_the_last_bit_of_a_byte` / `test_mm_model_loses_on_the_last_bit_of_a_byte`:
  the bytes differ only in bit 0, so arbitration is decided right before the ACK slot; the loser must
  let go of both lines and the winner's byte and ACK must be untouched.
- `test_mm_read_versus_write_to_the_same_address` / `test_mm_write_versus_read_to_the_same_address`:
  same address, different R/W bit (the last address bit), in both directions.

Mutation-checked against `pio/i2c_mm.pio` (restored afterwards, byte-identical): with the arbitration
jump removed, the PIO-loses-on-last-bit and read-versus-write tests fail along with the existing
"loses" tests; with the "a low was expected" check removed (false losses), identical-frames and both
mirror cases fail.

Note: data-byte arbitration (loss at bit 2 of a data byte, both directions) was already covered by
`test_mm_pio_wins_arbitration` and `test_mm_pio_loses_arbitration_releases_bus_and_retries`.

## Addendum: bus-busy limitation of `i2c_mm.pio`, documented and reproduced (53 -> 57 tests, all pass)
`pio/i2c_mm.pio` has no bus-busy detection: it never checks that the bus is free before a START and does
not follow START / STOP. Found by starting the PIO master part-way through another master's transaction
(the model master, which does wait for bus-free) at eight different offsets:
- **2 of 8 destroyed the other master's transaction.** At +30 clocks (its address byte) the PIO's START was
  a second START for the slave: all four bytes NAKed, garbage captured. At +1500 (data byte 3) byte `0x55`
  arrived as `0x6C` and the last two bytes were NAKed. In both the PIO also raised IRQ 1 ("lost"), so that
  flag does not prove the winner is unharmed.
- **6 of 8 were harmless** (SDA already low or SCL low when the START edge arrived). A passing bench is
  therefore not evidence of safety; the existing multi-master tests all START both masters together
  (`join_start=True`), which is the one case arbitration handles.
- **Mitigation, verified:** the host reads `PINS_IN` until SDA and SCL are both high for 128 clocks before
  queueing the START (and again before retrying after a loss). With it all eight timings give two clean
  transactions (other master's four bytes, then the PIO's `A0 33`).
- **Not covered by the mitigation:** another master starting between the host's check and the START (the
  CPU's step is thousands of clocks, so the window is wide); a bus held low by a stuck slave (no timeout,
  no recovery clocks). A real fix is a bus-free wait inside the PIO program; five words are free, not tried.
New tests, kept as a reproducer for the numbers above: `test_mm_no_bus_busy_check_start_in_the_address_byte_destroys_the_other_master`,
`test_mm_no_bus_busy_check_start_in_a_data_byte_corrupts_it` (these two assert the limitation, so they must
be changed together with the docs if the program ever gains a check), and
`test_mm_host_bus_free_wait_protects_the_other_master_early` / `_late`.
Also: a comment block in `pio/i2c_mm.pio` and a note in `docs/info.md`, `README.md`. No RTL or program-code change.

## Addendum: slave mutation sweep (69 -> 71 tests, all pass)
Until now only `i2c_mm` and the CPU bench had been mutation-checked. `tools/i2c_slave_mutation_sweep.py` applies
29 single-line breaks, one at a time, to `i2c_slave_rx.pio` (15) and `i2c_slave_tx.pio` (14) -- no address
compare, ACK polarity, never releasing SDA, byte count +-1, sampling edge, stretch removed, NAK ignored, START
detector weakened, bit counter not reset, ... -- and runs the 12 slave tests against each (`--list` shows them).
It works on a temporary copy of the project, so the working tree is never modified; about 12 minutes in all.

First pass: 20 caught cleanly, but it exposed three weaknesses in the TESTS (the programs were fine):

| Finding | Mutant | Fix |
|---|---|---|
| **Survived**: nothing noticed | START detector without its "SDA high" precondition | `test_i2c_slave_rx_stays_silent_during_other_devices_traffic`: another master talks to another slave with zero-heavy data; our slave must push nothing, drive nothing, and still serve its own write afterwards |
| Caught **only by the speed sweep** | sampling SDA just after SCL falls instead of at the rising edge | `test_i2c_slave_rx_fast_data_change_after_scl_fall`: a master that changes SDA one clock after the fall (legal) with fully alternating bytes. The master model gained a `hold=` parameter (default unchanged: a quarter period) |
| Caught **only by hanging** (7 mutants) | no address compare, ACKing writes, ignoring NAK, ... | `feed_tx` / `drain_rx` loops bounded to 30,000 iterations (a normal transfer needs about 1,500), so a broken slave fails in seconds with "master script never finished ..." instead of timing out |

Second pass after the fixes: **all 29 mutants are caught by an assertion**; none survive, none rely on a hang.
Safeguards: a mutation that does not apply is reported as an error rather than a pass; the sweep refuses to start
unless its copy of both programs is identical to the repository's, and checks again at the end. (An earlier
scratch run of mine was interrupted mid-mutation and left a mutated file behind; its "restored" check compared
against the mutated state. The tool now compares against the repository's files, and that run was discarded.)
Re-verified on this revision: full suite 71/71; the three formerly weak mutants (R15, R11, T1) are caught.

