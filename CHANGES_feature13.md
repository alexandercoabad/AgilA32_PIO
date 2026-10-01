# Feature #13: I2C slave (both directions) and an arbitrating I2C multi-master

Takes "I2C slave / multi-master" off the README's "Not yet" list. **No RTL change.** (The files
arrived labelled "feature 11" -- that number is the USB engine -- and were renumbered to 13. There is
no `CHANGES_feature12.md` in the repo this was merged into.)

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
CPU-driven end-to-end demo of the slaves; gate-level simulation of these tests; re-running CI with
this feature (no RTL changed, so the layout should be identical).
