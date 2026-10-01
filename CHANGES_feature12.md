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
Gate-level simulation of these tests; re-running CI with
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
