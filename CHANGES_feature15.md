# Feature #15: 1-Wire master (`pio/onewire.pio`)

No RTL change. A 31-word PIO program, 7 cocotb tests, and a CPU-driven top-level demo.

## `pio/onewire.pio`
Open-drain DQ on PIO pin 8 (uio[4]), standard speed, 1 us tick (CLKDIV 24 at 24 MHz). One FIFO command word per
operation (bit 0 op, bits [5:1] n-1, bits [31:6] data LSB first); each command pushes one RX word (reset: bit 31 is the
presence level, 0 = device answered; transfer: the sampled bits). Reset low 530 us, write-1/read low 3 us, sample 13 us
after the fall, write-0 low 65 us, slot 70-71 us.

## Tests
- `test/pio_tb_lib.py`: `crc8_maxim`, `make_rom`, `OneWireSlave` (checks reset length, presence, slot timing, recovery and
  read-slot hold; records violations).
- `test_pio_protocols.py` +7 (-> 69): reset and presence (four device timings, >= 480 us recovery) | absent device |
  write-slot timing (0x33 decoded LSB first) | READ ROM with CRC-8 | partial bit counts | read-0 hold margin (10 and 12 us
  fail, 13+ ok) | clock-tolerance sweep (CLKDIV 22.8-25.2 required; 22.0 and 28.0 fail).
- `tools/build_pio_onewire.py` + `test/tb_pio_cpu_onewire.v` (wired into `make standalone-tests`, 27 -> 28): CPU resets,
  sends READ ROM, shows 0xA0 then the 8 ROM bytes; CRC-8 = 0, no slot violation, bus released.
- `tools/pio_host.py`: new `add_imm(rd, imm)`.
- Mutation-checked: 8 mutants (short reset, late sample, short write-0, short slot, reads sent as zeros, IN shifting left,
  wrong ROM command, DQ never released) all fail the top-level testbench; the unmutated image passes.

## Limits
Clock window about -7.7 % / +15 % around 24 MHz. Not done: overdrive, strong pull-up, ROM search, a real device.

## Addendum: 1-Wire mutation sweep (`tools/onewire_mutation_sweep.py`)
Breaks `pio/onewire.pio` one line at a time (23 mutants: reset length / release / recovery, presence sample point, RX push
after reset and after a transfer, write-1 / read low time, read sample point, slot length, write-0 low time and release, the
dummy IN, bit count, op and data polarity, bit loop) on a temporary copy of the project. Layer 1 runs the seven
`test_onewire_*` cocotb tests (fastest first, stops at the first failure; `--full` runs all seven), layer 2 rebuilds the
firmware and runs `tb_pio_cpu_onewire.v` for mutants layer 1 missed. Shared engine: `tools/mutation_common.py`.
Result: all 23 caught (full run about 17 min, most in under 10 s; one survivor costs about 2.5 min).
Three things went wrong on the first run, all fixed:
- **O19 survived** (write-0 slot without the dummy `in pins, 1`): no test looked at the RX word of a write. `test_onewire_write_slot_timing`
  now asserts that writing 0x33 returns 0xFF (one sample per slot, 0-slots included).
- **O1 and O3 were badly chosen**: x = 14 gives a 497 us reset (inside the 480 us minimum) and a presence sample at 65 us is
  inside every device timing in the model. Replaced by x = 13 (464 us) and a sample 37 us after the release; both are caught.
- **Early read sampling cannot be tested here** (e.g. 7 us after the fall instead of 13): the spec allows it and the model's
  pull-up is ideal, so the mutant behaves like the real program. It is left out of the sweep and listed as a limit.
