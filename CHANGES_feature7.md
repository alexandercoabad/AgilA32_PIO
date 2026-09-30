# Feature #7: SPI master modes 2 and 3 (CPOL = 1) on the PIO

Adds the two missing SPI clock polarities as PIO programs; no RTL change.

## Programs
- `pio/spi_cpol1_cpha0.pio` (mode 2) = `spi_cpha0` with inverted side-set values.
- `pio/spi_cpol1_cpha1.pio` (mode 3) = `spi_cpha1` with inverted side-set values.
The Pico SDK uses a pad output override for CPOL = 1; this block has none, so SCK
idles high by inverting the side-set values instead.

## Tests
- `test/pio_tb_lib.py`: `SpiSlave` now models modes 0-3 (mode = CPOL*2 + CPHA), records
  idle SCK level, rising and falling edge cycles, and measures MOSI setup/hold in clocks.
- `test/test_pio_protocols.py`: `test_spi_mode2`, `test_spi_mode3`,
  `test_spi_modes_0_and_1_idle_low` (regression guard); 16 -> 19 tests.
  `spi_run` presets SCK high (forced `set pins, 1 side 1` = 0xF001) before `PIN_OWN` for CPOL = 1.
- Mutation-checked: swapping the mode 2 / mode 3 programs, or running mode 2 with the
  mode 0 program, makes the tests fail.

## Pitfalls found while doing it
- A forced instruction carries the side-set value in bit 12, so `set pins, 1` (0xE001)
  also side-sets SCK to 0. Use 0xF001 for CPOL = 1.
- A slave model that samples in the cycle MOSI changes cannot tell a correct program from
  one whose data and clock edge coincide; setup/hold are now asserted explicitly.
- On the real chip the pad is driven from `GPIO_OUT` (0xF0) until `PIN_OWN` is set, so
  firmware must set the SCK bit there first. `pio.v`-level tests cannot see this.

## Not done
- No CPU-driven, top-level (`tt_um_agila32`) test or flash image for modes 2/3 yet.
