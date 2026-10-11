# 11 Oct 2026 - random start-up test and shorter gate-level tests

## Reset / start-up contents
- tools/gen_random_init.py (new): Yosys lists every flip-flop; writes test/random_init.vh (225 assignments).
- test/tb.v: with `-DRANDOM_INIT=<seed>` those registers get random values 1 ns after time 0.
- test/Makefile: `make RANDOM_INIT=<seed>` (separate sim_build/rtl_rand<seed>).
- Result: 13/13 cocotb tests pass with seeds 1-5 and without a seed. Probe confirmed x5, RAM word 0 and PIO imem[0] start non-zero.
- .github/workflows/pio-tests.yaml: new step runs seeds 1-5.

## Shorter gate-level tests
- Cause: QSPI_CTRL resets to sys_clk/128 and the testbench force of the divider is a no-op on the flattened netlist.
- tools/pio_host.py: new `fast_qspi()` (SB x0 -> 0xFB). build_pio_gl_smoke.py / build_pio_gl_sig.py call it first; test/pio_gl_*_flash_image.hex regenerated.
- test/test.py: `EMULATE_GL_QSPI=1` skips the force, so an RTL run behaves like gate level for timing.
- Measured (EMULATE_GL_QSPI=1): smoke 1,927,200 -> 40,200 polling clocks; signature 4,362,900 -> 80,900 (4,401,920 -> 119,920 in all). Same golden values, all pass.
- CI confirmation (gds run #79, push ed772fa): gl_test 11 min 1 s (was 3 h 11 min), gds 2 h 35 min, whole workflow 2 h 46 min 38 s, 13/13 gate-level. Smoke 111 s (was 3,444 s), signature 164 s (was 7,564 s). pio-tests run #30: 13 min 55 s, passed (includes seeds 1-5).
