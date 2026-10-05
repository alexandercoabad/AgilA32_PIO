# Feature 20 -- post-layout PIO signature test

**Why.** The feature 18 smoke test is the only PIO test that runs on the post-layout netlist, and it never touches
`in`, most `out` / `mov` / `jmp` forms, side-set, PINDIRS, autopush / autopull, the input synchroniser or pads 8/9
(the I2C and SWD pads). This adds a second test that does, with golden values.

**New files**
- `pio/gl_sig0.pio` (17 words), `pio/gl_sig1.pio` (14 words): the two programs (31 of 32 instruction words).
- `tools/build_pio_gl_sig.py`: flash image builder (`cd tools && python3 build_pio_gl_sig.py && cp pio_gl_sig_flash_image.hex ../test/`).
- `test/pio_gl_sig_flash_image.hex`: the image. `tools/pio_gl_sig_flash_image.bin/.hex` are the builder's outputs (the .hex is copied to `test/`).
- `test/test_pio_gl_sig.py`: the cocotb test `test_pio_postlayout_signature` (docstring lists every form and value).
- `tools/gl_sig_mutation_sweep.py`: 40 one-line mutants of the programs, `pio_sm.v` and `pio.v`.

**Changed files:** `test/Makefile` (`COCOTB_TEST_MODULES = test,test_pio_gl,test_pio_gl_sig`), `README.md`, `docs/info.md`,
`.github/workflows/pio-tests.yaml` (the manual mutation-sweep step also runs the new sweep).

**What it checks** (pins only, so it runs on RTL and on the gate-level netlist): the exact `uo_out[7:0]` sequence,
the exact (output enable, output value) sequence on pads 8/9 with testbench pull-ups, the RX words the CPU collected
and the IRQ flags (shown on `uo_out` after `PIN_OWN` is cleared). Details: `docs/info.md`, "Post-layout PIO signature test".

**Evidence (all on 5 Oct 2026, this sandbox)**
- RTL: passes; the full `make` in `test/` passes 13/13 (12 before + this one).
- Yosys-synthesized netlist of `src/` (generic gates via `synth` + `abc`, hierarchy kept): passes. This is *not* the
  hardened IHP netlist and it is not a substitute for the CI `gl_test` run.
- Cost: about 112,000 clocks with fast QSPI. With the slow QSPI of the gate-level run I measured 3.68 million clocks on RTL against 1.97 million
  for the smoke test and predicted 1.9 times the time; the real gate-level run needed 4.40 million (2.2 times, see below).
  `GL_SIG_MAX_CLOCKS` (default 20,000,000) is the polling budget; the test fails, it does not hang, if the budget runs out.
- Mutation sweep: 38 of 40 caught. Survivors: autopull threshold boundary (R6) and the STATUS source select (R15).

**Real post-layout run (gds run #65, 5-6 Oct 2026, push `79f0997`):** gate-level **13/13 passed**, 0 failures, 0 errors, so every golden
value matched on the placed IHP netlist. Per test (wall clock / simulated clocks): signature 7,563.5 s / 4.40 million; smoke 3,443.6 s /
1.97 million; `test_bootloader_loads_and_runs_program` 267.5 s; the other ten 0.1-40 s. Jobs: `gds` 2 h 40 min, `gl_test` 3 h 11 min, precheck 8 min;
the whole workflow 5 h 52 min, which is about `gds` plus `gl_test`, so precheck overlaps. The signature test is therefore about 2.2 times the
smoke test (the 1.9 estimated above from an RTL measurement was low). Runner speed varies: the smoke test took 2,587 s or less in the
feature 18 run and 3,444 s in this one, so treat all times as single measurements. `gl_test` at 191 minutes is inside GitHub's 360-minute
limit per job (a margin of about 1.9 times).
