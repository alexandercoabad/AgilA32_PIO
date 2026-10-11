# 11 Oct 2026 - cheap-list improvements

1. **C flow in the repo**: `c/` (build.sh, link.ld, start.S, agila32.h, README.md, examples blink/counter/chase/switches and the
   PILIPINAS assembly program with simulator and hardware `.hex`).
2. **README**: new "Try it and program it" section (browser simulator, `c/`, `SUBMISSION.md`), `RANDOM_INIT` and `make benches`
   in "Testing locally", `c/`, `SUBMISSION.md` and `tools/gen_random_init.py` in the repo layout.
3. **pio-tests.yaml**: each cocotb seed keeps its own `results_seed<N>.xml` (uploaded with the logs); new step runs the Icarus benches
   with random start-up contents (seeds 1-3).
4. **Random start-up contents in the standalone benches**: the 22 `tb_*.v` benches that instantiate `tt_um_agila32` include
   `random_init.vh` under `ifdef RANDOM_INIT`; `make benches RANDOM_INIT=<seed>`; `make standalone-tests` = `benches` + protocol suite.
   `tools/gen_random_init.py` now randomises only flip-flops WITHOUT an asynchronous reset (97 assignments, was 225): randomising
   reset flip-flops at 1 ns only produced a pin glitch before the first clock that the bench monitors misread (UART / WS2812
   benches failed for seeds 1 and 3 in the first attempt; not a design bug). Results: cocotb seeds 1-5 pass 13/13; benches
   seeds 0-3 pass (247 PASS lines, no FAIL). Not randomised: the 12 benches that drive only the core, mem or PIO block.
5. **SUBMISSION.md**: checklist with the evidence for each contest requirement, timings and known limits.
6. Documentation (PDF/docx, 22 pages) updated for all of the above.
