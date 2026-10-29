# 30 Oct 2026: constrained-random protocol tests

No change to `src/`, `pio/`, `info.yaml` or `config.json`, so the hardened design and the passing run #90 are unaffected.

New files
- `test/test_pio_random.py`: 64 constrained-random cocotb tests per seed on `pio.v` alone (UART TX, UART RX incl. break frames and +-1.5 % baud error, SPI modes 0-3 incl. CLKDIV 1 with the MISO synchroniser bypassed, I2C write / read / register read / wrong address with random clock stretching, PS/2 RX), each checked against a reference model with bit-timing checks.
- `test/Makefile.random`: `make -f Makefile.random [PIO_RANDOM_SEED=n] [PIO_RANDOM_N=n]`.

Changed files
- `.github/workflows/pio-tests.yaml`: new step "Constrained-random protocol tests" (seeds 1-5, `python -m cocotb_tools.check_results` after each) and the result files in the log upload.
- `test/README.md`, `SUBMISSION.md`: description; the "Known limits" line now says exactly what the random tests cover.

Checked
- Seeds 1-6 (384 iterations): all pass; about 40 s per seed on one core.
- The tests were run against deliberately broken copies of the design (not committed): UART TX bit delay, UART RX sample point and bit delay, SPI mode-0 delay, I2C SCL / hold delays, PS/2 idle timeout, OSR shift direction in `pio_sm.v`. Each one made at least one iteration fail.
- While writing the tests, three of my own first-draft failures were test errors, not design faults: the UART TX drain wait was one frame short, the break test read the IRQ too early, and a PS/2 half period of 292 clocks is outside the program's specified range (idle timeout 288 clocks).
