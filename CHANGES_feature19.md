# Feature 19 -- SWD (ARM Serial Wire Debug) host on the PIO

No RTL change.

## `pio/swd.pio` (15 words, one state machine)
SWCLK on side-set pin 0 (uo_out[0]), SWDIO on PIO pin 8 (uio[4]; external pull-up). Two commands only:
header word `[4:0]` = n-1, `[5]` = read. *Write*: the next word is the data, LSB first, SWDIO driven. *Read*: SWDIO released,
sampled on the last tick before each rising edge. Every command pushes one RX word (write: 0 as completion marker; read: data in the
top n bits, `word >> (32-n)`). 8 ticks per bit, SWCLK 4 high / 4 low, idle low; SWDIO changes when SWCLK falls (setup 4, hold 4 ticks).
Everything else (request byte, parity, turnarounds, ACK, idle clocks, line reset, JTAG-to-SWD switch) is built by the CPU from
these two commands; turnarounds are read commands, so there is no bus contention by construction.

## `tools/pio_swd.py`
Request byte, parity, command words, DP / MEM-AP register addresses, CTRL/STAT bits, `select_word`.

## Tests
- `test/test_pio_swd.py`: your 14 tests unchanged (protocol-strict SW-DP + MEM-AP model, all pass) **+4**: command lengths 1..32 with
  bit order on the wire, the SYNC_BYP option, a full RX FIFO leaves SWCLK low, and a long session with WAIT retries and two AP banks.
  `test/Makefile.proto` now lists `test_pio_protocols,test_pio_swd`, so `make standalone-tests` and `pio-tests.yaml` run them
  (94 cocotb protocol tests in all).
- `tools/swd_mutation_sweep.py`: 20 one-line breaks of `swd.pio` **all 20 caught** (about 25 s per mutant; the first full run showed that a mutant which makes the target wait for data forever took minutes to fail, so the host helper in the test now gives up after 6000 polls).

## Limits
No CPU-driven demo through the top level (unlike JTAG / 1-Wire / VGA): the packet logic lives in the host code of the tests, and a
CPU doing a whole SWD request is a few hundred instructions across several 44-byte pages. Not tried on a real target; no
multi-drop (DLPIDR / TARGETSEL) and no SWD-to-dormant sequence; read-data parity errors are only reported to the host code.
