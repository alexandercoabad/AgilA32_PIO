# Feature 21 -- CAN 2.0A/B on the PIO (transmitter with bus readback + raw bit capture)

No RTL change.

## `pio/can_tx.pio` (20 words, SM0) and `pio/can_rx.pio` (10 words, SM1) -- 30 of the 32 instruction words
External CAN transceiver: TXD = PIO pin 0 (uo_out[0], `PIN_OWN` = 0x001, `set pins, 1` forced first), RXD = PIO pin 1 (ui_in[1],
`IN_BASE` = 1, `JMP_PIN` = 1 on both machines). 16 ticks per bit: bit rate = f_clk / (16 x CLKDIV).
**PIO has no XOR, so the CRC-15 and the bit stuffing are done on the CPU** (`tools/can_frame.py`).
- **TX**: command = `[n-1, ceil(n/32) data words]` (the stuffed SOF..CRC delimiter, LSB first). TXD changes on tick 2, the bus is read
  back on tick 14; the first mismatch aborts (TXD released, result word `0xFFFFFF00 | (255-x)`, machine parks; host disables, clears
  FIFOs, forces `jmp cmd`, enables). After the n bits the PIO adds ACK slot + 11 recessive bits itself and pushes the 12 samples.
  An error flag is the same command with n = 6. The whole frame is one command (no tick lost between words); the TX FIFO must not run
  empty inside a frame.
- **RX**: hard sync on the SOF edge, sample at 75 %, autopush every 32, stop after 11 consecutive recessive samples, push the partial
  word and a tag word. It sees every frame including its own. Decoding is done afterwards by the CPU (`decode_capture`).

## `tools/can_frame.py`
CRC-15, stuffing / destuffing, frame encode (standard / extended / remote), `parse_frame`, `tx_command`, `tx_result`, `abort_index`,
`capture_bits`, `decode_capture`, and `capture_words` (a Python reference model of the RX program).

## Tests
- `test/test_pio_can.py` (25 cocotb tests, added to `Makefile.proto`) with `test/pio_can_lib.py` (`CanBus`: wired-AND bus, loop delay,
  fault injection; `CanNode`: hard sync, 75 % sampling, destuffing, CRC, ACK, arbitration, error resync, clock-error scale; `CanHost`).
  CRC against an independent GF(2) division (literal polynomial), stuffing rules, 3000-frame round trip, program layout; TX: 8 frame types
  on the wire (TXD edges on the 16-tick grid, integer + fractional CLKDIV), unacknowledged frame, error flag, back-to-back commands
  (exactly 3 clocks between frames), arbitration lost (abort index = first differing bit) / won / at each identifier bit, bit errors both
  directions with recovery, loop-delay margin (0..7 clocks ok, >= 9 aborts at bit 0), `SYNC_BYP`; RX: every frame type word for word
  against the reference model, unacknowledged, back-to-back at minimum intermission, partial words of every residue, error flag, stuff
  violation, 7-word frame, clock tolerance of the sampling point (+0.19 % / -0.47 %), 14-frame mixed session.
- `test/tb_pio_cpu_can.v` + `tools/build_pio_can.py` (image `pio_can_flash_image.hex`, expectations `pio_can_expect.vh`): the CPU
  configures both machines, queues one frame, reads the result and two RX captures; the bench judges the wire bit for bit, acknowledges,
  sends a second frame, and compares every LED_OUT write. Added to `make standalone-tests` (34 bench logs).
- `tools/can_mutation_sweep.py`: 52 one-line breaks of the two programs and `can_frame.py`, **all caught** in the final run (the first
  full run showed three survivors: ACK slot one tick long -> back-to-back gap now asserted exactly; first RX sample one tick early ->
  new clock-tolerance test; CRC polynomial -> the reference no longer shares the constant). About 1.5 hours; in the manual
  `mutation_sweeps` step of `pio-tests.yaml` (description now "about 2 hours 50 minutes").
- Counts: 120 cocotb protocol tests (77 + 18 SWD + 25 CAN) and 34 bench logs.

## Limits
No resynchronisation after SOF (node clocks within about 0.2 % faster / 0.47 % slower over a 133-bit frame), no bus-idle detection
before a transmission (two nodes arbitrate only if they start together), no automatic ACK, error counters or error frames, no CAN FD;
CPU keeps FIFOs fed/drained (the CPU demo queues the whole command first). Not tried on a real bus; not run on GitHub yet.
