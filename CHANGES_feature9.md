# Feature #9: CPU-driven JTAG (IEEE 1149.1) through the PIO

Adds the JTAG item that the protocol table listed as "Not attempted". No RTL change.

## What it does
`pio/jtag.pio` (24 words, original) + `tools/build_pio_jtag.py` -> `pio_jtag_flash_image.{bin,hex}`
(2068 bytes, 47 pages). The CPU resets the TAP (TMS 1x5,0), reads the 32-bit IDCODE, loads IR = USER
(4 bits), writes the 16-bit USER register with the IDCODE bits it just popped from the RX FIFO (a
relay through a CPU register), reads USER back, shows the low byte on `GPIO_OUT` and EBREAKs.
Clock budget: 6 + (3+32+2) + (4+4+2) + (3+16+2) + (3+16+2) = **95 TCK pulses**.
Pins: TDI `uo_out[0]`, TMS `uo_out[1]`, TCK `uo_out[2]` (PIO side-set), TDO `ui_in[3]`. CLKDIV 4.

Header word: `[4:0]` pulses-1, `[5]` 1 = data shift (next word = TDI, TMS=1 on last pulse, captured TDO
pushed), 0 = TMS sequence (bits `[15:6]`, first pulse in bit 6). ISR shifts right, so a 32-bit scan
returns the register LSB-first and the low half can be sent straight back.

## Tools
`tools/pio_host.py`: new `wait_sm_idle(sm, pc)` (TX FIFO empty, then ADDR == loop top).

## Test: `test/tb_pio_cpu_jtag.v` (wired into `test/Makefile`)
Pins-only IEEE 1149.1 TAP model. Checks per-state walk, scan lengths DR32/IR4/DR16/DR16, exactly 95
TCK pulses, TCK idle low, final state Run-Test/Idle, TDI/TMS never change while TCK is high, >= 30 ns
setup and hold, USER captured 0000 then IDCODE[15:0] and ends holding it, `GPIO_OUT` = IDCODE[7:0],
and the core halts only after the last TCK pulse.

## Two bugs found while building it
1. **Destructive read.** First version passed 9/10: `USER ends 0x0000`. Every scan is also a write:
   the read-back shifted zeros in and Update-DR latched them, erasing what had just been written
   (the value read was right, so `GPIO_OUT` passed). Fix: the read-back scan shifts the same value in.
2. **Halt before the sequence ends.** The SM pushes the last scan's word *before* the trailing
   Exit1 -> Update -> RTI clocks, so halting on the RX word can stop the CPU mid-sequence. Fix:
   `wait_sm_idle` before `write_gpio_out`/`halt`. At CLKDIV 4 the race is masked by CPU speed (the
   CPU needs ~6 flash-fetched instructions after the pop), which is why it is not visible in the
   default run: at CLKDIV 200 without the wait the target is left in Exit1-DR with 93 of 95 pulses;
   with the wait the same run passes.

## Mutation checks (each built into its own image and run through the testbench)
Caught: relay replaced by a constant; result not shifted down; TMS never 1 on the last pulse; TDI
changing as TCK rises (22 setup violations); ISR shifting the wrong way; no idle-wait (at slow TCK).
Hung to a timeout (caught only by the simulation timeout, not an assertion): OUT shifting left.
Equivalent mutant (no observable change): keeping TCK high a little longer in the TMS path.

## Not done
- The TAP model starts in Test-Logic-Reset, so the testbench cannot tell a 5-clock reset from a
  shorter one; only the pulse count (95) pins the reset length.
- One target, one TAP, no TRST/SRST, no boundary scan, no SWD. Simulation only (RTL); not re-run
  through the GDS / gate-level flow (no RTL change, so the existing GL result is unaffected).
