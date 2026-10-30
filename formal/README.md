# Formal verification of the PIO block (FIFO and state machine)

Unbounded proofs (every input sequence, no depth limit) with SymbiYosys and the `abc pdr` engine. The design files in `src/` are
**not modified**: `instrument.py` writes a copy in `formal/build/` with the property file inserted before `endmodule`, so the
properties can see internal registers.

    cd formal && bash run.sh              # four proofs, about 1 minute
    cd formal && bash run.sh --mutants    # also: every harness must reject deliberately broken copies of the design (GitHub run #3 took 2 min 20 s in total with this ticked)

Tools: Yosys 0.33, SymbiYosys, `yosys-abc` (OSS CAD Suite or Ubuntu 24.04 `yosys` + SymbiYosys). The z3 and boolector SMT engines
were too slow for these designs, hence `abc pdr`.

## What is proven

`pio_fifo.v` (TX and RX queue of every state machine), files `fifo_props.vh`, `pio_fifo_formal.v`, `pio_fifo.sby`:

| | Property |
|---|---|
| P1 | level never above the depth; `full` / `empty` flags always agree with the level; never both |
| P2 | level next = level + push - pop; push into a full FIFO and pop of an empty FIFO change nothing; clear and reset empty the queue |
| P3 | read / write pointers agree with the level |
| P4 | data integrity: a word pushed is the word popped, in order, bit-exact, never lost or duplicated |
| equiv | the circular buffer behaves exactly like a shift-register reference queue for every push / pop / clear / reset sequence |

`pio_sm.v` (one state machine; any instruction word, any configuration, any FIFO / pin / IRQ state, reset at any time), files
`sm_props.vh`, `pio_sm.sby`:

| | Property |
|---|---|
| S1 | never pops an empty TX FIFO, never pushes into a full RX FIFO (the FIFO would silently drop the data) |
| S2 | the OSR / ISR bit counters never exceed 32 |
| S3 | quiet when not ticking: while disabled, between divided-clock ticks, or in a delay slot it drives no pin or direction, pops / pushes nothing, raises / clears no IRQ flag |
| S4 | pin and direction values only appear where the mask says the pin is written |
| S5 | a stalled instruction keeps the PC; a host-forced instruction that does not jump does not move the PC; no tick, no movement |
| S6 | delay slots last exactly the programmed number of ticks |
| S7 | restart clears the OSR / ISR counters, ISR, delay, phase and EXEC state |
| S8 | clock divider: ticks exactly every N clocks for an integer divider N, accumulator stays below the divisor |

## Result on GitHub

`.github/workflows/formal.yaml` (manual, `workflow_dispatch`, optional `mutants` input) ran for the first time successfully as run #3 on 30 Oct 2026
(push `f03db8d`): success, job 2 min 16 s, workflow 2 min 20 s, artifact `formal-logs`. All four proofs ended `DONE (PASS, rc=0)` with the `abc pdr` engine
(`pio_fifo_props` 4 s, `pio_fifo_equiv` 10 s, `pio_sm_core` 1 s, `pio_sm_clkdiv` 20 s of solver time). That run had the `mutants` option ticked: all 28 mutants were rejected
(39 harness checks). The mutant lines are in the job's console output, not in the `formal-logs` artifact. The one annotation was GitHub's Node.js 20 deprecation warning for one of the actions.

## Limits (stated honestly)

- S8 is proven for a fixed divider below 8.0 (`CLKDIV_SMALL`); the solver does not reach the full 24-bit divider range. S3-S7 have no such limit.
- Only `pio_fifo.v` and `pio_sm.v` are covered. Not covered formally: `pio.v` (register interface, pin multiplexing, IRQ flags), `mem.v`, the CPU core, the QSPI engine. They are covered by the cocotb / Icarus tests, the random tests and the mutation sweeps.
- The properties say what must never happen and some exact timing rules; they do not prove that a given PIO program implements a protocol correctly (that is what the protocol tests do).
- The asynchronous reset is modelled the way SymbiYosys models it (`async2sync`): the reset acts in the cycle it is low. The proofs start in reset and allow the reset to fall in any later cycle.
- Memory contents of the FIFO have no reset on silicon; the proofs start with arbitrary contents.

## Do the proofs bite?

`check_mutants.py` breaks a copy of the design 28 ways, one at a time (FIFO pointer and flag errors, popping an empty FIFO, pushing into a full one,
a counter that is not capped, a delay slot that executes, a stalled instruction that moves the PC, a divider that ticks one clock late, ...). Every harness
accepts the real design and rejects every broken copy. Two first-draft mutants of the state machine that were *not* rejected were equivalent changes
(the pop / push flags are only ever set on a completing tick) and were replaced by changes that alter behaviour.

## Toolchain note

`pio_sm.v` indexes `side_fld[val_bits]`, which can point past bit 4 when `side_count` is 5 without the enable bit.  The result is masked
by `!side_en ||`, so the value never matters, but recent Yosys releases (as shipped in the OSS CAD Suite) turn the out-of-range
`$shiftx` into a constant `x`, and `write_aiger` then stops with "Design contains 'x' or 'z' bits".  `pio_sm.sby` (and the mutant
template in `check_mutants.py`) therefore lower the shifts and replace the free value with `$anyseq` right after `prep`
(`techmap t:$shiftx t:$shift` + `setundef -anyseq`).  `src/pio_sm.v` is unchanged.
