# 30 Oct 2026: formal verification of the PIO FIFO and state machine

No change to `src/`, `pio/`, `info.yaml` or `config.json`: the hardened design and the passing run #90 are unaffected.

New: `formal/` (see `formal/README.md`)
- `run.sh`: four unbounded proofs (SymbiYosys, abc pdr), about 1 minute; `./run.sh --mutants` adds the mutant checks.
- `fifo_props.vh`, `pio_fifo_formal.v`, `pio_fifo.sby`: FIFO flags / level / pointers / in-order bit-exact data / clear / reset, and equivalence with a reference queue.
- `sm_props.vh`, `pio_sm.sby`: state machine never pops an empty or pushes into a full FIFO, counters <= 32, quiet when disabled / between ticks / in delay slots,
  stall and forced-instruction PC rules, exact delay-slot length, restart state, clock divider period (dividers below 8.0).
- `check_mutants.py`: 28 deliberately broken copies of the design; every harness rejects all of them.
- `instrument.py`: inserts the properties into a COPY of the design (src/ is never touched).
- `.github/workflows/formal.yaml`: manual-only workflow (untested on GitHub; the proofs were run locally).

Not covered formally: `pio.v`, `mem.v`, the CPU core, the QSPI engine; S8 only for dividers below 8.0.
