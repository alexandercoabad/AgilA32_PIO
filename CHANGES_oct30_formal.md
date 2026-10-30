# 30 Oct 2026: formal verification of the PIO FIFO and state machine

No change to `src/`, `pio/`, `info.yaml` or `config.json`: the hardened design and the passing run #90 are unaffected.

New: `formal/` (see `formal/README.md`)
- `run.sh`: four unbounded proofs (SymbiYosys, abc pdr), about 1 minute; `./run.sh --mutants` adds the mutant checks.
- `fifo_props.vh`, `pio_fifo_formal.v`, `pio_fifo.sby`: FIFO flags / level / pointers / in-order bit-exact data / clear / reset, and equivalence with a reference queue.
- `sm_props.vh`, `pio_sm.sby`: state machine never pops an empty or pushes into a full FIFO, counters <= 32, quiet when disabled / between ticks / in delay slots,
  stall and forced-instruction PC rules, exact delay-slot length, restart state, clock divider period (dividers below 8.0).
- `check_mutants.py`: 28 deliberately broken copies of the design; every harness rejects all of them.
- `instrument.py`: inserts the properties into a COPY of the design (src/ is never touched).
- `.github/workflows/formal.yaml`: manual-only workflow. First GitHub run (#3, 30 Oct 2026, push `f03db8d`, started by hand, `mutants` option ticked): success, job 2 min 16 s, workflow 2 min 20 s; all four proofs PASS (`pio_fifo_props` 4 s, `pio_fifo_equiv` 10 s, `pio_sm_core` 1 s, `pio_sm_clkdiv` 20 s); artifact `formal-logs` (8.33 KB). One annotation: GitHub's Node.js 20 deprecation warning. The mutant check also ran: all 28 mutants rejected (39 harness checks); its output is in the job console, not in the artifact.

Documentation updated after that run: `README.md` (badge, test summary, verification list, status checklist, repo layout, "Formal verification" section), `AgilA32_Project_Documentation.pdf` (Section 7.6 and the tables that said "no formal verification"), and the stale "untested on GitHub" note in `formal.yaml`.

Not covered formally: `pio.v`, `mem.v`, the CPU core, the QSPI engine; S8 only for dividers below 8.0.
