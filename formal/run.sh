#!/bin/bash
# Formal verification of the PIO block's FIFO and state machine.
#     cd formal && ./run.sh              proofs only (about 2 minutes)
#     cd formal && ./run.sh --mutants    also checks that every harness rejects deliberately broken copies of the design
# Needs yosys, sby (SymbiYosys) and yosys-abc (all in the OSS CAD Suite).  Works on COPIES of the design files in build/;
# src/ is never modified.  Results: PASS / FAIL per proof below; sby work directories are left in formal/ (git-ignored).
set -u
cd "$(dirname "$0")"
mkdir -p build
python3 instrument.py ../src/pio_fifo.v fifo_props.vh build/pio_fifo_props.v
python3 instrument.py ../src/pio_sm.v   sm_props.vh   build/pio_sm_props.v
fail=0
mutants=${1:-}      # 'set --' below overwrites $1
for job in "pio_fifo.sby props" "pio_fifo.sby equiv" "pio_sm.sby core" "pio_sm.sby clkdiv"; do
    set -- $job
    if sby -f "$1" "$2" > "build/$(basename $1 .sby)_$2.log" 2>&1; then
        echo "PASS  $1 $2"
    else
        echo "FAIL  $1 $2   (see formal/build/$(basename $1 .sby)_$2.log)"; fail=1
    fi
done
if [ "$mutants" = "--mutants" ]; then
    python3 check_mutants.py || fail=1
fi
exit $fail
