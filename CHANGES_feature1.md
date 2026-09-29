# Feature #1: Halted-status PIN_MUX mode (ported from AgilA8)

Ported AgilA8's `GPIO_DIR[7]` halted-status output mux to AgilA32.
Since AgilA32's `rv32i_core.v` had no `HALT` opcode to key off of
(unlike AgilA8's dedicated 16-instruction encoding), `EBREAK` was
wired up as the halt trigger instead — close to its usual RISC-V
"stop and hand off to a debugger" role.

## RTL changes

- **`src/rv32i_defs.vh`** — added `ST_HALTED` FSM state (`3'd7`).
- **`src/rv32i_core.v`** — added a `halted` output; `EXEC` now detects
  `EBREAK` (`OP_SYSTEM`, `funct3=000`, `imm[11:0]=0x001`) and parks
  the FSM in `ST_HALTED` (no more fetches, no memory activity) until
  `rst_n`. `FENCE`/`ECALL` are untouched — still NOPs.
- **`src/mem.v`** — `PIN_MUX` (`0xFA`) widened from 1 bit to 2 bits:
  `00`=LED (default), `01`=PWM, `10`=halted status (new), `11`=
  reserved (falls back to LED). **Fully backward compatible** — old
  code writing 0 or 1 behaves identically to before.
- **`src/tt_um_agila32.v`** — wires the core's `halted` output into
  the top-level `uo_out[7]` 3-way mux.

## Tooling

- **`tools/asm_pineapple.py`** — added `Asm.EBREAK()` so programs can
  actually trigger the new halt behavior.

## Tests

- **`test/tb_timer_pwm.v`** — updated for the 2-bit `PIN_MUX` port,
  added cases for the new halted-status mode and the reserved `11`
  fallback.
- **`test/tb_ebreak_halt.v`** (new) — Part 1 drives `rv32i_core`
  directly against a hand-built 4-instruction ROM (two `ADDI`s, an
  `EBREAK`, then a trailing `ADDI` that must never execute) and
  checks `halted` timing/persistence/reset-recovery. Part 2 bootloads
  the same program into the real `tt_um_agila32` top level over the
  GPIO DATA/CLOCK/START protocol and confirms `PIN_MUX=2'b10` surfaces
  the real halted core's status on `uo_out[7]`.
- **`test/Makefile`** — added the new testbench to `standalone-tests`.

## Docs

- **`docs/info.md`** — rewrote the `PIN_MUX` section for the 2-bit
  field, added a new "EBREAK halts the core" section.
- **`README.md`** — updated the Timer/PWM status bullet, added a new
  EBREAK-halt status bullet, corrected the test-suite counts (also
  fixed a pre-existing stale count: the repo already had 11 standalone
  testbenches before this change, not the 10 the README claimed).
- **`test/README.md`** — added the new testbench to the standalone
  list and command block.

## Verification

- Full standalone suite (`make standalone-tests`): **12/12 testbenches
  pass, 93 PASS assertions, 0 failures**.
- Full cocotb regression (`make SIM=icarus`): **11/11 tests pass**
  (unaffected by this change, run as a regression check).

## What's next

Per the prioritized roadmap:
2. Boot-timeout → automatic flash fallback (self-contained to the boot
   ROM program)
3. Variable SPI clock divider on the shared QSPI engine
4. General-purpose SPI peripheral (CS2)
5. `shared_ram` dual-port area merge
