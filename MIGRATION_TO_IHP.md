# Sky130 -> IHP SG13CMOS5L migration notes

Base: `ttihp-verilog-template-cmos5l`. RTL (`src/*.v`, `*.vh`) is unchanged; the design is PDK-agnostic.

| File | Change |
|---|---|
| `.github/workflows/*.yaml`, `.devcontainer/`, `.vscode/` | Taken from the IHP template (`ihp-cmos5l` actions, `pdk: ihp-sg13cmos5l`, `PDK=ihp-sg13g2`, LibreLane 3.0.0.dev44) |
| `src/config.json` | IHP template config (adds `FP_PDN_VWIDTH: 2.1`, `FP_PDN_VPITCH: 50.0`); kept `CLOCK_PERIOD: 1000` |
| `info.yaml` | Same metadata/pinout; `tiles` stays **6x2** (see below) |
| `test/Makefile` | IHP GL-sim libs (`sg13cmos5l_io/stdcell`), `FST`, `COCOTB_TEST_MODULES`; removed `USE_POWER_PINS`/`UNIT_DELAY`; explicit `PROJECT_SOURCES`; standalone-tests target kept |
| `test/tb.v` | IHP template tb (no VPWR/VGND ties, FST dump), instantiates `tt_um_agila32` |
| `src/project.v` | Removed (unused placeholder `tt_um_example`) |
| `README.md` | PDK references updated |

## Sizing (yosys, sg13g2 liberty as a stand-in for cmos5l)
~11,100 cells, 0.174 mm2 cell area, ~1,700 flops (register file + RAM). At ~0.03 mm2/tile and
60% density that needs ~10 tiles, so 6x2 (12 tiles, ~48% util) is the practical minimum; 6x4
(contest limit) gives ample headroom. The full LibreLane flow was not run here.

## GL-test fix
`test/Makefile` GL libraries are now derived from `$(PDK)` (set from the GDS run's `pdk.json`),
instead of hardcoding `ihp-sg13cmos5l`. The first GDS run hardened with `ihp-sg13g2`, so the
hardcoded paths did not exist.

## First GDS run (6x2, CLOCK_PERIOD 1000 ns)
Flow completed: 0 magic DRC, 0 LVS/antenna violations, no setup/hold violations,
17,795 std cells / 0.245 mm2 (~62% utilization). Worst setup slack ~794 ns => critical path
~206 ns (~4.8 MHz max). 5 max-slew violations in the slow corner (non-fatal).
