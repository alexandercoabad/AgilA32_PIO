# Feature #4 gate-level fix: "Multiple edge sensitive events found for this signal"

`Generate JSON Header` (Yosys, run by the wokwi/LibreLane hardening
flow) was failing with:

```
Creating register for signal `\mem.\gpio_out' using process `\mem...mem.v:639...'.
ERROR: Multiple edge sensitive events found for this signal!
```

Reproduced locally with:

```
yosys -p "read_verilog -sv tt_um_agila32.v rv32i_core.v mem.v qspi_shared_engine.v; hierarchy -top tt_um_agila32; proc"
```

This traced to **two separate bugs**, both introduced by feature #4
(the generic SPI peripheral), both simulation-clean but
synthesis-broken:

## Bug 1 -- `qspi_shared_engine.v`: conditional call to a Verilog function inside a clocked non-blocking assignment

`ST_IDLE` picks `sreg`'s preload value with:

```verilog
sreg <= (req_dev == 2'd0) ? {req_wdata[7:0], 56'h0}
                          : build_preload(req_we, req_size, req_addr, req_wdata);
```

Calling a plain Verilog function on only one branch of a ternary
feeding a non-blocking assignment makes Yosys's `proc_dff` pass
unable to resolve a single edge-sensitivity for the function's own
locals (reproduced: it fails specifically on `build_preload`'s
`addr` input). `half_period_for()`, called unconditionally a few
lines below, never had this problem -- that's the tell.

Marking `build_preload`/`half_period_for` `automatic` does **not**
fix this (tried, still fails identically). The actual fix: hoist the
function call out into an unconditional combinational wire, and only
make the *selection* between it and the raw-byte path conditional:

```verilog
wire [63:0] preload_value = build_preload(req_we, req_size, req_addr, req_wdata);
...
sreg <= (req_dev == 2'd0) ? {req_wdata[7:0], 56'h0} : preload_value;
```

## Bug 2 -- `mem.v`: `spi_last_rx`'s update as a second, sibling top-level `if`

The write-path `always @(posedge clk or negedge rst_n)` block had its
usual `if (!rst_n) ... else if (we) ...` chain, followed by a
*separate*, second top-level statement after it:

```verilog
end  // end of if(!rst_n)/else if(we)

if (rst_n && ext_ready && in_spi_write)
    spi_last_rx <= ext_rdata[7:0];
```

This simulates fine (all standalone testbenches passed with it), but
gives the process two disconnected conditional trees in the same
always block, which is what actually broke Yosys's `proc_dff` pass
for this process -- reported against `gpio_out` (the first signal
driven in the block, in source order) even though `gpio_out` itself
was never touched by this change. Confirmed by isolating `mem.v`
alone: `yosys -p "read_verilog -sv mem.v; hierarchy -top mem; proc"`
failed the same way with zero other RTL involved.

Fix: nest `spi_last_rx`'s update inside the *same* `else` branch as
everything else, as a sibling `if` (not a second top-level one):

```verilog
end else begin
    if (we) begin
        ... (unchanged)
    end
    if (ext_ready && in_spi_write)     // rst_n&& dropped -- already
        spi_last_rx <= ext_rdata[7:0]; // implied by being in `else`
end
```

## Verification

- `yosys -p "... hierarchy -top mem; proc"` and `... -top qspi_shared_engine; proc`:
  clean in isolation.
- `yosys -p "... hierarchy -top tt_um_agila32; proc"`: clean.
- `yosys -p "... synth -top tt_um_agila32"` (full generic synth, not
  just `proc`): completes, `CHECK` pass reports 0 problems.
- Full standalone suite (all 16 testbenches, including
  `tb_spi_periph.v` and `tb_qspi_clkdiv.v`): re-run after both fixes,
  **0 failures** -- confirms both changes are structural-only and
  don't alter simulated behavior.
- cocotb regression: not re-run in this environment (no `cocotb-config`
  available here); nothing in either fix touches simulation semantics,
  only how the RTL is packaged for synthesis, so no behavioral change
  is expected there either -- worth confirming in CI regardless.

Neither fix touches the register map, timing, or any externally
visible behavior -- both are purely about giving Yosys an RTL shape
it can lower to gates.
