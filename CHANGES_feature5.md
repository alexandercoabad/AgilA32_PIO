# Feature #5: `shared_ram` dual-port area merge -- investigated, not applicable

The roadmap's feature #5 is AgilA8's `shared_ram.v`: a single physical
128-byte array serving both DMEM (`dmem_addr`/`dmem_valid`/`dmem_we`)
and IMEM (`imem_addr`/`imem_valid`) roles, merged from two separate
predecessor arrays (`ram32.v` + `iram.v`) specifically because
`a8_core.v` has two genuinely separate physical bus interfaces --
fetch and data access are two distinct ports at the core level, and
without the merge they'd synthesize as real two-port memory even
though they're never active the same cycle (`shared_ram.v`'s own
header documents this measured directly: 5948 cells for two
independent address expressions vs. 3615/3707 for the two single-port
arrays it replaced -- 60% *more*, not less, until the fix explicitly
merged both callers into one address expression ahead of the array).

**This doesn't apply to AgilA32.** `rv32i_core.v` has exactly ONE
memory port -- `mem_addr`/`mem_wdata`/`mem_size`/`mem_we`/`mem_valid`/
`mem_ready`/`mem_rdata` -- used for both instruction fetch and data
access, sequenced through the core's own FSM (`FETCH` -> ... -> `MEM`
-> ...) rather than split into separate imem/dmem interfaces the way
`a8_core.v` is. `mem.v`'s `ram_words` array is already the one and
only physical on-chip RAM, serving both roles for free, by
construction -- and `mem.v`'s own header has said as much since before
this porting roadmap existed:

> "PC and mem_addr are the same bus here, so a loaded program is
> immediately both writable AND fetchable at the same address -- no
> separate IMEM/DMEM aliasing trick needed the way AgilA8's shared_ram
> requires."

There is no second array anywhere in this design to merge. The
roadmap entry's benefit ("cutting on-chip flip-flop cost by roughly
40% versus two separate arrays") is something AgilA32 already has, not
something still to port -- most likely the roadmap entry was carried
over from AgilA8's feature list without re-deriving it against
AgilA32's already-unified single-port core interface.

## False start: the straddling half-word write

Before reaching this conclusion, a different area-reduction idea was
pursued and abandoned -- worth recording since the RTL was fully
implemented and tested before the numbers ruled it out, and the
underlying lesson (measure before committing to a hypothesis about
*why* something costs area) is exactly what surfaces `shared_ram.v`'s
own real precedent as informative here, not a false analogy.

**Hypothesis:** `mem.v`'s straddling half-word (`SH` at
`ram_byte_off==3`) RAM write needs two *simultaneous* word-index
writes (`ram_words[ram_widx0]` and `ram_words[ram_widx1]` in the same
cycle), which was measured (generic yosys `synth`, `mem.v` alone,
`qspi_shared_engine` blackboxed) at ~353 extra cells (~5% of the
module's own logic) versus a version with that capability removed
entirely. The proposed fix: serialize the straddling write into two
single-word-index cycles using the existing `ready`/wait-state
mechanism (the same pattern already used for external QSPI accesses),
on the theory that avoiding the *simultaneous* write would avoid
needing a second write port.

**What actually happened:** the serialization was implemented
correctly (verified against a dedicated 13-check testbench,
`tb_mem_straddle.v`, plus zero regressions across the existing
14-testbench suite) -- but a full generic yosys `synth`+`abc` pass
showed **no area reduction at all**:

| Version | Cells |
|---|---|
| Baseline (simultaneous dual-index write, the shipped version) | 6639 |
| Serialized over 2 cycles, still 2 separate indexed LHS expressions | 6645 |
| Serialized *and* funneled through one shared, explicitly-muxed index | 6646 |
| Ablation: capability removed entirely | 6278 |

The ablation confirms the original ~353-cell hypothesis almost exactly
(6639 - 6278 = 361 cells, ~5.4%) -- but neither serialized version
recovers any of it. The cost isn't from writing two indices
*simultaneously*; it's from the RTL needing to be able to select
`ram_widx1` as a write target *at all*. That comparator/select logic
has to exist in the netlist whether it fires on the same clock edge as
the `ram_widx0` write or a different one -- serializing changes *when*
the second write happens, not *whether* the capability's hardware
exists, so it can't reduce area on its own. Explicitly forcing both
writes through one shared muxed index (hoping Yosys would build one
decoder instead of two) made no difference either -- a full `synth`+
`abc` pass already does that sharing/optimization on its own,
regardless of source-level structure, once `mem2reg` has already
expanded the array into individual per-word flip-flops+muxes during
`proc` (confirmed via `dump`: `memory_collect` finds zero real `$mem`
cells for `ram_words`, meaning the "port count" framing this
hypothesis was built on doesn't even apply at that stage the way it
does for a genuinely-inferred multi-port memory).

This is the useful contrast with `shared_ram.v` above: AgilA8's
`shared_ram.v` gets a *real*, confirmed win from a shared-index
refactor because `mem[]` there is a true 128-entry inferred memory
(no `mem2reg`), where write-port count is a real, coarse structural
property `memory_collect`/`memory_share` reason about directly. For
`mem.v`'s 12-word `ram_words` -- deliberately kept as `mem2reg`
individual flip-flops instead, per that array's own header history
(measured cheaper than genuine memory inference at this small size) --
that same framing doesn't hold, and a full logic-optimization pass
already finds whatever sharing is actually available regardless of
how the RTL is structured. Reverted; not shipped. `mem.v` is unchanged
from before this investigation.

Recovering the real ~5% would require *removing* the capability (e.g.
requiring `SH` stores to be 2-byte aligned, the way `SW` already must
be 4-byte aligned) -- a breaking change to the documented "byte/half
accesses are supported at any address" contract, needing real
code-generation-side changes (`tools/asm_pineapple.py`, the boot ROM,
every existing flash image builder) to guarantee no straddling `SH` is
ever emitted. Not pursued -- out of scope for what was meant to be an
area-neutral timing restructuring, not an ISA-compatibility change.

## Verification

- `tb_mem_straddle.v` and the serialized `mem.v` changes were both
  fully reverted; `git diff` against the pre-investigation `mem.v` is
  empty.
- Full standalone suite (15/15 testbenches) re-run against the
  reverted, unchanged `mem.v` to confirm a clean baseline going into
  this conclusion -- no regressions, nothing left half-applied.

## Conclusion

Feature #5 is complete by construction, not "planned." No RTL, test,
or address-map change accompanies this file -- this is a documentation-
only update recording that finding (and the false start that preceded
it) for anyone re-reading the roadmap later.

## What's next

None remaining on the prioritized porting roadmap -- all five AgilA8
features have now either been ported (1-4) or confirmed already
present by AgilA32's existing architecture (5).
