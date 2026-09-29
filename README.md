![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# AgilA32 — a from-scratch RV32I CPU for Tiny Tapeout (IHP SG13CMOS5L shuttle)

Originally inspired by [Pineapple ONE](https://pineapple-one.github.io/),
a RISC-V CPU built entirely out of discrete 7400-series logic chips (no
FPGA, no microcontroller) -- the "just basic logic" idea of a minimal,
from-scratch RISC-V core is where this whole line of projects started.

## Lineage: AgilA8 -> AgilA32

Structurally and feature-wise, though, AgilA32 is the direct successor
to [AgilA8](https://github.com/alexandercoabad/AgilA8_IHP) -- my own
earlier Tiny Tapeout project. Both are mine, and both belong to the
same "AgilA" line: *Agila* is Tagalog/Filipino for "eagle," and the
"A8" named that first core's 8-bit custom ISA. AgilA32 swaps that "A8"
for "A32" for exactly the reason it looks like: this is the 32-bit
RV32I successor to that 8-bit core, built under the same one-Tiny-
Tapeout-tile constraint AgilA8 already had to fit.

Most of what makes AgilA32 more than "just an RV32I core" is ported
from AgilA8 rather than designed fresh for this project:

- the shared QSPI engine driving flash + PSRAM (and now a generic SPI
  peripheral on CS2) off one state machine
- the GPIO bootloader protocol
- bank-switched flash execution
- the Timer/PWM peripheral -- register layout ported near-verbatim
  from AgilA8's `a8_peripherals.v`
- the generic SPI peripheral (`SPI_DATA`, `0xFD`) -- ported from
  AgilA8's `spi_ctrl.v`/CS2 front-end

The RV32I core itself -- the actual instruction set implementation --
is written fresh for this project (RV32I is a public, open
instruction-set standard, not something owned by either AgilA8 or
Pineapple ONE); it's the surrounding system AgilA32 needed to actually
be useful -- boot, memory-mapped I/O, external storage, peripherals --
that carries AgilA8's design forward, widened from 8 bits to 32.

- [Read the project datasheet](docs/info.md) — how it works, how to test it, pinout
- [AgilA8, the 8-bit predecessor this project is built on](https://github.com/alexandercoabad/AgilA8_IHP)
- [Original Pineapple ONE project](https://pineapple-one.github.io/) (the origin-story inspiration)

**Scope note:** the original design has a 500 kHz clock, 512 kB program
memory, 512 kB RAM, and a VGA card — none of which fits in a TT tile
(~167×108 µm). This project keeps the RV32I instruction set and the
"no FPGA, just logic" philosophy, starting from a 256-byte address
space with no native video output, plus a small external RAM window
over the [Tiny Tapeout QSPI Pmod](https://github.com/mole99/qspi-pmod)
for anyone who wants more headroom than the on-chip memory alone
gives. A bit-banged SPI LCD driver (ST7789) now runs over that same
Pmod's GPIO, and a PS/2 keyboard reader is next to it feeding the same
external address space — see "Status" below.

## Layout

<img width="1305" height="297" alt="Screenshot 2026-09-09 at 7 54 46 PM" src="https://github.com/user-attachments/assets/6945c66d-a44e-43ee-a7ae-b8bbbedcb1e7" />


https://gds-viewer.tinytapeout.com/?model=https://alexandercoabad.github.io/AgilA32/tinytapeout.oas&pdk=ihp-sg13cmos5l



## Status

- [x] Full RV32I base integer ISA (all loads/stores/branches/ALU ops;
      FENCE/ECALL/EBREAK decode as no-ops, no trap support yet)
- [x] Multi-cycle FSM core (fetch/fetch_wait/decode/exec/mem/mem_wait/
      writeback, 7 clock cycles per instruction -- the two `*_wait`
      states let the core stall on a `ready` handshake during external
      QSPI accesses; on-chip accesses see `ready` high immediately)
- [x] `tools/asm_pineapple.py` wraps every opcode the core above
      actually implements, not just the handful each script originally
      needed -- `SUB`, register-register `SLL`/`SRL`/`SRA`, `SLT`/
      `SLTU`, `SLTI`/`SLTIU`, `XORI`, `SRAI`, `LB`/`LH`/`LHU`, `SH`,
      `BGE`/`BLTU`/`BGEU`, and `LUI`/`AUIPC` are all now available to
      every script that builds on `Asm`/`PagedAsm`, confirmed against
      the real core (not just on paper) in `test/tb_alu_test.v`
- [x] Memory: 176 B combinational boot ROM (self-test + demo/listen
      loop + bootloader) + 48 B flip-flop RAM (4 B always on-chip
      scratch + 44 B bootloader-loadable/executable window, the latter
      redirectable to external flash via `FLASH_MODE`, and bank-
      switchable across that flash chip via `FLASH_PAGE`) + 16 B
      external PSRAM window over the QSPI Pmod + memory-mapped LED
      output (`0xF0`) / switch input (`0xF4`) / `FLASH_MODE` (`0xF8`) /
      `FLASH_PAGE` (`0xFC`)
- [x] **Timer/PWM peripheral**, ported from AgilA8's
      `a8_peripherals.v` (same bit layout/behavior): a free-running
      16-bit timer with enable/reset/overflow-flag registers
      (`TIMER_LO`/`TIMER_HI`/`TIMER_CTRL`/`TIMER_FLAG` at `0xF2`/
      `0xF3`/`0xF5`/`0xF6`) and an 8-bit free-running PWM generator
      (`PWM_DUTY`/`PWM_CTRL` at `0xF7`/`0xF9`), with the 2-bit
      `PIN_MUX` (`0xFA`) selecting whether `uo_out[7]` shows
      `LED_OUT[7]` (default), the PWM waveform, or the core's halted
      status -- see `test/tb_timer_pwm.v` and docs/info.md's
      "Timer / PWM" section
- [x] **EBREAK halts the core**, ported from AgilA8's `a8_core.v`
      `S_HALTED`/`halted` pattern: executing `EBREAK` parks the FSM in
      a new `ST_HALTED` state (no further fetches or memory activity)
      until the next reset, and the `halted` output it drives is what
      `PIN_MUX = 2'b10` exposes on `uo_out[7]` -- see
      `test/tb_ebreak_halt.v` and docs/info.md's "EBREAK halts the
      core" section
- [x] **Reprogrammable at runtime, no reflash/retapeout needed**: the
      boot ROM listens for a bootload request over `ui_in[0:2]`
      (DATA/CLOCK/START) and runs whatever program it receives
      straight out of on-chip RAM -- see `tools/build_boot_rom.py` and
      docs/info.md's "Reprogrammability" section
- [x] **Boot-timeout flash fallback**, ported from AgilA8's boot_rom:
      the listen loop above is bounded, not indefinite -- if START
      never comes, the boot ROM gives up on its own after a fixed
      number of iterations, sets `FLASH_MODE` itself, and falls into
      whatever's already sitting in external flash, so an unattended
      chip still boots something useful instead of blinking forever
      -- see `test/tb_boot_timeout.v` and docs/info.md's "Boot-timeout
      flash fallback" section
- [x] **Variable SPI clock divider**, ported from AgilA8's `SPI_CTRL`
      clock-divider field: `QSPI_CTRL` (`0xFB`) gates the shared QSPI
      engine's actual SCK rate (`clk`/2, `/8`, `/32`, `/128`), reset
      default is the slowest setting (matching AgilA8's own reset-safe
      default), sampled once per transaction so a mid-flight write
      can't corrupt a transfer already in progress -- `2'd0`
      reproduces the engine's original fixed-fast timing exactly, so
      nothing downstream needs to change to keep running as fast as
      before, it just isn't the default anymore -- see
      `test/tb_qspi_clkdiv.v` and docs/info.md's "Variable SPI clock
      divider" section
- [x] **Generic SPI peripheral (CS2)**, ported from AgilA8's
      `spi_ctrl.v`: `SPI_DATA` (`0xFD`) is a raw 8-bit transfer with no
      command/address framing, sharing CS2 with PSRAM "RAM B" -- **requires
      one board modification** (a trace cut on the QSPI Pmod, same as
      AgilA8's own CS2 always needed) before it can reach anything
      external; functionally inert until then -- see "General-purpose
      SPI (CS2)" below for the full caveat. An `SB` write clocks the
      byte out and simultaneously captures MISO; a plain `LBU` read
      returns the captured byte immediately, with zero wait cycles and
      no hardware retrigger, exactly AgilA8's `spi_ctrl.v` behavior --
      see `test/tb_spi_periph.v` and docs/info.md's "Generic SPI
      peripheral" section
- [x] **Bank-switched flash execution**: a bootloaded 1-instruction
      stub can hand off into external flash (`FLASH_MODE`), and
      `FLASH_PAGE` lets a running program page through a flash image
      far larger than the 44-byte on-chip execute window -- see
      docs/info.md's "Bank-switched flash execution" section and
      `tools/asm_pineapple.py`'s `PagedAsm`
- [x] **Real ST7789 LCD driver** (`tools/build_st7789_flash_image.py`),
      bit-banged SPI over `GPIO_OUT` (no dedicated SPI peripheral),
      built entirely on `PagedAsm` -- init sequence + fill loop
      verified byte-for-byte in `test/tb_st7789_driver.v`
- [x] **PS/2 keyboard reader**, bit-banged over two free `ui_in` pins:
      Step 1 (`tools/build_ps2_reader.py`) reads raw scancodes onto
      `GPIO_OUT`; Step 2 (`tools/build_ps2_ascii.py`) adds scancode-to-
      ASCII translation, including make/break-code and extended-code
      (`0xE0`) handling
- [x] **Hardened on the Sky130 shuttle CI (previous target)** at 6x2
      tiles, 53.8% routing utilization, 12,548 cells (excluding
      fill/tap), clean DRC/precheck (15/15 checks) and gate-level tests
      (11/11) -- see `.github/workflows/gds.yaml` run history
- [x] Sixteen test suites (see "Testing locally" below): on-chip
      cocotb regression (self-test, demo counter, full bootload-and-run)
      plus fifteen standalone Icarus testbenches -- QSPI engine
      bit-level protocol, external-window integration via direct bus
      driving, full CPU-driven external load/store, self-test/bootload,
      `FLASH_MODE` handoff to external flash, `FLASH_PAGE`
      bank-switched flash execution, the ST7789 LCD driver, the PS/2
      reader (raw scancodes, then scancode-to-ASCII translation), an
      instruction-encoding check for every opcode
      `tools/asm_pineapple.py` wraps, the Timer/PWM peripheral, the
      EBREAK-halt behavior, the boot-timeout flash fallback, the
      variable SPI clock divider, and the generic SPI peripheral
      (`test/tb_timer_pwm.v`, `test/tb_ebreak_halt.v`,
      `test/tb_boot_timeout.v`, `test/tb_qspi_clkdiv.v`,
      `test/tb_spi_periph.v`) -- all wired into CI, all gating the
      build, all 11 cocotb tests + all 15 standalone tests currently
      passing
- [ ] **Step 3, in progress:** a bitmap font + terminal renderer tying
      the PS/2 reader to the ST7789 driver, so keystrokes actually
      appear on screen -- the biggest piece yet; no interrupts on this
      core, so keyboard polling and display draws have to be
      interleaved cooperatively by the same program, not preempted
- [ ] Validate the external memory path against a real flash/PSRAM chip
      or a vendor-accurate behavioral model (currently only tested
      against a hand-written behavioral model, `test/spi_ram_model.v`)
- [ ] Widen the address bus beyond 8 bits to actually reach the QSPI
      Pmod's real multi-megabyte capacity (current external window is
      a fixed 64 bytes within the existing 256-byte address space)

## Repo layout

```
src/
  rv32i_defs.vh          opcode/state constants
  rv32i_core.v            the CPU: regfile, ALU, decode, control FSM
  mem.v                   boot ROM + RAM + external QSPI windows + LED/switch/FLASH_MODE/FLASH_PAGE registers
  boot_rom_body.vh        generated boot ROM bytes, `include`d by mem.v -- don't hand-edit
  qspi_shared_engine.v    single-line SPI master shared between flash (CS0) and PSRAM (CS1)
  tt_um_agila32.v   Tiny Tapeout top-level pin mapping (incl. QSPI Pmod pins on uio)
  config.json             LibreLane flow config (clock period, density, etc.)
tools/
  build_boot_rom.py       assembles the boot ROM (self-test + demo/listen loop + bootloader)
                          into src/boot_rom_body.vh -- run this and re-copy its output if you
                          change what the boot ROM itself does
  asm_pineapple.py         RV32I assembler (Asm, wrapping every opcode the core implements) +
                          PagedAsm, the FLASH_PAGE bank-switching helper every *_flash_image.py
                          builder below shares
  build_flash_handoff_stub.py  1-instruction stub bootloaded over ui_in to hand off into flash
  build_flash_canary.py    tiny known-good flash program, for confirming the handoff mechanism alone
  build_flash_pagetest.py  synthetic 4-page program exercising switch_to()/switch_to_computed()
  build_st7789_flash_image.py  real ST7789 LCD driver, PagedAsm-based, bank-switched
  build_ps2_reader.py      PS/2 keyboard reader, Step 1: raw scancode -> GPIO_OUT
  build_ps2_ascii.py       PS/2 keyboard reader, Step 2: scancode -> ASCII translation
  build_alu_test.py        standalone program exercising every asm_pineapple.py opcode, for tb_alu_test.v
test/
  tb.v, test.py           cocotb testbench: self-test pass/fail, demo counter, full bootload-and-run
  tb_check.v              standalone: same three scenarios as a single self-contained Icarus testbench
  tb_qspi_engine.v        standalone: QSPI engine bit-level protocol + byte-order check
  spi_ram_model.v         behavioral single-line SPI RAM model (flash CS0 and PSRAM CS1), for the tests below
  tb_mem_ext.v            standalone: external window via direct bus driving + real engine + spi_ram_model
  mem_extmem_test.v       copy of mem.v with a test program in place of the boot ROM
  tb_core_ext.v           standalone: the real CPU running that test program against the external window
  tb_flash_handoff.v      standalone: bootloaded stub hands off into a small flash-resident program
  tb_flash_paging.v       standalone: FLASH_PAGE bank-switching across a synthetic multi-page program
  tb_st7789_driver.v      standalone: the real ST7789 driver, byte stream reconstructed and checked
  tb_ps2_reader.v         standalone: raw PS/2 frames -> GPIO_OUT (Step 1)
  tb_ps2_ascii.v          standalone: PS/2 frames -> translated ASCII on GPIO_OUT (Step 2)
  tb_qspi_clkdiv.v        standalone: QSPI_CTRL clock-divider timing, engine-level and through mem.v
  tb_spi_periph.v         standalone: generic SPI peripheral (SPI_DATA, CS2), engine-level and through mem.v
  alu_test_mem.v          minimal flat ROM+RAM harness (not mem.v) used only by tb_alu_test.v
  tb_alu_test.v           standalone: every asm_pineapple.py opcode, run through the real core, checked
                          against hand-computed register values
info.yaml                 Tiny Tapeout project metadata (title, pinout, tiles...)
docs/info.md              project datasheet shown on the Tiny Tapeout site
```

## How the boot ROM works

On every reset, the boot ROM self-tests the external QSPI PSRAM
(result latched into `uo_out[7]`), then loops forever incrementing a
demo counter into `uo_out[3:0]` while listening on `ui_in[0:2]` for a
bootload request -- assert START and stream over a new program at any
time, and the chip runs it immediately out of RAM, no reflash or
retapeout needed. Full protocol, address map, pinout, and the
`FLASH_MODE`/`FLASH_PAGE` opt-in for booting from (and paging through)
external flash are in [docs/info.md](docs/info.md).


## Testing locally

```
cd test
pip install -r requirements.txt
make                    # cocotb: self-test, demo counter, full bootload-and-run
make standalone-tests   # QSPI engine, clock divider, generic SPI peripheral, external-window, full-CPU,
                         # self-test/bootload, FLASH_MODE handoff, FLASH_PAGE bank-switching, ST7789 driver,
                         # PS/2 reader/ASCII, and asm_pineapple.py instruction-encoding tests
```

Both targets are also run automatically by `.github/workflows/test.yaml`
on every push, and both must pass for that workflow to go green.

## External memory over QSPI

`uio[0:7]` are wired to the
[Tiny Tapeout QSPI Pmod](https://github.com/mole99/qspi-pmod): a
single-line SPI master (`src/qspi_shared_engine.v`) shared between an
external flash chip (CS0) and PSRAM (CS1). PSRAM backs a 16-byte
external RAM window at `0xE0-0xEF` in `mem.v`, always -- this is also
what the boot ROM's own power-on self-test probes. Flash only gets
selected once a *bootloaded* program writes `FLASH_MODE` (`0xF8`),
which redirects the 44-byte loadable window at `0xB4-0xDF` from
on-chip RAM to flash from then on; the boot ROM itself never asserts
CS0. From there, `FLASH_PAGE` (`0xFC`) lets a running program bank-
switch through a much larger flash image 44 bytes at a time -- see
`tools/asm_pineapple.py`'s `PagedAsm`, which the ST7789 driver and PS/2
reader both build on. The core's FSM stalls on a `ready` handshake
while an external access is in flight, and picks up exactly where it
left off once the SPI transaction completes — on-chip accesses are
unaffected and still get an immediate response. See `docs/info.md` for
the full address map and pinout.

This currently only uses a fixed handful of bytes of the Pmod's actual
multi-megabyte capacity at any one time (`FLASH_PAGE` reaches further
into flash by paging, not by widening the address space itself), since
the CPU's address bus is still 8 bits wide. Reaching the Pmod's real
capacity as a flat address space means widening `pc`/`mem_addr` and the
jump/branch immediate math throughout `rv32i_core.v` — a bigger
follow-up change, not yet done here.

### General-purpose SPI (CS2) - requires one board modification

A fourth front-end, `SPI_DATA` (`0xFD`) — a raw byte-oriented SPI
master intended for driving an external device (an LCD, an ADC,
another MCU) — shares the same physical lines using CS2, alongside
PSRAM "RAM B" (`PSRAM_BANK = 1`). **This requires one board
modification first**: on the stock QSPI Pmod, CS2 ("RAM B") is wired
directly to a second, populated PSRAM chip, not out to any external
connector pin. Per the Pmod's own documentation
([mole99/qspi-pmod](https://github.com/mole99/qspi-pmod)), each of its
three chip-select traces can be cut on the back of the board — doing
so for CS2 disables that second PSRAM chip (a 1k pull-up holds its
`/CS` disabled) and makes the pad available via a through-hole header
pin as a plain input or output. That's a documented, intended
modification on the board as sold, not a custom respin — and it
leaves flash (CS0) and PSRAM RAM A (CS1) untouched, so the external
flash window and the boot ROM's own self-test are unaffected either
way.

Until that trace is cut, this peripheral is functionally inert: CS2
still selects the live RAM B chip, so `SPI_DATA` transfers just talk
to that PSRAM with the wrong command protocol rather than reaching any
external device. A program should only ever use one of `PSRAM_BANK =
1` or `SPI_DATA`, never both, depending on what's actually populated
on its particular board — see `qspi_shared_engine.v`'s header and
docs/info.md's "Generic SPI peripheral" section for the full
explanation.

If you don't want to modify the board (or just want the simplest path
for something slow enough that bit-banging is a non-issue, like an
e-paper display), drive the external device over the GPIO pins in
software instead — `uo_out[6:0]` and `ui_in[7:0]` are on a separate
header from the QSPI Pmod's `uio` bus entirely, so they aren't
affected by any of the above either way. The ST7789 driver and PS/2
reader both already do exactly this (see "How the boot ROM works"
above and docs/info.md).

## What is Tiny Tapeout?

Tiny Tapeout is an educational project that aims to make it easier and cheaper than ever to get your digital and analog designs manufactured on a real chip.

To learn more and get started, visit https://tinytapeout.com.

## Resources

- [FAQ](https://tinytapeout.com/faq/)
- [Digital design lessons](https://tinytapeout.com/digital_design/)
- [Learn how semiconductors work](https://tinytapeout.com/siliwiz/)
- [Join the community](https://tinytapeout.com/discord)
- [Build your design locally](https://www.tinytapeout.com/guides/local-hardening/)

## What next?

- [Submit your design to the next shuttle](https://app.tinytapeout.com/).
- Share your project on your social network of choice:
  - LinkedIn [#tinytapeout](https://www.linkedin.com/search/results/content/?keywords=%23tinytapeout) [@TinyTapeout](https://www.linkedin.com/company/100708654/)
  - Mastodon [#tinytapeout](https://chaos.social/tags/tinytapeout) [@matthewvenn](https://chaos.social/@matthewvenn)
  - X (formerly Twitter) [#tinytapeout](https://twitter.com/hashtag/tinytapeout) [@tinytapeout](https://twitter.com/tinytapeout)
  - Bluesky [@tinytapeout.com](https://bsky.app/profile/tinytapeout.com)