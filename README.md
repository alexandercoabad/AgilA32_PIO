![](../../workflows/gds/badge.svg) ![](../../workflows/docs/badge.svg) ![](../../workflows/test/badge.svg) ![](../../workflows/fpga/badge.svg)

# AgilA32 — a from-scratch RV32I CPU for Tiny Tapeout (IHP SG13CMOS5L shuttle)

> **In one line:** a from-scratch RV32I CPU with RP2040-compatible PIO state
> machines on the same die, so the CPU can load, *and later replace*, the
> protocol engines at run time -- and can then halt itself while the PIO
> keeps running the protocol on its own.

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

## What is novel here (Jane Street protocol-emulator competition)

The brief asks for a chip that supports new protocols *after* fabrication,
and to say what the architecture makes possible beyond the RP2040's PIO.
Here is what this design does, with the test that backs each claim.

**1. The programmer lives on the die.** RP2040 PIO is driven by an external
Cortex-M0+. Here a from-scratch RV32I core drives the PIO over a two-byte
window (`PIO_IDX`/`PIO_DATA`, with auto-increment) squeezed into an 8-bit
address space. No host microcontroller is needed: a flash image or the GPIO
bootloader is enough to bring the whole system up.
(`tools/pio_host.py`, docs/info.md "PIO")

**2. One state machine, three protocols, one image.** `tools/build_pio_multi.py`
builds a single flash image whose firmware reprograms the *same* PIO state
machine at run time: UART TX, then SPI mode 0, then I2C. Between phases it
disables and restarts the SM, overwrites instruction memory, and
reconfigures pins, shifting and clock divider. Pin hand-over is glitch-free
(UART idles high before it owns the pad, SCK is preset low, I2C pins are
handed over released). This is the "support new protocols after fabrication"
requirement demonstrated, not just claimed.
(`test/tb_pio_cpu_multi.v` checks all three waveforms against a UART
receiver, an SPI slave and an I2C slave, and that the phases ran strictly in
sequence.)

**3. The CPU can leave; the protocol keeps going.** After queueing its last
word, the firmware executes `EBREAK`, which parks the core. The PIO finishes
the transfer alone -- in the I2C demo it generates the STOP condition with the
CPU halted. Protocol timing is therefore independent of firmware timing, and
the CPU can be halted for power or debugging without corrupting a transfer.
(`test/tb_pio_cpu_uart.v`, `test/tb_pio_cpu_i2c.v`)

**4. Stock Pico SDK programs run unmodified.** The PIO executes the RP2040
instruction set (JMP WAIT IN OUT PUSH/PULL MOV IRQ SET, side-set, delays,
autopush/autopull, IRQ flags, 16.8 fractional divider). Pico SDK `uart_tx`,
`uart_rx_mini` and `spi_cpha0` run as-is on pins 0-9, and `tools/pioasm.py`
assembles stock `pioasm` syntax, so existing PIO programs are a starting
point rather than something to rewrite.

**5. Open-drain buses on a real bidirectional pad.** Pins 8-9 (`uio[4]`,
`uio[5]`) use `PINDIR` as the true output enable, which is what makes I2C
possible. The polarity differs from the RP2040 (`PINDIR = 1` pulls the pad
low), so `pio/i2c.pio` has its side-set values swapped and
`tools/pio_i2c.py` complements the pindir bits it sends. Pins are only taken
from the LED/QSPI logic when their `PIN_OWN` bit is set, so reset behaviour
is identical to the pre-PIO chip and every earlier test still passes
unchanged.

**6. Verification against protocol peers, not just waveforms.**
- 16 cocotb protocol tests (`test/test_pio_protocols.py`, `make -f
  Makefile.proto`) drive `pio.v` over the same bus the CPU uses, against
  cycle-accurate peer models (`test/pio_tb_lib.py`): UART TX at integer,
  fractional and averaged dividers, UART RX including framing error and baud
  tolerance, two-SM UART loopback, SPI master modes 0 and 1 (plus fast SCK
  with `SYNC_BYP`), and I2C write, read, repeated-start register read, NAK
  raising IRQ 0, and clock stretching.
- The I2C slave model flags any SDA change while SCL is high, so START/STOP
  conditions are checked as protocol events, not just as edges.
- `tb_pio_isa.v` checks the semantics of every PIO instruction. While
  building it, three WAIT encodings in the supplied testbench turned out to
  have the wrong source field; they are fixed and documented in
  `CHANGES_feature6.md`.
- Gate-level tests run on the hardened netlist in CI (11/11 passing).

**7. Designed for the synthesis flow, not just for simulation.** The first
CI synthesis of the PIO block stalled in Yosys' SAT-based `share` pass on the
variable-amount `<<`/`>>` operators in `pio_sm.v`. `pio_sm.v` now contains no
variable shifts: rotates, masks and 32-bit shifts are fixed-stage mux
shifters, with behaviour unchanged (all PIO testbenches still pass). The
`share` pass dropped from 145 analyses to 5 locally (28 s to 6 s) for about
+2% generic gates. `tools/sta.py` gives a quick pre-layout register-to-
register timing estimate from a Yosys JSON netlist and a liberty file.

**Honest limits.** Two state machines (the RP2040 has eight), a shared
32-word instruction memory, pins 0-9 only, and a 1 MHz clock in `info.yaml`.
The CPU costs about 3000 clock cycles per queued word (flash paging), so a
bus must be slower than that per byte for firmware to stay ahead of it -- the
FIFOs and the halt-and-continue behaviour above are how this is worked
around. UART, SPI (modes 0/1) and I2C master are demonstrated. Low-speed USB
and 10BASE-T (the brief's stretch goals) are not attempted here, and nothing
has been measured on silicon yet.

## Layout

<img width="909" height="507" alt="Screenshot 2026-09-30 at 9 55 39 AM" src="https://github.com/user-attachments/assets/dd35fa5a-1f16-4868-9cc4-c33615c759e6" />



https://gds-viewer.tinytapeout.com/?model=https://alexandercoabad.github.io/AgilA32_PIO/tinytapeout.oas&pdk=ihp-sg13g2




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
- [x] **Hardened on the IHP CMOS5L flow (current target)** at 6x4 tiles with
      the PIO block: 51.45% routing utilization, 34,053 cells (excluding
      fill/tap), clean lint, precheck 10/10 and gate-level tests 11/11
      (CI run #11, ~2h28m for the `gds` job -- expect a long build)
- [x] **PIO block (protocol emulator)**: two RP2040-compatible PIO state
      machines + shared 32-word instruction memory + FIFOs at
      `PIO_IDX`/`PIO_DATA` (`0xFF`/`0xFE`), pins 0-9 (`uo_out[7:0]`,
      `uio[5:4]`). UART TX/RX, SPI master (modes 0/1) and I2C master run as Pico SDK
      programs (`pio/`), assembled by `tools/pioasm.py`, loaded by
      `tools/pio_host.py` -- see docs/info.md's "PIO" section and
      `CHANGES_feature6.md`. Built for the Jane Street protocol-emulator
      ASIC competition (6x4 tiles).
- [x] Twenty test suites (see "Testing locally" below): on-chip
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
  pio.v, pio_sm.v, pio_fifo.v   PIO block: host registers/pins/IRQs, one state machine, FIFO
  config.json             LibreLane flow config (clock period, density, etc.)
pio/
  uart_tx.pio, uart_rx.pio, spi_master.pio, spi_cpha1.pio, i2c.pio   Pico SDK programs (i2c: side-set polarity swapped)
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
  pioasm.py, pio_host.py, pio_i2c.py, build_pio_uart.py, build_pio_i2c.py, build_pio_multi.py   PIO assembler/disassembler, flash-image host library, UART demo
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
  tb_pio_isa.v, tb_pio_uart.v, tb_pio_spi.v, tb_pio_cpu_uart.v, tb_pio_cpu_i2c.v, tb_pio_cpu_multi.v, test_pio_protocols.py   PIO block tests (see docs/info.md)
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
