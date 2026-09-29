# Sample testbench for a Tiny Tapeout project

This is a sample testbench for a Tiny Tapeout project. It uses [cocotb](https://docs.cocotb.org/en/stable/) to drive the DUT and check the outputs.
See below to get started or for more information, check the [website](https://tinytapeout.com/hdl/testing/).

## Setting up

1. Edit [Makefile](Makefile) and modify `PROJECT_SOURCES` to point to your Verilog files.
2. Edit [tb.v](tb.v) and replace `tt_um_example` with your module name.

## How to run

To run the RTL simulation:

```sh
make -B
```

To run gatelevel simulation, first harden your project and copy `../runs/wokwi/results/final/verilog/gl/{your_module_name}.v` to `gate_level_netlist.v`.

Then run:

```sh
make -B GATES=yes
```

If you wish to save the waveform in VCD format instead of FST format, edit tb.v to use `$dumpfile("tb.vcd");` and then run:

```sh
make -B FST=
```

This will generate `tb.vcd` instead of `tb.fst`.

## Additional standalone testbenches

Fifteen extra testbenches cover the QSPI external-memory addition, the
reprogrammable boot ROM (self-test + demo/listen loop + bootloader),
the boot-timeout automatic flash fallback, the FLASH_MODE handoff to
external flash, FLASH_PAGE bank-switched flash execution, a real
ST7789 LCD driver built on top of it, a PS/2 keyboard reader (raw
scancodes, then scancode-to-ASCII translation) built the same way, a
standalone ALU/instruction-encoding check for
`tools/asm_pineapple.py` itself, the Timer/PWM peripheral (counter
enable/reset/overflow, PWM duty cycle at 0x00/0x80/0xFF, and the
PIN_MUX wiring onto uo_out[7]), the EBREAK-halts-the-core behavior
(FSM parks in ST_HALTED, `halted` output, and PIN_MUX's halted-status
mode), the variable SPI clock divider (QSPI_CTRL's four SCK rates,
latch-at-accept-time, end-to-end through mem.v), and the generic SPI
peripheral (SPI_DATA's raw-byte framing and its no-hardware-retrigger
plain-read behavior, engine-level and end-to-end through mem.v).
They're plain Icarus testbenches, not cocotb, so they don't run as
part of `make` above -- run them together with:

```sh
make standalone-tests
```

which is also its own step in `.github/workflows/test.yaml`, so it gates
CI same as the cocotb suite. Individually, that target runs:

```sh
# QSPI engine bit-level protocol check (write bitstream, byte order, CS behavior)
iverilog -g2012 -o /tmp/tb1.vvp ../src/qspi_shared_engine.v tb_qspi_engine.v && vvp /tmp/tb1.vvp

# mem.v integration test: bus signals driven directly, through the real engine + a behavioral SPI RAM model
iverilog -g2012 -I ../src -o /tmp/tb2.vvp ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_mem_ext.v && vvp /tmp/tb2.vvp

# Full core-level test: the real rv32i_core executing actual RV32I load/store
# instructions against the external window (mem_extmem_test.v swaps in a small
# test program in place of the boot ROM)
iverilog -g2012 -I ../src -o /tmp/tb3.vvp ../src/rv32i_core.v ../src/qspi_shared_engine.v mem_extmem_test.v spi_ram_model.v tb_core_ext.v && vvp /tmp/tb3.vvp

# Full top-level test: self-test fail/pass (with and without a simulated QSPI
# slave) and a full bootload-and-run, all against the real tt_um_agila32
iverilog -g2012 -I ../src -o /tmp/tb4.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_check.v && vvp /tmp/tb4.vvp

# FLASH_MODE handoff: bootload the 1-instruction stub, confirm execution
# actually continues from the correct external flash byte afterward
iverilog -g2012 -I ../src -o /tmp/tb5.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_flash_handoff.v && vvp /tmp/tb5.vvp

# FLASH_PAGE bank-switching: a 4-page flash image (built by
# tools/build_flash_pagetest.py) exercises both switch_to() (a
# compile-time-constant page target) and switch_to_computed() (a
# runtime-decided one, via a self-loop), confirming GPIO_OUT visits
# every page's value in the right order
iverilog -g2012 -I ../src -o /tmp/tb6.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_flash_paging.v && vvp /tmp/tb6.vvp

# Real ST7789 LCD driver (tools/build_st7789_flash_image.py), simulation-sized:
# reconstructs the actual bit-banged SPI byte stream from GPIO_OUT and checks
# it against st7789_expected_seq.hex (regenerate that if the driver's init
# sequence or FILL_COLOR/panel constants change)
iverilog -g2012 -I ../src -o /tmp/tb7.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_st7789_driver.v && vvp /tmp/tb7.vvp

# PS/2 keyboard reader (tools/build_ps2_reader.py), a FLASH_PAGE
# bank-switched state machine like the LCD driver above (one page per
# protocol phase instead of one page per bit): drives ui_in[3]/ui_in[4]
# with real bit-banged PS/2 frames (start/8 data bits LSB-first/odd
# parity/stop) across several distinct scancodes and confirms GPIO_OUT
# lands on each one in turn
iverilog -g2012 -I ../src -o /tmp/tb8.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_ps2_reader.v && vvp /tmp/tb8.vvp

# PS/2 keyboard reader, Step 2: scancode -> ASCII translation
# (tools/build_ps2_ascii.py). Reuses tb_ps2_reader.v's bootload/frame-
# bit-banging tasks; checks GPIO_OUT against the TRANSLATED ASCII value
# rather than the raw scancode, and specifically exercises: a plain
# make code, a make+break pair (exactly one emit, the break itself
# produces none), an extended (0xE0-prefixed) make+break pair (fully
# consumed, no emit), an unmapped key (0x00), several plain keys back-
# to-back (state machine keeps re-arming), and a scancode above
# MAX_SAFE_SCANCODE -- proving the FLASH_PAGE 8-bit bounds check in
# PROCESS_DISPATCH actually prevents the page-number wraparound bug it
# exists for (an earlier version without it wrapped scancode 0xFF onto
# EMIT_PAGE's own page number and re-emitted a stale ASCII value from
# the previous real translation instead of 0x00)
iverilog -g2012 -I ../src -o /tmp/tb9.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_ps2_ascii.v && vvp /tmp/tb9.vvp

# tools/asm_pineapple.py instruction-encoding check: every opcode added
# beyond the original handful (SUB, SLL/SRL/SRA, SLT/SLTU, SLTI/SLTIU,
# XORI, SRAI, LB/LH/LHU, SH, BGE/BLTU/BGEU, LUI/AUIPC) run through the
# real rv32i_core (against a minimal flat ROM+RAM harness, not mem.v --
# nothing here needs the external-QSPI machinery) and checked against
# hand-computed expected register values. Test inputs are deliberately
# chosen so signed/unsigned and logical/arithmetic variants of the same
# opcode pair disagree on the given value (-1 == 0xFFFFFFFF throughout)
# -- a wrapper with the wrong funct3/funct7 is expected to fail loudly
# here, not coincidentally pass.
iverilog -g2012 -I ../src -o /tmp/tb10.vvp ../src/rv32i_core.v alu_test_mem.v tb_alu_test.v && vvp /tmp/tb10.vvp

# Timer/PWM peripheral (0xF2/0xF3/0xF5/0xF6/0xF7/0xF9/0xFA), ported
# from AgilA8's a8_peripherals.v: drives mem.v's register bus directly
# to check the free-running 16-bit timer (enable, write-1-to-reset,
# overflow flag on 0xFFFF->0x0000 wraparound, clear-on-any-write), the
# 8-bit free-running PWM generator at duty 0x00/0x80/0xFF and with
# PWM_CTRL disabled, then confirms against the real tt_um_agila32 top
# level that the 2-bit PIN_MUX actually selects between LED_OUT[7],
# the PWM waveform, the halted status, and (reserved) LED_OUT[7] again
# on uo_out[7]
iverilog -g2012 -I ../src -o /tmp/tb11.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v tb_timer_pwm.v && vvp /tmp/tb11.vvp

# EBREAK-halts-the-core, ported from AgilA8's a8_core.v S_HALTED/
# `halted` pattern: Part 1 drives rv32i_core directly against a tiny
# 4-instruction hand-built ROM (two ADDIs, an EBREAK, then a trailing
# ADDI that must never execute) and confirms `halted` goes high
# exactly on EBREAK, stays high indefinitely with no further memory
# activity, and clears on reset. Part 2 bootloads the identical
# program into the real tt_um_agila32 top level over the GPIO
# DATA/CLOCK/START protocol and confirms PIN_MUX=2'b10 surfaces the
# real bootloaded-and-halted core's status on uo_out[7]
iverilog -g2012 -I ../src -o /tmp/tb12.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_ebreak_halt.v && vvp /tmp/tb12.vvp

# Boot-timeout automatic flash fallback, ported from AgilA8's boot_rom:
# Part 1 leaves ui_in[2] (START) low forever -- no bootload attempted
# at all -- and confirms flash (CS0) stays untouched through the early
# part of the wait, FLASH_MODE latches on its own once the boot ROM's
# free-running demo/timeout counter crosses TIMEOUT_SHIFT, and the
# unattended chip falls through to run a tiny canary program preloaded
# into external flash (uo_out=0x2A). Part 2 confirms a host that DOES
# assert START well before the timeout still gets a completely normal
# RAM bootload (same 5-instruction program tb_check.v's scenario 3
# uses), unaffected by the new fallback path.
iverilog -g2012 -I ../src -o /tmp/tb13.vvp ../src/tt_um_agila32.v ../src/rv32i_core.v ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_boot_timeout.v && vvp /tmp/tb13.vvp

# Variable SPI clock divider (QSPI_CTRL, 0xFB), ported from AgilA8's
# SPI_CTRL clock-divider field: Part 1 drives qspi_shared_engine
# directly and confirms each req_div_sel setting's SCK half-period (1/
# 4/16/64 clk cycles, i.e. clk/2, clk/8, clk/32, clk/128) and whole-
# transaction cycle count exactly match a derived golden formula --
# including that req_div_sel=0 reproduces the engine's original fixed-
# speed timing bit-for-bit -- and that changing req_div_sel mid-flight
# never perturbs a transaction already in progress (latched once, at
# accept time). Part 2 drives mem.v directly (through the real engine
# and two behavioral SPI RAM models) and confirms QSPI_CTRL itself
# resets to 2'd3 (slowest), reads back what's written, and that a
# write actually reaches the engine and changes real external-access
# timing end to end in both directions, with the same mid-flight-
# immunity guarantee holding through the register path too.
iverilog -g2012 -I ../src -o /tmp/tb14.vvp ../src/mem.v ../src/qspi_shared_engine.v spi_ram_model.v tb_qspi_clkdiv.v && vvp /tmp/tb14.vvp

# Generic SPI peripheral (SPI_DATA, 0xFD), ported from AgilA8's
# spi_ctrl.v. Part 1 drives qspi_shared_engine directly (req_dev=2'd0)
# and confirms a raw byte gets exactly 8 bits on the wire (no cmd/addr
# framing at all), MOSI-first, with only CS2 asserted, and that a
# write's simultaneously-captured MISO byte round-trips correctly.
# Part 2 drives mem.v directly (with a standing "echo slave" process
# on MISO, since Part 2 doesn't sequence a fixed miso-drive task the
# way Part 1 does) and confirms SPI_DATA resets to 0x00, a plain read
# never touches hardware at all (zero wait cycles, zero SCK pulses),
# a write is a real 8-SCK-pulse transfer that leaves flash/RAM-A CS
# untouched, and -- the actual point of this port -- repeated reads
# after a write keep returning the same captured byte for free, never
# re-triggering a transfer.
iverilog -g2012 -I ../src -o /tmp/tb15.vvp ../src/mem.v ../src/qspi_shared_engine.v tb_spi_periph.v && vvp /tmp/tb15.vvp
```

None of these has been checked against a real flash/PSRAM chip or a
vendor-accurate behavioral model -- see `docs/info.md`'s "Known
limitation" note.

All 11 cocotb tests (run via plain `make`) currently pass, including
`test_selftest_passes_with_pmod` and
`test_selftest_passes_again_after_soft_reset`, which drive a *Python*
behavioral QSPI slave rather than the Verilog one the standalone
testbenches above use.

## How to view the waveform file

Using GTKWave

```sh
gtkwave tb.fst tb.gtkw
```

Using Surfer

```sh
surfer tb.fst
```
