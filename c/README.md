# Programming AgilA32 in C (and assembly)

AgilA32 is a plain RV32I core, so any RISC-V C compiler can target it. This folder holds a small, tested flow for the
GPIO bootloader's 44-byte program window, with examples.

## Build

Needs `pip install ziglang` (clang) plus `ld.lld`, `llvm-objcopy` and `llvm-objdump` from LLVM:

    cd c
    ./build.sh examples/blink.c         # writes examples/blink.elf / .bin / .hex / .lst
    STACK=1 ./build.sh prog.c           # adds start.S (sp = 0xB4, stack in the 4 bytes at 0xB0-0xB3)

`build.sh` compiles with `-target riscv32-freestanding -mcpu=generic_rv32 -Os -ffreestanding -nostdlib`, links with
`link.ld` (code and data at 0xB4, length 44 bytes, entry `main`) and prints the size against the 44-byte limit.
The `.hex` file is the byte string the GPIO bootloader expects (the sender adds the length byte; see
`boot_send_program` in `test/test.py`). The `.hex`/`.lst` files of the examples are committed so they can be used without the toolchain.

## Examples

| File | What it does | Bytes |
|---|---|---|
| `examples/blink.c` | blinks LED0 | 44 or fewer, see `.lst` |
| `examples/counter.c` | binary counter on the LEDs | |
| `examples/chase.c` | one LED running along the row | |
| `examples/switches.c` | copies the input switches to the LEDs | |
| `examples/pilipinas.S` | scrolls PILIPINAS LASALLE on a 7-segment display (assembly, 44 bytes) | 44 |
| `examples/pilipinas_agila.S` | scrolls PILIPINAS AGILA32 on a 7-segment display (assembly, 44 bytes) | 44 |
| `examples/pilipinas_agila_flash.S` | the same text, run from the QSPI flash with no bootload | 44 |

Timing uses a free-running counter, `(c++ >> SLOW)`; `SLOW` is 18 in `agila32.h` (a step of about 0.3-0.5 s at 24 MHz).
`pilipinas.hex` uses shift 12 (fast, for the browser simulator); `pilipinas_hardware.hex` uses shift 18 (about 0.45 s per character).
`pilipinas_agila.hex` and `pilipinas_agila_hardware.hex` are the same for PILIPINAS AGILA32; `examples/make_pilipinas_agila.py` regenerates them and the flash images without a toolchain.

## Starting from flash at power-up

If the QSPI Pmod carries a flash chip and nothing drives `ui_in`, the boot ROM stops waiting for a bootload after a short timeout
(`TIMEOUT_SHIFT` in `tools/build_boot_rom.py`, about 1 ms at 24 MHz), sets `FLASH_MODE` and runs flash byte 0. Program
`examples/pilipinas_agila_flash.bin` (44 bytes, shift 14, about 0.6 s per character at 24 MHz) at flash address 0 and the 7-segment display
spells PILIPINAS AGILA32 by itself after power-up or reset. The program's first instruction sets the QSPI clock to the fastest
divider (the reset value, sys_clk/128, is too slow to run from), and the boot ROM leaves `gp` = 1, so the counter starts at the
first character. `pilipinas_agila_flash_shift12.bin` uses shift 12 (about 0.15 s per character) for quick checks; running from flash costs about 900 clocks per loop, so the flash images use smaller shifts than the RAM ones. In the browser
simulator (QSPI forced-fast box unticked) the first character appears about 2.3 million clocks, roughly 0.1 s at 24 MHz, after
reset, because the first flash reads run at the slow reset divider.


## Limits (from `src/mem.v`)

- Code and data share the 44-byte window 0xB4-0xDF, plus 4 bytes of RAM at 0xB0-0xB3.
- RV32I only: no multiply or divide, no libc, no interrupts. Keep programs register-only (the examples avoid the stack).
- Programs loaded over the GPIO bootloader are lost when the design is deselected or reset; load them again.
- For larger programs use the QSPI flash window (see `docs/info.md`, flash paging).

## What was tested

The example `.hex` files were run on the RTL compiled to WebAssembly in the browser simulator
(https://alexandercoabad.github.io/agila32/) and their output was checked against the expected LED and
7-segment patterns. They have **not** been run on silicon or an FPGA.
