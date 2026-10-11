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

Timing uses a free-running counter, `(c++ >> SLOW)`; `SLOW` is 18 in `agila32.h` (a step of about 0.3-0.5 s at 24 MHz).
`pilipinas.hex` uses shift 12 (fast, for the browser simulator); `pilipinas_hardware.hex` uses shift 18 (about 0.45 s per character).

## Limits (from `src/mem.v`)

- Code and data share the 44-byte window 0xB4-0xDF, plus 4 bytes of RAM at 0xB0-0xB3.
- RV32I only: no multiply or divide, no libc, no interrupts. Keep programs register-only (the examples avoid the stack).
- Programs loaded over the GPIO bootloader are lost when the design is deselected or reset; load them again.
- For larger programs use the QSPI flash window (see `docs/info.md`, flash paging).

## What was tested

The example `.hex` files were run on the RTL compiled to WebAssembly in the browser simulator
(https://alexandercoabad.github.io/agila32/) and their output was checked against the expected LED and
7-segment patterns. They have **not** been run on silicon or an FPGA.
