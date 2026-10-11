#!/bin/sh
# usage: ./build.sh examples/blink.c  -> examples/blink.{elf,bin,hex,lst}
set -e
CC="python3 -m ziglang cc -target riscv32-freestanding -mcpu=generic_rv32 -Os -ffreestanding -nostdlib -fno-pic -fomit-frame-pointer -fno-stack-protector -I. $CFLAGS"
n=${1%.c}
$CC -c $1 -o $n.o
if [ "$STACK" = 1 ]; then $CC -c start.S -o $n.start.o; ld.lld -T link.ld $n.start.o $n.o -o $n.elf
else ld.lld -T link.ld $n.o -o $n.elf; fi
llvm-objcopy -O binary $n.elf $n.bin
od -An -v -tx1 $n.bin | tr -d ' \n' > $n.hex; echo >> $n.hex
llvm-objdump -d --no-show-raw-insn $n.elf > $n.lst
echo "$1: $(stat -c %s $n.bin) of 44 bytes"
