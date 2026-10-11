/* Optional startup (use only for programs that need a stack):
   build.sh uses it when STACK=1. Stack is 0xB0-0xB3 and grows down from 0xB4. */
    .section .text.start,"ax"
    .globl _start
_start:
    li   sp, 0xB4
    j    main
