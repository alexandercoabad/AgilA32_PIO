/* Load window 0xB4-0xDF (44 bytes). 0xB0-0xB3 is the stack (grows down from 0xB4). */
MEMORY { WIN (rx) : ORIGIN = 0xB4, LENGTH = 44 }
ENTRY(main)
SECTIONS {
  .text : { *(.text.start) *(.text.main) *(.text*) } > WIN
  /DISCARD/ : { *(.comment) *(.riscv.attributes) *(.eh_frame*) *(.data* .bss* .rodata*) }
}
