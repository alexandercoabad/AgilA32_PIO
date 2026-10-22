# PILIPINAS AGILA32 on the 7-segment display, run from the external QSPI flash (uo_out[6:0] = segments a-g, uo7 = dp).
# Program the 44 bytes of pilipinas_agila_flash.bin at flash address 0 on the QSPI Pmod (CS0) and power the chip with nothing
# driving ui_in: the boot ROM times out of its bootload wait (about 1 ms), sets FLASH_MODE and jumps to flash byte 0.
# At that point the boot ROM leaves x3 (gp) = 1, so the counter needs no initialisation and byte 0 sets the QSPI clock.
    .section .text.main,"ax"
    .globl main
main:
    sb    zero, 0xFB(zero)  # QSPI_CTRL = 0: fastest QSPI clock (the reset value, sys_clk/128, is too slow to run from)
loop:
    addi  gp, gp, 1         # counter (gp = 1 on entry from the boot ROM)
    srli  t0, gp, 14        # character number (SHIFT: 14 for about 0.6 s per character at 24 MHz; each loop takes about 900 clocks from flash; 12 for quick checks)
    andi  t0, t0, 15        # 16 characters
    lbu   t1, 0xD0(t0)      # pattern = table[t0]  (flash byte 28 appears at address 0xB4 + 28 = 0xD0)
    sb    t1, 0xF0(zero)    # LED_OUT = pattern
    j     loop
table:
    .byte 0x73, 0xB0, 0x38, 0xB0, 0x73, 0xB0, 0x37, 0xF7   # P I L I P I N A
    .byte 0x6D, 0xF7, 0x3D, 0xB0, 0x38, 0xF7, 0x4F, 0xDB   # S A G I L A 3 2
