# PILIPINAS LASALLE on the 7-segment display (uo_out[6:0] = segments a-g, uo7 = dp)
# 7 instructions + a 16-byte pattern table = 44 bytes, the whole bootload window.
# gp is a free-running counter; its bits 15:12 pick the character, so each
# character stays up for 2^12 loop passes of 6 instructions (about 172k clocks).
# dp (bit 7) is set on every second character so a repeated letter (LL) still
# shows as a change.
    .section .text.main,"ax"
    .globl main
main:
    mv    gp, zero          # start at the first character
loop:
    addi  gp, gp, 1         # counter
    srli  t0, gp, 12        # character number (SHIFT: 12 here, 18 for about 0.45 s on hardware)
    andi  t0, t0, 15        # 16 characters
    lbu   t1, 0xD0(t0)      # pattern = table[t0]  (table is at 0xB4 + 28 = 0xD0)
    sb    t1, 0xF0(zero)    # LED_OUT = pattern
    j     loop
table:
    .byte 0x73, 0xB0, 0x38, 0xB0, 0x73, 0xB0, 0x37, 0xF7   # P I L I P I N A
    .byte 0x6D, 0xB8, 0x77, 0xED, 0x77, 0xB8, 0x38, 0xF9   # S L A S A L L E
