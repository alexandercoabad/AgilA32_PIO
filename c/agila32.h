#ifndef AGILA32_H
#define AGILA32_H
/* AgilA32 memory-mapped registers (8-bit address space) */
#define REG8(a)    (*(volatile unsigned char *)(a))
#define LED_OUT    REG8(0xF0)   /* write-only, drives uo_out[7:0]           */
#define SW_IN      REG8(0xF4)   /* read-only, reflects ui_in                */
#define FLASH_MODE REG8(0xF8)   /* write any value: 0xB4-0xDF reads flash   */
#define PIO_DATA   REG8(0xFE)
#define PIO_IDX    REG8(0xFF)
/* The code window is only 44 bytes, so examples keep a free-running counter
   in a register and show a slice of it instead of calling a delay routine.
   Bit N of the counter toggles every 2^N loop passes. Measured in the
   simulator, a pass takes 28 clocks (counter), 35 (blink) or 42 (chase), so
   with SLOW=18 one step is about 0.31 s, 0.38 s or 0.46 s at 24 MHz. */
#ifndef SLOW
#define SLOW 18
#endif
#endif
