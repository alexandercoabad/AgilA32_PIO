#include "agila32.h"
/* 8-bit binary counter on the LEDs. */
void main(void){ unsigned c = 0; for(;;){ LED_OUT = c++ >> SLOW; } }
