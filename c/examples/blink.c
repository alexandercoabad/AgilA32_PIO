#include "agila32.h"
/* Blink LED0 about every 0.3 s at 24 MHz. */
void main(void){ unsigned c = 0; for(;;){ LED_OUT = (c++ >> SLOW) & 1; } }
