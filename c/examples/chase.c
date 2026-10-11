#include "agila32.h"
/* One lit LED walks across LED0..LED7 and wraps. */
void main(void){ unsigned c = 0; for(;;){ LED_OUT = 1 << ((c++ >> SLOW) & 7); } }
