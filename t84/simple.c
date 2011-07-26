#include <avr/io.h>     
#include <util/delay.h>

// Simple blinky program for ATtiny84
// Kelsey Jordahl
// Time-stamp: <Sun Jul 17 17:38:24 EDT 2011>

int main(void)
{
  DDRA = 0xFF;			/* all pins set  as outputs */
  PORTA = 0x00;			/* all pins low */

  for (;;) {
    
    /* on */
    PORTA = 0xFF;
    _delay_ms(200);

    /* off */
    PORTA = 0x00;
    _delay_ms(1000);
	    
  }

  return 1;
}
