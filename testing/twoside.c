#include <avr/io.h>     
#include <util/delay.h>

// My first blinky program
// Kelsey Jordahl
// Time-stamp: <Thu Dec 10 21:49:21 EST 2009>

#define IN1 1
#define IN2 4
#define OUT1 3
#define OUT2 2
#define OUT3 0

int main(void)
{
  DDRB = 0xFF;			/* all pins set  as outputs */
  PORTB = 0x00;			/* all pins low */

  for (;;) {
    
    /* one side on */
    PORTB |= (1 << IN1);
    PORTB &= ~(1 << IN2);
    _delay_ms(500);

    /* other side on */
    PORTB &= ~(1 << IN1);
    PORTB |= (1 << IN2);
    _delay_ms(500);
	    
  }

  return 1;
}
