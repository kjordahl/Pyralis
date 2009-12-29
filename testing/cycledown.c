#include <avr/io.h>     
#include <util/delay.h>

// My first blinky program
// Kelsey Jordahl
// Time-stamp: <Thu Dec 10 22:31:29 EST 2009>

#define IN1 1
#define IN2 4
#define OUT1 3
#define OUT2 2
#define OUT3 0

char outpin[3] = {OUT1, OUT2, OUT3};
short int i;

int main(void)
{
  DDRB = 0xFF;			/* all pins set  as outputs */
  PORTB = 0xFF & ~(1<<IN1 | 1<<IN2); /* input pins low; others high */

  for (;;) {
    
    PORTB = 0x00;
    PORTB |= (1 << IN1);
    _delay_ms(50);
    PORTB |= (1 << OUT1);
    _delay_ms(50);
    PORTB |= (1 << OUT2);
    _delay_ms(50);
    PORTB |= (1 << OUT3);
    _delay_ms(50);
    PORTB = 0x00;
    PORTB |= (1 << IN2);
    _delay_ms(50);
    PORTB |= (1 << OUT1);
    _delay_ms(50);
    PORTB |= (1 << OUT2);
    _delay_ms(50);
    PORTB |= (1 << OUT3);
    _delay_ms(50);
	    
  }

  return 1;
}
