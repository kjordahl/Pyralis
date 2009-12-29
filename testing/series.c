/* Name: pyralis.c
 * Author: Kelsey Jordahl
 * Time-stamp: <Fri Dec 11 14:50:42 EST 2009> 
 * License: CC by-sa-nc 3.0
 *          (Creative Commons 3.0, attribution, share-alike, non-commercial)
 *
 * simulate fireflies in a jar
 * uses the same hardware circuit as the Jar-of-Fireflies Instructable
 *    http://www.instructables.com/id/EITCRQFB2JEWP86TYK/
 * with simpler software logic
 */

#include <avr/io.h>     
#include <util/delay.h>

#define WAIT 500		/* 100 ms delay */
#define IN1 (1 << 1)		/* Pin  1 */
#define IN2 (1 << 4)		/* Pin  4 */
#define OUT1 (1 << 3)		/* Pin  3 */
#define OUT2 (1 << 2)		/* Pin  2 */
#define OUT3 (1 << 0)		/* Pin  0 */
#define IPINS (IN1 | IN2)
#define OPINS (OUT1 | OUT2 | OUT3)

short int inpin[2] = {IN1, IN2};
short int outpin[3] = {OUT1, OUT2, OUT3};
short int i;

int main(void)
{
  DDRB = 0x1F;			/* set pins 0-4 as outputs */
  PORTB = 0x1F & ~IPINS;	/* input pins low; others high */

  for (;;) {
    
    PORTB |= IN1;		/* in1 on */
    PORTB &= ~IN2;		/* in2 off */
    for (i=0; i<3; i++) {
      PORTB &= ~outpin[i];	/* one output pin on */
      _delay_ms(WAIT);
      PORTB |= OPINS;		/* all outputs off */
    }
    PORTB &= ~IN1;		/* in1 off */
    PORTB |= IN2;		/* in2 on */
    for (i=0; i<3; i++) {
      PORTB &= ~outpin[i];	/* one output pin on */
      _delay_ms(WAIT);
      PORTB |= OPINS;		/* all outputs off */
    }
	    
  }

  return 1;
}
