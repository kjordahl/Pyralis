/* Name: pyralis.c
 * Author: Kelsey Jordahl
 * Time-stamp: <Sat Dec 12 17:04:19 EST 2009> 
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
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>

#define WAIT 250		/* 100 ms delay */
#define IN1 (1 << 1)		/* Pin  1 */
#define IN2 (1 << 4)		/* Pin  4 */
#define OUT1 (1 << 3)		/* Pin  3 */
#define OUT2 (1 << 2)		/* Pin  2 */
#define OUT3 (1 << 0)		/* Pin  0 */
#define IPINS (IN1 | IN2)
#define OPINS (OUT1 | OUT2 | OUT3)
#define ALL_ON 0x1F & ~OPINS
#define ALL_OFF 0x1F & ~IPINS	/* input pins low; others high */

short int inpin[2] = {IN1, IN2};
short int outpin[3] = {OUT1, OUT2, OUT3};
char song[20] = {
  0, 90,  168,  223,  252,  255,  236,  202,
  162,  122,   86,   58,   37,   22,   12,    7, 3,    2,    1, 0
};

int main(void)
{  short int i;
  short int j;
  short int k;
  flashall();
  ioinit();
  //  TCCR0B = (1<<CS01);
  // GTCCR |= (1 << PWM1B) | (1 << COM1B1);

  for (;;) {

    // timer0 on
    TCCR0B = (1<<CS01);

    //    _delay_ms(6000);		/* wait 6 s */
    PORTB = ALL_OFF;
    _delay_ms(1000);
    PORTB = ALL_ON;

    // this sets IN1 to PWM
    TCCR0A = (1<<COM0B1)|(0<<COM0B0)|(1<<WGM01)|(1<<WGM00);
    for( k = 0; k < 20; k++ ) { /* play a song */
      OCR0B = song[k];
      _delay_ms(25);
    } 

    PORTB = ALL_OFF;
    _delay_ms(1000);
    PORTB = ALL_ON;

    // this sets OUT3 to PWM
    TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
    for( k = 0; k < 20; k++ ) { /* play a song */
      OCR0A = song[k];
      _delay_ms(25);
    } 

    // timer0 off
    TCCR0A = 0x00;
    TCCR0B = 0x00;
    // timer1 on
    GTCCR = (1 << PWM1B) | (1 << COM1B1);

    PORTB = ALL_OFF;
    _delay_ms(1000);
    PORTB = ALL_ON;

    // sets IN2 to PWM
    TCCR1 = (0<<COM1A1)|(1<<COM1A0)|(1 << PWM1A) | (1<<WGM01)|(1<<WGM00);
    for( k = 0; k < 20; k++ ) { /* play a song */
      OCR1A = song[k];
      OCR1B = song[k];
      _delay_ms(25);
    } 

    PORTB = ALL_OFF;
    _delay_ms(1000);
    PORTB = ALL_ON;

    // 
    TCCR1 = (1<<COM1A1)|(0<<COM1A0)|(1 << PWM1A)|(1<<WGM01)|(1<<WGM00);
    for( k = 0; k < 20; k++ ) { /* play a song */
      OCR1A = song[k];
      OCR1B = song[k];
      _delay_ms(25);
    } 

    PORTB = ALL_OFF;
    _delay_ms(1000);
    PORTB = ALL_ON;

    // 
    TCCR1 = (1<<COM1A1)|(1<<COM1A0)|(1 << PWM1A) | (1<<WGM01)|(1<<WGM00);
    for( k = 0; k < 20; k++ ) { /* play a song */
      OCR1B = song[k];
      OCR1A = song[k];
      _delay_ms(25);
    } 

    // timer1 off
    TCCR1 = 0x00;
    GTCCR = 0x00;

  }

  return 1;
}

void ioinit(void) {
  DDRB = 0x1F;			/* set pins 0-4 as outputs */
  PORTB = 0x1F & ~IPINS;	/* input pins low; others high */
   // Timer0, Set OC0A on comp. match (inv). Mode 3: Fast PWM
  //   TCCR0A = (1<<COM0A1)|(1<<COM0A0)|(1<<WGM01)|(1<<WGM00);
   // 1:8 presc.
  //   TCCR0B = (1<<CS01);
/*   TCCR1A = TIMER1_PWM_INIT; */
/*   TCCR1B |= TIMER1_CLOCKSOURCE; */
/*   OCR = 0; */
/*   DDROC = _BV (OC1); */
/*   TIMSK = _BV (TOIE1); */
/*   sei (); */

}

// this function works, but made the code huge with a delay arg!
void flashall(void) {
  short int i;
  short int j;

  DDRB = 0x1F;			/* set pins 0-4 as outputs */
  for (j=0; j<2; j++) {
    PORTB = 0x1F & ~IPINS;	/* all off */
    PORTB |= inpin[j];		/* one input pin on */
    for (i=0; i<3; i++) {
      PORTB &= ~outpin[i];	/* one output pin on */
      _delay_ms(WAIT);
      PORTB |= OPINS;		/* all outputs off */
    }
  }
}


/* void pwm(short int level){ */
/*         OCR0A = level;   //Load Pulse width */
/* 	//        OCR0AH = 0; */
/* /\*         DDRD |= (1<<5);         //PortD.5 as o/p *\/ */
/*         TCCR0A = 0x81;          //8-bit, Non-Inverted PWM */
/*         TCCR0B = 1;             //Start PWM */
/* } */
