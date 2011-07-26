/* Name: pyralis84.c
 * Version: 1.2
 * Author: Kelsey Jordahl
 * Copyright: Kelsey Jordahl 2009-2011
 * License: GPLv3
 * Time-stamp: <Wed Jul 27 23:26:02 EDT 2011> 

Simulate fireflies in a jar
Based the Jar-of-Fireflies Instructable
   http://www.instructables.com/id/EITCRQFB2JEWP86TYK/
Modified for use with ATtiny84

Simulate fireflies of species Photinus pyralis, the common Eastern
firefly.  Pulse shape and firefly behavior modeled after Case [2004],
synchronization loosely based on Buck [1988].  Some liberty was taken
making P. pyralis synchronize effectively, it is only occasionally
seen in nature, though other species do.

Buck, J., Synchronous rhythmic flashing of fireflies. II. The Quarterly
     Review of Biology, 63(3):265–289, 1988.
Case, J., Flight studies on photic communication by the firefly
     Photinus pyralis, Integrative and Comparative Biology, 44(3), 250,
     doi:10.1093/icb/44.3.250, 2004.
Lewis, S. M. and C. Cratsley, Flash signal evolution, mate choice, and
     predation in fireflies, Ann. Rev. Ent., 2008.
Lloyd, J.. Studies on the flash communication system in Photinus fireflies,
     Museum of Zoology, University of Michigan, 1966.

    This program is free software: you can redistribute it and/or
    modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.  A copy of the GPL
    version 3 license can be found in the file COPYING or at
    <http://www.gnu.org/licenses/>.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

 */

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>
#include <avr/eeprom.h>

#define WAIT 200		/* 250 ms delay */
#define MAXDELTA 40
#define MINDELTA 10
#define WINDOW 136		/* should be 136 for 3.5 s */
#define PULSELEN 20		/* length of pulse */
#define SLEEPAFTER 6  /* shut down after this many full cycles (20-25 min ea) */
#define SEED_ADDR 0x06		/* EEPROM address to store random seed */

// pins for ATtiny84
//timer0 PWM (not currently used)
#define IN3 (1 << PB2)		/* Pin  5, OC0A */
#define IN4 (1 << PA7)		/* Pin  6, OC0B */
//timer1 PWM
#define IN1 (1 << PA6)		/* Pin  7, OC1A */
#define IN2 (1 << PA5)		/* Pin  8, OC1B */
//
#define OUT1 (1 << PA0)		/* Pin 13 */
#define OUT2 (1 << PA1)		/* Pin 12 */
#define OUT3 (1 << PA2)		/* Pin 11 */
#define OUT4 (1 << PA3)		/* Pin 10 */
#define OUT5 (1 << PA4)		/* Pin 9 */
#define OUT6 (1 << PA7)		/* Pin 6 */
#define IPINS (IN1 | IN2)
#define NOUT 6			/* number of output pins used */
//#define OPINS (OUT1 | OUT2 | OUT3)
#define OPINS (OUT1 | OUT2 | OUT3 | OUT4 | OUT5 | OUT6)
#define ALL_ON IPINS		/* input pins high; output pins low */
#define ALL_OFF OPINS		/* input pins low; output pins high */

// declare functions
void flashall (void);
void init (void);
void start_timers (void);
void getmask (void);

// declare variables
static unsigned char inpin[2] = {IN1, IN2};
static unsigned char outpin[6] = {OUT1, OUT2, OUT3, OUT4, OUT5, OUT6};
static unsigned char mpulse[20] = { /* pulse based on y=t*exp(t^2) */
  0x00, 0x5A, 0xA8, 0xDF, 0xFC, 0xFF, 0xEC, 0xCA, 0xA2, 0x7A,
  0x56, 0x3A, 0x25, 0x16, 0x0C, 0x07, 0x03, 0x02, 0x01, 0x00
};
static unsigned char fpulse[20] = { /* female signal is dimmer by 1/10 */
  0, 9, 17, 23, 26, 26, 24, 21, 17, 13, 9, 6, 4, 3, 2, 1, 1, 1, 0, 0
};
 
volatile unsigned char count;	/* count ticks within a cycle */
volatile unsigned char cycles;	/* count full periods */
volatile unsigned char bigcount=0; /* count reset cycles ~  */
volatile unsigned char fcount;
volatile unsigned char mcount;
volatile unsigned char ncount;
volatile signed char delta;	/* difference of period for new male */
volatile unsigned char nmales;
volatile unsigned char nfemales;
volatile unsigned char nchance;
volatile unsigned char fchance;
volatile unsigned char fpin;	   /* output pin for females */
volatile unsigned char malemask;   /* opins for males */
volatile unsigned char newmask;	   /* opins for a new male */

// define needed bit flags
typedef struct {
  unsigned char bit0:1;
  unsigned char bit1:1;
  unsigned char bit2:1;
  unsigned char bit3:1;
  unsigned char bit4:1;
  unsigned char bit5:1;
  unsigned char bit6:1;
  unsigned char bit7:1;
} io_reg;
#define m1 ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit0
#define m2 ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit1
#define f1 ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit2
#define f2 ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit3
#define n1 ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit4
#define newmale ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit5
#define state ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit6
/* #define  ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit7 */

// external interrupt (button press)
ISR(EXT_INT0_vect) {		/* this is INT0_vect on most AVRs */
  //flashall();			/* visual feedback */
  //  _delay_ms(500);
  // use counter for debouncing
  if (count>0) {
    PORTA = OPINS;		/* all cathodes high, anodes low */
    OCR1A = 0; OCR1B = 0;
    state = ~state;		/* toggle on/off state */
    if (state) {			/* reset */
      init();
      nmales=1;
      set_sleep_mode(SLEEP_MODE_IDLE); /* enable timer interrupts */
    } else {
      count=0;
      set_sleep_mode(SLEEP_MODE_PWR_DOWN); /* don't wake for other interrupts */
    }
  }
  GIFR = (1 << INTF0); /* Clear interrupt flag (just in case) */
}

// interrupt timer function
ISR(TIM0_COMPA_vect) {

    // synchronized male flash
    if (++count<PULSELEN) {
      //      OCR1A = count;
      if (nmales>0) {
	PORTA = ~malemask;
	if (m1) {
	  OCR1A = mpulse[count];
	}
	if (m2) {
	  OCR1B = mpulse[count];
	}	  
      }
    }
    // unsynchronized male flash
    if (newmale) {
      if((count>ncount) & (count<(ncount+PULSELEN))) {
	PORTA = ~newmask;
	if (n1) {
	  OCR1A = mpulse[count-ncount];
	} else {
	  OCR1B = mpulse[count-ncount];
	}	  
      }
    }
    // female response
    if (nfemales>0) {
      if ((count>fcount) & (count<(fcount+PULSELEN))) {
	PORTA = ~outpin[fpin];
	if (f1) {
	  OCR1A = fpulse[count-fcount];
	}
	if (f2) {
	  OCR1B = fpulse[count-fcount];
	}	  
      }
    }
    // see if we're done with this cycle
    if (count==mcount) {
      if (--cycles==0) {
	bigcount++;
	init();			/* reset all */
	nmales=0;
      } else {
	if (newmale) {		/* new asynchronous male */
	  ncount += delta;
	  if (ncount<PULSELEN) {	// pulses overlap
	    if (nmales==1) {	/* OK if only one sync male */
	      delta=-((rand() % 3) + 3); /* set delay small but negative */
	      if (ncount<-delta) {
		nmales++;
		newmale=0;
	      } 
	    } else {		/* otherwise set them to be in sync */
	      nmales++;
	      newmale=0;
	    }
	  } else {
	    if ((ncount>(mcount-PULSELEN)) | (ncount>(mcount-delta))) {
	      // can't finish before end of cycle
	      nmales++;
	      newmale=0;
	    }
	  }
	} else {
	  if ((nmales<(NOUT*2)) & ((rand() % nchance) == 0) ) {
	    /* new male */
	    if (nmales==0) {
	      nmales=1;		/* start flashing at sync rate */
	    } else {
	      ncount=(rand() % mcount);
	      if (ncount<PULSELEN) { /* make sure there's no overlap */
		ncount+=PULSELEN;
	      } else {
		if ((mcount-ncount)<PULSELEN) {
		  ncount-=PULSELEN;
		}
	      }
	      delta=(rand() % MAXDELTA) + MINDELTA;
	      if ((rand() % 2) | (ncount<(3*PULSELEN))) {
		delta = -delta;	/* flip sign */
	      }
	      newmale=1;
	      nfemales=0; /* females stop responding to nonsync males */
	    }
	  } else {
	    if ( (nfemales==0) & (nmales>0) & ((rand() % fchance) == 0) ) {
	      nfemales=1;	/* female response */
	    }
	  }
	}
	/* can't light odd # of LEDs >6 */
	if ((nmales==7) | (nmales==9) | (nmales==11)) {nmales++;}
	getmask();
	count=0;
      }
    }
}


int main(void)
{
  unsigned int seed;		/* random seed */
  DDRA = OPINS | IPINS;		/* set pins 0-4 as outputs */
  PORTA = OPINS;

  //flashall();			/* boot sequence */

  /* read a random seed from EEPROM to keep from being identical each time */
  seed=eeprom_read_word(SEED_ADDR);	/* load last stored seed */
  srand(++seed);  		/* increment and use new value as seed */
  eeprom_write_word(SEED_ADDR,seed);	/* store for next time */

  state=1;
  init();

  nmales=1;
  getmask();

  set_sleep_mode(SLEEP_MODE_IDLE);
  start_timers();

  sei();

  for (;;) {	     /* do nothing; everything happens in interrupts */
    if (bigcount>=SLEEPAFTER) {	/* turn off after certain amount of time */
      state=0;
      PORTA = 0x00;
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    }
    /* if (~state) { // go into low power mode and stay there */
    /*   //      cli();  //bad idea */
    /*   PORTA = 0x00; */
    /*   set_sleep_mode(SLEEP_MODE_PWR_DOWN); */
    /* } */
    sleep_mode();
  }

  return 1;			/* never gets here */
}

void init(void) {
  cycles=255 - (rand() % 55);	/* number of cycles to run */
  //  cycles=80;			/* reset sooner for testing */
  nchance= (rand() % 16) + 4;	/* set odds of new male */
  fchance= (rand() % 12) + 4;	/* set odds of new female */
  count=0;
  nfemales=0; newmale=0;
  fcount=82 + (rand() % 8);	/* 1.9-2.3 s response time */
  mcount=255 - (rand() % 25);	/* 5.3-6.5 s flash interval */
  //mcount=130;			/* shorter interval for testing */
  fpin = (rand() % NOUT);		/* pick an output pin for female */
  f1=(rand() % 2); f2=~f1;	/* pick a side for female */
  getmask();
}

void start_timers(void) {
  // set 16-bit timer1 to phase correct PWM on both IN1 and IN2
  TCCR1A = (1 << WGM11) | (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13);
  TCCR1B |= (1 << CS11);	/* clk/8 */
  TCCR1C = 0x00;
  ICR1 = 0x00FF;	     /* set TOP (only use 8 bits) */
  OCR1A = 0; OCR1B = 0;
  // timer0 will be set to generate an interupt every ~25.6 ms
  TCCR0A = (1 << WGM01);	    /* CTC mode */
  TCCR0B = (1 << CS02 | 1 << CS00);		/* clock/1024 */
  TIMSK0 = (1 << OCIE0A);			/* enable interupts */
  OCR0A = 25;
  // pin change interrupt
  GIMSK = (1<<INT0); 		/* enable external interrupt */
  //MCUCR = (1<<ISC00);
  //  MCUCR = (1<<ISC01)|(1<<ISC00);
  //  PCMSK1 = (1<<PCINT10);	/* PC interrupt on pin 5 */
  //  DDRB = 0x00;
  //  PORTB = (1<<PB2);		/* PB2 (pin 5) as input */

}

// turn on all LEDs one at a time for boot sequence
void flashall(void) {
  unsigned char i;

  PORTA = ALL_ON;
  _delay_ms(WAIT);

  PORTA = (OPINS | IN1);
  for (i=0; i<NOUT; i++) {
    PORTA &= ~outpin[i];	/* one output pin on */
    _delay_ms(WAIT);
    PORTA |= OPINS;		/* all outputs off */
  }
  PORTA = (OPINS | IN2);	/* one input pin on */
  for (i=0; i<NOUT; i++) {
    PORTA &= ~outpin[i];	/* one output pin on */
    _delay_ms(WAIT);
    PORTA |= OPINS;		/* all outputs off */
  }
  _delay_ms(WAIT);
}

// choose which LEDs will be lit for the number of active males
// depends on global variables, not passed through arguments
// sets up the masks for existing males and for a new syncronizing one
void getmask() {
  unsigned int temp;	   /* temporary storage of rand() */
  unsigned char mpin;	   /* output pin for males */

  temp=rand();		/* use one rand() call */

  switch (nmales)
    {
    case 0:			/* all off */
      malemask=0;
      m1=0; m2=0;
    case 1:
      mpin = (temp % NOUT);		/* pick an output pin for male */
      malemask=outpin[mpin];
      m1=(temp % 2);			/* pick a side */
      if ((mpin==fpin) & (m1==f1)) {	/* make sure different than female */
	m1=~f1;
      }
      m2=~m1;
      if (newmale) {
	newmask = malemask;
	n1=m2;
      }
      break;
    case 2:
      mpin = (temp % NOUT);		/* pick an output pin for 1st male */
      malemask = outpin[mpin];
      mpin = ((temp << 4) % NOUT);     /* pick an output pin for 2nd male */
      if (malemask==outpin[mpin]) {    /* avoid conflict */
	if (mpin>1) {
	  malemask = malemask | outpin[mpin-1];
	} else {
	  malemask = malemask | outpin[mpin+1];
	}
      } else {
	malemask = malemask | outpin[mpin];
      }
      m1=(temp % 2);			/* pick a side */
      if ((mpin==fpin) & (m1==f1)) {	/* make sure different than female */
	m1=~f1;
      }
      m2=~m1;
      if (newmale) {
	newmask = outpin[mpin];
	n1=m2;
      }
      break;
    case 3:
      switch (temp % 4)		/* choose one of 4 options */
	{
	case 0:
	  malemask=(OUT1 | OUT2 | OUT3);
	case 1:
	  malemask=(OUT1 | OUT3 | OUT5);
	case 2:
	  malemask=(OUT2 | OUT4 | OUT6);
	case 3:
	  malemask=(OUT4 | OUT5 | OUT6);
	}
      m1=((temp << 2) % 2); m2=~m1;
      if (newmale) {
	newmask= outpin[(temp % NOUT)];
	n1=~m1;
      }
      break;
    case 4:
      switch (temp % 4)		/* choose one of 4 options */
	{
	case 0:
	  malemask=(OUT1 | OUT2 | OUT3 | OUT4);
	case 1:
	  malemask=(OUT1 | OUT3 | OUT5 | OUT6);
	case 2:
	  malemask=(OUT2 | OUT3 | OUT4 | OUT6);
	case 3:
	  malemask=(OUT2 | OUT4 | OUT5 | OUT6);
	}
      m1=(temp % 2); m2=~m1;
      if (newmale) {
	newmask= outpin[(temp % NOUT)];
	n1=~m1;
      }
      break;
    case 5:
      m1=(temp % 2); m2=~m1;
      if ((nfemales>0) & (m1==f1)) { /* avoid using same LED as female */
	mpin=fpin;	       /* don't use same pin (gets negated) */
      } else {
	mpin = (temp % NOUT);	/* randomly pick pin not to be used */
      }
      malemask= OPINS & ~outpin[mpin];
      if (newmale) {		/* pick an LED on other side */
	newmask= outpin[((temp << 4) % NOUT)];
	n1=~m1;
      }
      break;
    case 6:
      switch (temp % 4)		/* choose one of 4 options */
	{
	case 0:
	  malemask=(OUT1 | OUT2 | OUT3);
	case 1:
	  malemask=(OUT1 | OUT3 | OUT5);
	case 2:
	  malemask=(OUT2 | OUT4 | OUT6);
	case 3:
	  malemask=(OUT4 | OUT5 | OUT6);
	}
      m1=1; m2=1;
      if (newmale) {
	newmask= outpin[(temp % NOUT)];
	n1=((temp << 2) % 2);
      }
      break;
    case 7:
      ++nmales;
      break;
    case 8:
      switch (temp % 4)		/* choose one of 4 options */
	{
	case 0:
	  malemask=(OUT1 | OUT2 | OUT3 | OUT4);
	case 1:
	  malemask=(OUT1 | OUT3 | OUT5 | OUT6);
	case 2:
	  malemask=(OUT2 | OUT3 | OUT4 | OUT6);
	case 3:
	  malemask=(OUT2 | OUT4 | OUT5 | OUT6);
	}
      m1=1; m2=1;
      if (newmale) {
	newmask=malemask;	/* actually light 2 new ones */
	n1=(temp % 2);
      }
      break;
    case 9:
      ++nmales;
      break;
    case 10:
      mpin = (temp % NOUT);	/* pick which pin won't be used */
      malemask= OPINS & ~outpin[mpin];
      m1=1; m2=1;
      if (newmale) {
	newmask=malemask;	/* actually light 2 new ones */
	n1=(temp % 2);
      }
      break;
    case 11:
      ++nmales;
      break;
    case 12:			/* light 'em all */
      malemask=OPINS;
      m1=1; m2=1;
      break;
    }
}
