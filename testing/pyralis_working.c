/* Name: pyralis.c
 * Author: Kelsey Jordahl
 * Time-stamp: <Fri Dec 18 08:59:35 EST 2009> 
 * License: CC by-sa-nc 3.0
 *          (Creative Commons 3.0, attribution, share-alike, non-commercial)
 *
 * simulate fireflies in a jar
 * uses the same hardware circuit as the Jar-of-Fireflies Instructable
 *    http://www.instructables.com/id/EITCRQFB2JEWP86TYK/
 * with simpler software logic
 * 
 * Simulate fireflies of species Photinus pyralis, the common Eastern
 * firefly.  Pulse shape and firefly behavior modeled after Case [2004],
 * synchronization loosely based on Buck [1988].  Some liberty was taken
 * making P. pyralis synchronize effectively, it is only occasionally
 * seen in nature, though other species do.
 *
 * Buck, J., Synchronous rhythmic flashing of fireflies. II. The Quarterly
        Review of Biology, 63(3):265–289, 1988.
 * Case, J., Flight studies on photic communication by the firefly
 *      Photinus pyralis, Integrative and Comparative Biology, 44(3), 250,
 *      doi:10.1093/icb/44.3.250, 2004.
 * Lewis, S. M. and C. Cratsley, Flash signal evolution, mate choice, and
 *      predation in fireflies, Ann. Rev. Ent., 2008.
 * Lloyd, J.. Studies on the flash communication system in Photinus fireflies,
 *      Museum of Zoology, University of Michigan, 1966.
 */

#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <stdlib.h>

#define WAIT 250		/* 250 ms delay */
//#define NCHANCE 8		/* 1 in N chance of new male each cycle */
//#define FCHANCE 4		/* 1 in N chance of new female each cycle */
//#define MAXCOUNT 150		/* number of counts per cycle */
#define MAXDELTA 40
#define MINDELTA 10
#define WINDOW 50		/* should be 136 for 3.5 s */
#define SEED 0x27E8		/* random number seed */
#define PULSELEN 20		/* length of pulse */
#define IN1 (1 << PB1)		/* Pin  1 */
#define IN2 (1 << PB4)		/* Pin  4 */
#define OUT1 (1 << PB3)		/* Pin  3 */
#define OUT2 (1 << PB2)		/* Pin  2 */
#define OUT3 (1 << PB0)		/* Pin  0 */
#define IPINS (IN1 | IN2)
#define OPINS (OUT1 | OUT2 | OUT3)
#define ALL_ON 0x1F & ~OPINS	/* input pins high; output pins low */
#define ALL_OFF 0x1F & ~IPINS	/* input pins low; output pins high */

// declare functions
void flashall (void);
void init (void);
void start_timers (void);
void getmask (void);

// declare variables
volatile unsigned char inpin[2] = {IN1, IN2};
volatile unsigned char outpin[3] = {OUT1, OUT2, OUT3};
volatile unsigned char mpulse[20] = { /* pulse based on y=t*exp(t^2) */
  0, 90, 168, 223, 252, 255, 236, 202, 162, 122,
  86, 58, 37, 22, 12, 7, 3, 2, 1, 0
};
//volatile unsigned char fpulse[20] = { /* female signal is dimmer by 1/2 */
//  0, 45, 84, 112, 126, 128, 118, 101, 81, 61,
//  43, 29, 19, 11, 6, 4, 2, 1, 1, 0
//};
//volatile unsigned char fpulse[20] = { /* female signal is dimmer by 1/4 */
//  0, 23, 42, 56, 63, 64, 59, 51, 41, 31, 22, 15, 10, 6, 3, 2, 1, 1, 1, 0
//};
volatile unsigned char fpulse[20] = { /* female signal is dimmer by 1/10 */
  0, 9, 17, 23, 26, 26, 24, 21, 17, 13, 9, 6, 4, 3, 2, 1, 1, 1, 1, 0
};
 
volatile unsigned char count;	/* count ticks within a cycle */
volatile unsigned char cycles;	/* count full periods */
volatile unsigned char fcount;
volatile unsigned char mcount;
volatile unsigned char ncount;
volatile signed char delta;	/* difference of period for new male */
volatile unsigned char nmales;
volatile unsigned char nfemales;
volatile unsigned char nchance;
volatile unsigned char fchance;
volatile unsigned char fpin;	   /* output pin for females */
volatile unsigned char mpin;	   /* output pin for males */
volatile unsigned char malemask;   /* opins for males */
volatile unsigned char newmask;	   /* opins for a new male */
volatile unsigned char seed;	   /* random seed */
volatile unsigned int temp;	   /* temporary storage of rand() */

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
/* #define  ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit6 */
/* #define  ((volatile io_reg*) _SFR_MEM_ADDR(GPIOR0))->bit7 */

// interrupt timer function
ISR(TIM0_COMPA_vect) {

  //    PORTB = ALL_ON;
    // synchronized male flash
    if (++count<PULSELEN) {
      //      OCR1A = count;
      if (nmales>0) {
	PORTB = ~malemask;
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
	PORTB = ~newmask;
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
	PORTB = ~outpin[fpin];
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
	init();			/* reset all */
	nmales=0;
      } else {
	if (newmale) {		/*  */
	  //	nmales++;
	  //	newmale=0;
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
	    if ((mcount-ncount)<WINDOW) {
	      ncount=PULSELEN;	/* reset flash */
	      delta=-((rand() % 3) + 3); /* set delay small but negative */
	    } else {
	      if ((ncount>(mcount-PULSELEN)) | (ncount>(mcount-delta))) {
		// can't finish before end of cycle
		nmales++;
		newmale=0;
	      } else {		/* old male sync to new male */
		if ( (nmales==1) & (((ncount-ncount)<WINDOW)) ) {
		  ncount=PULSELEN;	/* reset flash */
		  delta=-((rand() % 3) + 3); /* set delay small but negative */
		}
	      }
	    }
	  }
	} else {
	  if ((nmales<6) & ((rand() % nchance) == 0) ) {
	    /* new male */
	    if (nmales==0) {
	      nmales=1;		/* start flashing at sync rate */
	    } else {
	      //	    ncount=(PULSELEN + rand() % (mcount-(2*PULSELEN)-1)); /* start at a random time */
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
	if (nmales==5) {nmales++;} /* can't light 5 */
	getmask();
	count=0;
      }
    }
}


int main(void)
{
  DDRB = 0x1F;			/* set pins 0-4 as outputs */
  PORTB = ALL_OFF;

  flashall();			/* boot sequence */

  // Read the seed value from EEPROM 0x00, use it, increment and store it.
  // https://randomskk.net/projects/fireflies_in_jar/
	
  EEARL	= 0x00;
  EECR	|= (1<<EERE);
  seed	= EEDR;
  seed++;
  srand( (int)seed );
  EECR	= (0<<EEPM1)|(0>>EEPM0);
  EEARL	= 0x00;
  EEDR	= seed;
  EECR	|= (1<<EEMPE);
  EECR	|= (1<<EEPE);
  
  init();
  nmales=1;

  set_sleep_mode(SLEEP_MODE_IDLE);
  start_timers();
  sei();

  for (;;) {	     /* do nothing; everthing happens in interrupts */
    sleep_mode();
  }

  return 1;			/* never gets here */
}

void init(void) {
  cycles=255 - (rand() % 155);	/* number of cycles to run */
  //    cycles=80;			/* reset sooner for testing */
  nchance= (rand() % 8) + 4;	/* set odds of new male */
  fchance= (rand() % 8) + 4;	/* set odds of new female */
  count=0;
  nfemales=0; newmale=0;
  fcount=82 + (rand() % 8);	/* 1.9-2.3 s response time */
  mcount=255 - (rand() % 25);	/* 5.3-6.5 s flash interval */
  //  mcount=130;			/* shorter interval for testing */
  fpin = (rand() % 3);		/* pick an output pin for female */
  f1=(rand() % 2); f2=~f1;	/* pick a side for female */
  getmask();
}

void start_timers(void) {
  // timer1 will generate PWM on both IN1 and IN2
  TCCR1 = (1 << COM1A1) | (1 << PWM1A) | (1 << WGM01) | (1 << WGM00);
  GTCCR = (1 << PWM1B) | (1 << COM1B1);
  OCR1A = 0; OCR1B = 0;
  // timer0 will be set to generate an interupt every ~25.6 ms
  TCCR0A = (1 << WGM01);	    /* CTC mode */
  TCCR0B = (1 << CS02 | 1 << CS00);		/* clock/1024 */
  TIMSK = (1 << OCIE0A);			/* enable interupts */
  OCR0A = 25;
}

// turn on all LEDs one at a time for boot sequence
void flashall(void) {
  unsigned char i;
  unsigned char j;

  for (j=0; j<2; j++) {
    PORTB = ((ALL_OFF) | inpin[j]);	/* one input pin on */
    for (i=0; i<3; i++) {
      PORTB &= ~outpin[i];	/* one output pin on */
      _delay_ms(WAIT);
      PORTB |= OPINS;		/* all outputs off */
    }
  }
}

// choose which LEDs will be lit for the number of active males
// depends on global variables, not passed through arguments
// sets up the masks for existing males and for a new syncronizing one
void getmask() {

  temp=rand();		/* use one rand() call */

  switch (nmales)
    {
    case 0:			/* all off */
      malemask=0;
      m1=0; m2=0;
    case 1:
      mpin = (temp % 3);		/* pick an output pin for male */
      malemask=outpin[mpin];
      m1=(temp % 2);			/* pick a side */
      if ((mpin==fpin) & (m1==n1)) {	/* make sure different than female */
	m1=~f1;
      }
      m2=~m1;
      if (newmale) {
	newmask = malemask;
	n1=m2;
      }
      break;
    case 2:
      m1=(temp % 2); m2=~m1;
      if ((nfemales>0) & (m1==f1)) { /* avoid using same LED as female */
	mpin=fpin;	       /* don't use same pin (gets negated) */
      } else {
	mpin = (temp % 3);	/* randomly pick pin not to be used */
      }
      malemask= OPINS & ~outpin[mpin];
      if (newmale) {		/* pick an LED on other side */
	newmask= outpin[((temp << 4) % 3)];
	n1=~m1;
      }
      break;
    case 3:			/* light a whole side */
      malemask=OPINS;
      m1=(temp % 2); m2=~m1;
      if (newmale) {
	newmask= outpin[(temp % 3)];
	n1=~m1;
      }
      // should only do this if there are already 2 females
      //      ++nmales;
      break;
    case 4:
      mpin = (temp % 3);	/* pick which pin won't be used */
      malemask= OPINS & ~outpin[mpin];
      m1=1; m2=1;
      if (newmale) {
	newmask=malemask;	/* actually light 2 new ones */
	n1=~m1;
      }
      break;
    case 5:			/* can't light 5; let's go to 6 */
      ++nmales;
      break;
    case 6:			/* light 'em all */
      malemask=OPINS;
      m1=1; m2=1;
      break;
    }
}
