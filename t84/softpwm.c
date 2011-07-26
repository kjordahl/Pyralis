/* Name: softpwm.c
 * Version: 1.0
 * Author: Kelsey Jordahl
 * Copyright: Kelsey Jordahl 2011
 * License: GPLv3
 * Time-stamp: <Sat Jul 23 15:55:37 EDT 2011> 

Simple demonstration of 8-channel software PWM on ATtiny84.  Output to
channels PA0-7, pins 6-13.

Based on Atmel Application Note AVR136, Low-Jitter Multi-Channel Software
PWM, by Andy Gayne, copyright Atmel 2006.

AVR136: http://www.atmel.com/dyn/resources/prod_documents/doc8020.pdf
code:   http://www.atmel.com/dyn/resources/prod_documents/AVR136.zip


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

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/sleep.h>


#define DEBUG 0

//! Pin mappings
#define CHMAX       8    // maximum number of PWM channels

#define CH0_CLEAR (pinlevelA &= ~(1 << PA0))
#define CH1_CLEAR (pinlevelA &= ~(1 << PA1))
#define CH2_CLEAR (pinlevelA &= ~(1 << PA2))
#define CH3_CLEAR (pinlevelA &= ~(1 << PA3))
#define CH4_CLEAR (pinlevelA &= ~(1 << PA4))
#define CH5_CLEAR (pinlevelA &= ~(1 << PA5))
#define CH6_CLEAR (pinlevelA &= ~(1 << PA6))
#define CH7_CLEAR (pinlevelA &= ~(1 << PA7))

#define PORTA_MASK 0xFF

volatile unsigned char level[CHMAX];

void Init (void);

void main(void)
{
  unsigned char i;
  Init();
  //  cli();			/* disable interrupts for testing */
  for(;;)
  {
    _delay_ms(20);
    for(i=0 ; i<CHMAX ; i++)      // initialise all channels
      {
	level[i]++;
      }
  }
}

/*! \brief Init function. This function initialises the hardware
 */
void Init(void)
{

  CLKPR = (1 << CLKPCE);        // enable clock prescaler update
  CLKPR = 0;                    // set clock to maximum (= crystal)


  DDRA = PORTA_MASK;            // set port pins to output

  level[0] = 0x05;
  level[1] = 0x10;
  level[2] = 0x15;
  level[3] = 0x20;
  level[4] = 0x25;
  level[5] = 0x30;
  level[6] = 0x35;
  level[7] = 0x40;


  //  ADMUX |= (1 << REFS1) | (1 << MUX3) |  (1 << MUX2) |(1 << MUX1) |(1 << MUX0); //Select Temp  sense and 1.1V ref and left-adjusting result.

  //ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1);  //Turn on ADC and set clock to /64.

  TCCR1B = (1 << WGM12);	    /* CTC mode */
  TCCR1B |= (1 << CS00); // start timer, no prescale
  TIMSK1 = (1 << OCIE1A);			/* enable interupts */
  OCR1A = 100;

  sei();         // enable interrupts
}

ISR(TIM1_COMPA_vect) {
  static unsigned char pinlevelA=PORTA_MASK;
  static unsigned char softcount=0xFF;

  PORTA = pinlevelA;            // update outputs


  if(++softcount == 0){
    pinlevelA = PORTA_MASK;     // set all port pins high
  }
  // clear port pin on compare match
  if(level[0] == softcount) CH0_CLEAR;
  if(level[1] == softcount) CH1_CLEAR;
  if(level[2] == softcount) CH2_CLEAR;
  if(level[3] == softcount) CH3_CLEAR;
  if(level[4] == softcount) CH4_CLEAR;
  if(level[5] == softcount) CH5_CLEAR;
  if(level[6] == softcount) CH6_CLEAR;
  if(level[7] == softcount) CH7_CLEAR;


}
