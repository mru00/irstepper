/************************************************************************
 * This file is part of TerraControl.								    *
 * 																	    *
 * TerraControl is free software; you can redistribute it and/or modify *
 * it under the terms of the GNU General Public License as published    *
 * by the Free Software Foundation; either version 2 of the License, or *
 * (at your option) any later version.								    *
 * 																	    *
 * TerraControl is distributed in the hope that it will be useful,	    *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of	    *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the	    *
 * GNU General Public License for more details.						    *
 * 																	    *
 * You should have received a copy of the GNU General Public License    *
 * along with TerraControl; if not, write to the Free Software		    *
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 *
 * USA																    *
 * Written and (c) by mru											    *
 * Contact <mru@sisyphus.teil.cc> for comment & bug reports				*
 ************************************************************************/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>

#include <stdlib.h>

#include "common.h"

#define UART_BAUD_RATE     38400

static void __attribute__((constructor)) 
uart_constructor(void) {
  uart_init(UART_BAUD_SELECT(UART_BAUD_RATE,XTAL));
}

#define PHASE_A 0
#define PHASE_B 1

#define PORT_DC_PHASE &PORTC, PC5
#define PORT_DC_INHIB &PORTB, PB1

#define PORT_BIP_PHASE_A &PORTB, PB0
#define PORT_BIP_PHASE_B &PORTD, PD6
#define PORT_BIP_PHASE_I &PORTB, PB2

#define PORT_UNI_PHASE_A1 &PORTC, PC0
#define PORT_UNI_PHASE_A2 &PORTC, PC1
#define PORT_UNI_PHASE_B1 &PORTC, PC2
#define PORT_UNI_PHASE_B2 &PORTC, PC3

#define RC5_VOL_UP 16
#define RC5_VOL_DN 17
#define RC5_PROG_UP 32
#define RC5_PROG_DN 33
#define RC5_FR 13
#define RC5_FF 52
#define RC5_PLAY 53
#define RC5_STOP 54


static void bip_sleep_phase(void) {
  _delay_ms(5);
}

static void uni_sleep_phase(void) {
  _delay_ms(10);
}


static void set_phase(uint8_t phase, uint8_t value) {
  if ( phase == PHASE_A ) xpin2 ( value,  PORT_BIP_PHASE_A );
  else 	xpin2 ( value,  PORT_BIP_PHASE_B );

  bip_sleep_phase();
}



static void uni_set_phase(uint8_t phase, uint8_t value) {

  if ( phase == PHASE_A ) {

	if ( value ) {
	  clearpin2(PORT_UNI_PHASE_A2);
	  setpin2(PORT_UNI_PHASE_A1);
	}
	else {
	  clearpin2(PORT_UNI_PHASE_A1);
	  setpin2(PORT_UNI_PHASE_A2);
	}
  }
  else {

	if ( value ) {
	  clearpin2(PORT_UNI_PHASE_B2);
	  setpin2(PORT_UNI_PHASE_B1);
	}
	else {
	  clearpin2(PORT_UNI_PHASE_B1);
	  setpin2(PORT_UNI_PHASE_B2);
	}
  }

  uni_sleep_phase();
}


static void bip_step_forward(void) {
  clearpin2(PORT_BIP_PHASE_I);
  set_phase(PHASE_A, 0);
  set_phase(PHASE_B, 0);
  set_phase(PHASE_A, 1);
  set_phase(PHASE_B, 1);
}

static void bip_step_backward(void) {
  clearpin2(PORT_BIP_PHASE_I);
  set_phase(PHASE_B, 0);
  set_phase(PHASE_A, 0);
  set_phase(PHASE_B, 1);
  set_phase(PHASE_A, 1);
}


static void uni_step_forward(void) {
  uni_set_phase(PHASE_A, 0);
  uni_set_phase(PHASE_B, 0);
  uni_set_phase(PHASE_A, 1);
  uni_set_phase(PHASE_B, 1);
}

static void uni_step_backward(void) {
  uni_set_phase(PHASE_B, 0);
  uni_set_phase(PHASE_A, 0);
  uni_set_phase(PHASE_B, 1);
  uni_set_phase(PHASE_A, 1);
}

static void dc_forward(void) {
  setpin2(PORT_DC_PHASE);
}

static void dc_backward(void) {
  clearpin2(PORT_DC_PHASE);
}

static void step_forward(void) {
  uni_step_forward();
  bip_step_forward();
}

static void step_backward(void) {
  uni_step_backward();
  bip_step_backward();
}

static void power_off(void) {
  clearpin2(PORT_UNI_PHASE_A1);
  clearpin2(PORT_UNI_PHASE_A2);
  clearpin2(PORT_UNI_PHASE_B1);
  clearpin2(PORT_UNI_PHASE_B2);
  setpin2(PORT_BIP_PHASE_I);
}

#define uchar unsigned char
#define uint unsigned int

#define	xRC5_IN		PIND
#define	xRC5		PD7			// IR input low active


#define RC5TIME 	1.778e-3		// 1.778msec
#define PULSE_MIN	(uchar)(XTAL / 512 * RC5TIME * 0.4 + 0.5)
#define PULSE_1_2	(uchar)(XTAL / 512 * RC5TIME * 0.8 + 0.5)
#define PULSE_MAX	(uchar)(XTAL / 512 * RC5TIME * 1.2 + 0.5)


uchar	rc5_bit;				// bit value
uchar	rc5_time;				// count bit time
uint	rc5_tmp;				// shift bits in
volatile uint	rc5_data;				// store result


SIGNAL (SIG_OVERFLOW0)
{
  uint tmp = rc5_tmp;				// for faster access

  TCNT0 = -2;					// 2 * 256 = 512 cycle

  if( ++rc5_time > PULSE_MAX ){			// count pulse time
    if( !(tmp & 0x4000) && tmp & 0x2000 )	// only if 14 bits received
      rc5_data = tmp;
    tmp = 0;
  }

  if( (rc5_bit ^ xRC5_IN) & 1<<xRC5 ){		// change detect
    rc5_bit = ~rc5_bit;				// 0x00 -> 0xFF -> 0x00

    if( rc5_time < PULSE_MIN )			// to short
      tmp = 0;

    if( !tmp || rc5_time > PULSE_1_2 ){		// start or long pulse time
      if( !(tmp & 0x4000) )			// not to many bits
        tmp <<= 1;				// shift
      if( !(rc5_bit & 1<<xRC5) )		// inverted bit
        tmp |= 1;				// insert new bit
      rc5_time = 0;				// count next pulse time
    }
  }

  rc5_tmp = tmp;
}


static void dc_stop(void) {
  TCCR1A &= ~_BV(COM1A1);
  setpin2(PORT_DC_INHIB);
}

static void dc_pwm(void) {
  TCCR1A |= _BV(COM1A1);
}

static void dc_full(void) {
  TCCR1A &= ~_BV(COM1A1);
  clearpin2(PORT_DC_INHIB);
}


static void dc_faster(uint16_t step) {
  if ( TCCR1A & _BV(COM1A1) && OCR1A > (0 + step) ) {
	OCR1A -= step;
  }
}

static void dc_slower(uint16_t step) {
  if ( TCCR1A & _BV(COM1A1) && OCR1A < (ICR1 + step) ) {
	OCR1A += step;
  }
}

int main(void)
{

  uint i;
  _delay_ms(100);

  // init section -------------------------------------

  TCCR0 = 1<<CS02;			//divide by 256
  TIMSK = 1<<TOIE0;			//enable timer interrupt

  DDRC = 0xff;
  PORTC = 0x00;

  DDRD &= ~_BV(PD7);
  DDRD |= _BV(PD6);
  PORTD |= _BV(PD7);

  DDRB = 0x07;

  //http://www.mikrocontroller.net/articles/AVR-GCC-Tutorial#PWM_.28Pulsweitenmodulation.29

  TCCR1A = _BV(COM1A1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);

  ICR1 = 0xf00;

  OCR1A = 0x0;

  step_backward();  
  step_backward();
  step_forward();

  sei();
  /* main loop section ---------------------------------- */

  uint8_t move = 0;
  uint16_t step = ICR1 / 16;

  for(;;) {

    cli();
    i = rc5_data;			// read two bytes from interrupt !
    rc5_data = 0;
    sei();
    if( i ){
	  //	  uint device = i >> 6 & 0x1F;
	  uint code = (i & 0x3F) | (~i >> 7 & 0x40);

	  switch (code) {

	  case RC5_VOL_UP:

		dc_forward();
		dc_full();
		_delay_ms(30);
		dc_stop();

		move = 1;
		break;

	  case RC5_VOL_DN:


		dc_backward();
		dc_full();
		_delay_ms(30);
		dc_stop();

		move = 2;

		break;
				
	  case RC5_PROG_DN:
		dc_slower(step);
		break;

	  case RC5_PROG_UP:
		dc_faster(step);
		break;

	  case RC5_FF:
		dc_full();
		break;

	  case RC5_FR:
		dc_full();
		break;

	  case RC5_STOP:
		dc_stop();
		break;

	  case RC5_PLAY:
		dc_pwm();
		break;
		
	  default:
		move = 0;
	  }

	}


	if ( move == 1 ) step_backward();
	else if ( move == 2 ) step_forward();
	else {
	  _delay_ms(10);
	  power_off();
	}

	move = 0;
  }
}


/*  LocalWords:  eeprom
 */
