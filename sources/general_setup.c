/*
 * general_setup.c
 *
 * Created: 2015-11-04 17:45:11
 *  Author: remy mourard
 */
#include "general_setup.h"

#include <avr/io.h>
#include <avr/interrupt.h>

#ifndef SREG
	#define SREG _SFR_IO8(0x3F)
#endif

#define SREG_GLOBAL_INT 7

void global_int_enable()
{
	sei();
}

void accel_int_enable()
{
	// set PORTD-PIN2 to input, could handle interrupts
	DDRD &= ~(1 << DDD2);
    // and active pull ups
    PORTD |= (1 << PIND2);
	
	// deactivate interrupts during EICRA
	// manipulation
	EIMSK &= ~(1 << INT2);
	
	// set interrupts on low level 00 (according to accelgyro.c configure_accelgyro()
	EICRA &= ~(1 << ISC21);
	EICRA &= ~(1 << ISC20);
	
	// reactivate interrupts
	EIMSK |= (1 << INT2);
}