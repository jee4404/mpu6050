/*
 * general_setup.c
 *
 * Created: 2015-11-04 17:45:11
 *  Author: remy
 */
#include "general_setup.h"

#ifndef SREG
	#define SREG _SFR_IO8(0x3F)
#endif

#define SREG_GLOBAL_INT 7

uint8_t global_int_enable()
{
	SREG |= (1 << SREG_GLOBAL_INT);
	
	return 0;
}

uint8_t accel_int_enable()
{
	// set portb1 to input, could handle interrupts
	DDRB &= (0 << DDB1);
	
	// deactivate interrupts during EICRA
	// manipulation
	EIMSK &= ~(0 << INT1);
	
	// set interrupts on low level 00
	EICRA &= ~(0 << ISC11);
	EICRA &= ~(0 << ISC10);
	
	// reactivate interrupts
	EIMSK |= (1 << INT1);
	
	// activate interrupts on PCINT1 pin 
	PCMSK0 |= (1 << PCINT1);
		
	return 0;
}