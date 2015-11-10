/*
 * I2CTest.c
 *
 * Created: 2015-10-14 19:42:16
 * Author : remy
 */ 

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart.h"
#include "accelgyro.h"

int main(void)
{
    uart0_init();
    
	stdout = &uartIO;
	stdin  = &uartIO;
		
	DDRB |= (1 << DDB7);
    /* Replace with your application code */
    while (1)
    {
		PORTB ^= (1 << PB7);
		_delay_ms(500);
        initialize_accelgyro();
        exit(0);
    }
}

