/*
 * I2CTest.c
 *
 * Created: 2015-10-14 19:42:16
 * Author : remy
 */ 

#define DBG 1

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "general_setup.h"
#include "uart.h"
#include "i2cmaster.h"
#include "accelgyro.h"

int main(void)
{
    // global int
    //global_int_enable();
    
    // init i2c
    i2c_init();
    
    // init uart
    uart0_init();
    // redirect stdin/out vers uart 0
	stdout = &uartIO;
	stdin  = &uartIO;
    
    initialize_accelgyro();
    
	DDRB |= (1 << DDB7);
    /* Replace with your application code */
    while (1)
    {
		PORTB ^= (1 << PB7);
		_delay_ms(500);
    }
}

