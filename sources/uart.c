/*
 * Usart.c
 *
 * Created: 2015-09-26 12:29:13
 *  Author: remy mourard
 */
#include "uart.h"

void uart0_init()
{
	// uart speed - default
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
	
	// @todo read the doc again
	#if USE_2X
		UCSR0A |= _BV(U2X0);
	#else
		UCSR0A &= ~(_BV(U2X0));
	#endif
	
	//enable Tx/Rx
	UCSR0B |= (1 << TXEN0) | (1 << RXEN0);
	
	// data size, a byte
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);
}


void uart0_put_char(char c, FILE* stream)
{
	// wait until uart ready to Tx
	loop_until_bit_is_set(UCSR0A, UDRE0);
	
	// transmitter pret, on peut initialiser le registre
	UDR0 = c;
}


char uart0_read_char(FILE* stream)
{
	// wait byte on uart ready to Rx
	loop_until_bit_is_set(UCSR0A, RXC0);
	
	return UDR0;
}

FILE uartIO = FDEV_SETUP_STREAM(uart0_put_char, uart0_read_char, _FDEV_SETUP_RW);