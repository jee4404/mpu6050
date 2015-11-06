/*
 * Usart.c
 *
 * Created: 2015-09-26 12:29:13
 *  Author: remy mourard
 */
#include "uart.h"

void UART0Init()
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


void UART0PutChar(char c, FILE* stream)
{
	// wait until uart ready to Tx
	loop_until_bit_is_set(UCSR0A, UDRE0);
	
	// transmitter pret, on peut initialiser le registre
	UDR0 = c;
}


char UART0ReadChar(FILE* stream)
{
	// wait byte on uart ready to Rx
	loop_until_bit_is_set(UCSR0A, RXC0);
	
	return UDR0;
}

FILE uartIO = FDEV_SETUP_STREAM(UART0PutChar, UART0ReadChar, _FDEV_SETUP_RW);