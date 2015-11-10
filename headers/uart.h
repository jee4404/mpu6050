/*
 * IncFile1.h
 *
 * Created: 2015-09-26 12:28:04
 *  Author: remy mourard
 */ 
#ifndef UART0_H
#define UART0_H

#ifndef BAUD
#define BAUD 9600
#endif

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <stdio.h>
#include <util/setbaud.h>
#include <avr/io.h>

/*
 * init uart0 of Atmega 2560 - will be sync
 * with serial term
 */
void uart0_init();

/*
 * Transmettre sur le USART
 */
void uart0_put_char(char c, FILE* stream);

/*
 * Recevoir sur le USART
 */
char uart0_read_char(FILE* stream);

/*
 * flux pour E/S avec le terminal
 */
FILE uartIO;

#endif /* UART0_H */