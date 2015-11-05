/*
 * general_setup.h
 *
 * Created: 2015-11-04 17:45:28
 *  Author: remy
 */ 

#ifndef GENERAL_SETUP_H_
#define GENERAL_SETUP_H_

#include <avr/io.h>
#include <stdint.h>

/************************************************************************/
/* enable global interrupts                                             */
/************************************************************************/
 uint8_t global_int_enable();

/************************************************************************/
/* enable external interrupts for accelerometer                         */
/************************************************************************/
uint8_t  accel_int_enable();

#endif /* GENERAL_SETUP_H_ */