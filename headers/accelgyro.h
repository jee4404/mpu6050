/*
 * accel.h
 *
 * Created: 2015-11-04 18:40:20
 *  Author: remy
 */ 


#ifndef ACCEL_H_
#define ACCEL_H_

#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "accel_registers.h"

/************************************************************************/
/* @details interrupt flag from MPU                                     */
/************************************************************************/
uint8_t interrupt_accel_flag;

/************************************************************************/
/* @details clear interrupt flag from MPU                               */
/************************************************************************/
void clear_interrupt_accel_flag();

/************************************************************************/
/* @details configure the MPU 6050 basic features                       */
/************************************************************************/
void configure_accelgyro();

/************************************************************************/
/* @details configure the dmp                                           */
/************************************************************************/
void confiugre_dmp();

/************************************************************************/
/* @details test connection with a who am i                             */
/************************************************************************/
uint8_t test_connection();

/************************************************************************/
/* @details call test_connection, configure_accelgyro and configure_dmp */
/************************************************************************/
uint8_t initialize_accelgyro();

/************************************************************************/
/* @details perform accel and gyro self test                            */
/************************************************************************/
int8_t run_accelgyro_selftest();

/************************************************************************/
/* @details check for correct accelgyro config                          */
/************************************************************************/
void check_accelgyro_configure();
#endif /* ACCEL_H_ */