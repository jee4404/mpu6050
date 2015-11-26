/*
 * general_setup.h
 *
 * Created: 2015-11-04 17:45:28
 *  Author: remy mourard
 */ 

#ifndef GENERAL_SETUP_H_
#define GENERAL_SETUP_H_

/************************************************************************/
/* @details enable global interrupts                                    */
/************************************************************************/
 void global_int_enable();

/************************************************************************/
/* @details enable external interrupts from accelerometer               */
/*          interrupts are binded to INT2 (Port D - Bit 2)              */
/*          so they are managed by EICRA register (ISC20 and ISC21 bits)*/
/*          for sense control, EIMSK (INT2 bit) register for enabling   */
/*          the interruption                                            */
/************************************************************************/
void  accel_int_enable();

#endif /* GENERAL_SETUP_H_ */