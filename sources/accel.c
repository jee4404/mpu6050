/*
 * accel.c
 *
 * Created: 2015-11-04 18:39:56
 *  Author: remy
 */ 
#include "accel.h"

uint8_t interrupt_accel_flag = 0;

uint8_t configure()
{
	// 1. sample rate, try 1khz ? seems like accel output rate is 1khz
	// so why shouting a lot of bit...
	// sample rate = gyro output rate / (1+SMPLRT_DIV)
	// 1khz = 8khz/(1+7)
	// MPU_SMPRT_DIV <- 0000 1101
	
	// 2. configuration register
	// digital low pass to 0 and FSYNC input disabled
	
	// 3. enable interrupts from FIFO oflow and Data ready
	// MPU_INT_EN xxx1 xxx1
	
	// 4. config accel range to +- 16g
	// ACCEL_CONFIG  xxx1 1xxx
	
	// 5. enable fifo only for accel values
	// warning, this prevent gyro and mpu's i2c slave
	// pushing data in fifo
	// FIFO_EN 0000 1000
	
	// 6. configure int pin
	// interrupt level active low : 1 on 7th bit
	// interrupt pin configured as a push pull : 0 on 6th bit
	// interrupt pin emits a 50us long pulse : 0 on 5th bit
	// interrupt status bit are cleared on any read operation : 1 on 4th bit
	// no interrupt generated on FSYNC pin for now : 0 on 2nd bit
	
	// MPU_INT_PIN_CFG 
	
	// 7. user control register
	// enable use of FIFO (6th bit), disable I2c master (5th bit)
	// always 4th bit to 0 for MPU6050, bit 2:0 are reset so can be set to 0
	// bit 7 and 3 arent used
	// MPU_USER_CTRL 0100 0000
	
	return 0;
}

uint8_t confiugre_dmp()
{
	return 0;
}

/************************************************************************/
/* handle interrupts thrown by accel                                    */
/************************************************************************/
ISR(PCINT1_vect)
{
	interrupt_accel_flag = 1;
}

void clear_interrupt_accel_flag()
{
	interrupt_accel_flag = 0;
}