/*
 * I2C master routines. Inspired by Jeff Rowber I2C devlib
 * and Peter Fleury improved lib.
 */

#include "i2cmaster.h"
#include <stdio.h>
/* define CPU frequency in Mhz here if not defined in Makefile */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

/* I2C timer max delay */
#define I2C_TIMER_DELAY 0xFF

/************************************************************************/
/* @details Initialization of the I2C bus interface. Need to be called  */
/* only once. Current I2C devices are MPU 6050 which max I2C clock freq */
/* is 400 KHz and                                                       */
/************************************************************************/
void i2c_init(void)
{
    // set SCL frequency. see atmega 2560 data sheet p248 
    // for how to set TWBR according to desired SCL frequency
    TWBR = 0x08; // 3 into TWBR -> 200 kHz
    TWSR &= ~(1 << TWPS0); // pre-scaler to 1
    TWSR &= ~(1 << TWPS1);
    
}/* i2c_init */

/************************************************************************/
/* @details sends a start condition to slave, make TWDR ready for data  */
/* @param address : adress of slave                                     */
/* @param mode : Read 1|Write 0                                         */
/************************************************************************/
uint8_t i2c_start(uint8_t address, uint8_t mode)
{
    // start condition
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    
    // wait for TWINT flag indicating tha start has been Tx
    while (!(TWCR & (1<<TWINT)));
    
    if(TW_STATUS != TW_START) // start condition not sent
        return TW_STATUS;
    
    // now decide if well Tx(SLA+W or I2C_WRITE) or Rx(SLA+R or I2C_READ)
    TWDR = (address << 1) | mode;
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // wait for TWINT flag indicating that SLA+mode have been transmitted
    while (!(TWCR & (1<<TWINT)));
    
    // TWDR is now ready (or not, the caller take action) for Tx/Rx data :)
    return TW_STATUS;
}

void i2c_stop(void)
{
    /* send stop condition */
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
	
	// wait until stop condition is executed and bus released
	while((TWCR & (1<<TWSTO)));
}

uint8_t i2c_transmit( uint8_t data )
{
    // we want to Tx
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // wait for TWINT to be cleared before putting data into TWDR
    //while((TWCR & (1 << TWINT)) == 1);
    while (!(TWCR & (1<<TWINT)));
    
    // put data in TWDR
    TWDR = data;
    // and send it
    TWCR = (1 << TWINT) | (1 << TWEN);
    
    // wait for ack of slave
    //while((TWCR & (1 << TWINT)) == 1);
    while (!(TWCR & (1<<TWINT)));
    
    // return status, error handling responsibility of the caller
    return TW_STATUS;
}

uint8_t i2c_receive(uint8_t* data)
{
    // we want to Rx
    TWCR = (1 << TWINT) | (1 << TWEN);   
    // wait for data available on TWDR
    //while((TWCR & (1 << TWINT)) == 1);
    while (!(TWCR & (1<<TWINT)));
    
    *data = TWDR;
    return TW_STATUS;
}