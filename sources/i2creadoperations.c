/*
 * i2creadoperations.c
 *
 * Created: 2015-11-05 20:46:53
 * Author: remy
 * Most functions copyright (c) Davide Gironi, 2012
 * Released under GPLv3.
 * Please refer to LICENSE file for licensing information.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
 
#include "i2creadoperations.h"

#include <util/delay.h>

/*
 * read bytes from i2c slave
 */
int8_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
    uint8_t i = 0;
    int8_t count = 0;
    if(length > 0)
    {
	    //request register
	    i2c_start(device_addr | I2C_WRITE);
	    i2c_write(reg_addr);
	    _delay_us(10);
	    //read data
	    i2c_start(device_addr | I2C_READ);
	    for(i=0; i<length; i++)
        {
		    count++;
		    if(i==length-1)
			    data[i] = i2c_readNak();
		    else
			    data[i] = i2c_readAck();
	    }
	    i2c_stop();
    }
    return count;
}

int8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data)
{
    return i2c_read_bytes(device_addr, reg_addr, 1, data);
}

int8_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
    int8_t ret = -1;
    if(length > 0)
    {
        //write data
        i2c_start(device_addr | I2C_WRITE);
        i2c_write(reg_addr); //reg
        for (uint8_t i = 0; i < length; i++) {
            i2c_write((uint8_t) data[i]);
        }
        i2c_stop();
        ret = 0;
    }
    return ret;
}

int8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* data)
{
    return i2c_write_bytes(device_addr, reg_addr, 1, data);
}