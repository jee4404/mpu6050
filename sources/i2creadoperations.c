/*
 * i2creadoperations.c
 *
 * Created: 2015-11-05 20:46:53
 * Author: remy mourard
 * Most functions copyright (c) Davide Gironi, 2012
 * Released under GPLv3.
 * Please refer to LICENSE file for licensing information.
 */
#ifndef F_CPU
#define F_CPU 16000000UL
#endif
 
#include "i2creadoperations.h"
#include <util/delay.h>
#include <stdio.h>
/*
 * read bytes from i2c slave
 */
uint8_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
    uint8_t i = 0;
    int8_t count = 0;
    if(length > 0)
    {
	    // send start
        uint8_t twi_status = i2c_start(device_addr, TW_WRITE);
        uint8_t go_on = 0;
        switch(twi_status)
        {
            case TW_MT_SLA_ACK: //success SLA+W :)
            case TW_MR_SLA_ACK: //success SLA+R
                go_on = 1;
                break;
        }
        
	    if(go_on == 0)
        {
            printf("error while issuing i2c start condition [0x%X]\n", twi_status);
            return 0; // return 0 byte read ?
        }
        else
        {
            go_on = 0;
            twi_status = i2c_transmit(reg_addr);
            switch(twi_status)
            {
                case TW_MT_DATA_ACK: // success data transmitted, ack received
                    go_on = 1;
                    break;
                    
                default:
                case TW_MT_DATA_NACK: // error, data transmitted, nack received
                    go_on = 0;
                    break;
            }                
	        if(go_on == 1)
            {
	            _delay_us(10);
	            //read data
	            i2c_start(device_addr, TW_READ);
                
	            for(i=0; i<length; i++)
                {
		            count++;
                    twi_status = i2c_receive(&data[i]);
	            }
	            i2c_stop();
            }
            else
            {
                printf("error while issuing i2c transmit condition [0x%X]\n", twi_status);
            }            
        }        
    }
    return count;
}

uint8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* data)
{
    return i2c_read_bytes(device_addr, reg_addr, 1, data);
}

uint8_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data)
{
    /*int8_t ret = 0;
    if(length > 0)
    {
        //write data
        i2c_start(device_addr, TW_WRITE);
        i2c_transmit(reg_addr); //reg
        
        for (uint8_t i = 0; i < length; i++) {
            ret += i2c_transmit((uint8_t) data[i]);
        }
        i2c_stop();
    }
    return ret;*/
    return 0;
}

uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* data)
{
    return i2c_write_bytes(device_addr, reg_addr, 1, data);
}

uint8_t read_bits_from_byte(uint8_t offset, uint8_t length, uint8_t data)
{
    printf("[read_bits_from_byte] data : 0x%X\n", data);
    printf("[read_bits_from_byte] return : 0x%X\n", (data >> offset) & ((1 << length) - 1));
    return (data >> offset) & ((1 << length) - 1);
}