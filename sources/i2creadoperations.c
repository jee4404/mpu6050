/*
 * i2creadoperations.c
 *
 * Created: 2015-11-05 20:46:53
 * Author: Remy Mourard
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
    uint8_t i = 0, twi_status = 0, tries = 1;
    int8_t count = 0;
    if(length > 0)
    {
	    // send start
        twi_status = i2c_start(device_addr, TW_WRITE);
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
            //printf("[i2c_read] error while issuing i2c start condition [0x%X]\n", twi_status);
            i2c_stop();
            if(tries < NOACK_TRIES)
            {
                ++tries;
                i2c_read_bytes(device_addr, reg_addr, length, data);
            }
        }
        else
        {
            if((twi_status = i2c_transmit(reg_addr)) == TW_MT_DATA_ACK)
            {
	            _delay_us(10);
	            //read data
	            go_on = 0;
                twi_status = i2c_start(device_addr, TW_READ);
	            
                switch(twi_status)
                {
                    case TW_REP_START: // success not sure about it
                    case TW_MR_SLA_ACK:
                        go_on = 1;
                        break;
                    default: // error
                        break;
                }
                
                if(go_on == 1)
                {
	                for(i=0; i<length; i++)
                    {
		                count++;
                        if(i == length-1)
                            twi_status = i2c_receive(&data[i], 1);
                        else
                            twi_status = i2c_receive(&data[i], 0);
	                }
	                i2c_stop();
                }
                else
                {
                    //printf("[i2c_read] error while issuing i2c repeated start condition [0x%X] retrying...\n", twi_status);
                    i2c_stop();
                    if(tries < NOACK_TRIES)
                    {
                        ++tries;
                        i2c_read_bytes(device_addr, reg_addr, length, data);
                    }
                }                
            }
            else
            {
                //printf("[i2c_read] error while issuing i2c transmit condition [0x%X]\n", twi_status);
                i2c_stop(); // no sure if i2c stop is necessary...
                if(tries < NOACK_TRIES)
                {
                    ++tries;
                    i2c_read_bytes(device_addr, reg_addr, length, data);
                }
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
    uint8_t twi_status, count = 0, tries = 0;
    if(length > 0)
    {
        if((twi_status = i2c_start(device_addr, TW_WRITE)) == TW_MT_SLA_ACK)
        {
            if((twi_status = i2c_transmit(reg_addr)) == TW_MT_DATA_ACK)
            {
                for(int i = 0; i < length; i++)
                {
                    ++count;
                    twi_status = i2c_transmit((uint8_t) data[i]);
                }
                i2c_stop();
            }
            else
            {
                printf("[i2c_write] error while issuing i2c_transmit [0x%X]\n", twi_status);
                i2c_stop();
                if(tries < NOACK_TRIES)
                {
                    ++tries;
                    i2c_write_bytes(device_addr, reg_addr, length, data);
                }
            }
        }
        else
        {
            printf("[i2c_write] error while issuing i2c start condition [0x%X]\n", twi_status);
            i2c_stop();
            if(tries < NOACK_TRIES)
            {
                ++tries;
                i2c_write_bytes(device_addr, reg_addr, length, data);
            }
        }
    }        
    return count;
}

uint8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* data)
{
    return i2c_write_bytes(device_addr, reg_addr, 1, data);
}

uint8_t read_bits_from_byte(uint8_t offset, uint8_t length, uint8_t data)
{
    return (data >> offset) & ((1 << length) - 1);
}