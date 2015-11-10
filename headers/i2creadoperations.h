/*
 * i2creadoperations.h
 *
 * Created: 2015-11-05 20:46:38
 *  Author: remy
 */ 


#ifndef I2CREADOPERATIONS_H_
#define I2CREADOPERATIONS_H_

#include "i2cmaster.h"

/************************************************************************/
/* read bytes from i2c slave                                            */
/* @param device_addr slave device address                              */
/* @param reg_addr slave device register to read from                   */
/* @param length of data                                                */
/* @param data                                                          */
/************************************************************************/
int8_t i2c_read_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);

/************************************************************************/
/* read a single byte                                                   */
/************************************************************************/
int8_t i2c_read_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t *data);

/************************************************************************/
/* write bytes to i2c slave                                             */
/* @param device_addr salve device address                              */
/* @param reg_addr slave device register to wrtie to                    */
/* @param length length of data                                         */
/* @param data                                                          */
/************************************************************************/
int8_t i2c_write_bytes(uint8_t device_addr, uint8_t reg_addr, uint8_t length, uint8_t* data);

/************************************************************************/
/* write a single byte                                                  */
/************************************************************************/
int8_t i2c_write_byte(uint8_t device_addr, uint8_t reg_addr, uint8_t* data);

#endif /* I2CREADOPERATIONS_H_ */