/*
 * accel_registers.h
 *
 * Created: 2015-11-04 20:07:16
 *  Author: remy
 */ 


#ifndef ACCEL_REGISTERS_H_
#define ACCEL_REGISTERS_H_

#define MPU6050_DMP_CODE_SIZE 1929 // the day I understand why i have to put bytes in dmp...
#define MPU6050_DMP_CONFIG_SIZE 192 // i swear i'll explain that
#define MPU6050_DMP_UPDATES_SIZE 47

#define MPU_ADDRESS 0x69;      // mine is 
#define MPU_CONFIG 0x1A;       // config register
#define	MPU_SMPRT_DIV 0x19     // sample rate divider register
#define MPU_INT_EN 0x38        // interrupt enable register
#define MPU_ACCEL_CONFIG 0x1C  // accelerometer configuration register
#define MPU_FIFO_EN 0x23	   // FIFO enable register
#define MPU_INT_PIN_CFG 0x37   // Interrupt pin / bypass enable configuration register
#define MPU_USER_CTRL 0x6A	   // User control register (fifo enable, reset, i2c mst enable...)

#endif /* ACCEL_REGISTERS_H_ */