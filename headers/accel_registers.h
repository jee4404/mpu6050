/*
 * accel_registers.h
 *
 * Created: 2015-11-04 20:07:16
 *  Author: remy mourard
 */ 


#ifndef ACCEL_REGISTERS_H_
#define ACCEL_REGISTERS_H_

/************************************************************************/
/* MPU6050 constants                                                    */
/************************************************************************/
#define MPU_DMP_CHUNK_SIZE 16
#define MPU_DMP_CODE_SIZE 1929 // the day I understand why i have to put bytes in dmp...
#define MPU_DMP_CONFIG_SIZE 192 // i swear i'll explain that
#define MPU_DMP_UPDATES_SIZE 47
#define WHO_AM_I 0x34

/************************************************************************/
/* MPU6050 register address                                             */
/************************************************************************/

// following three taken from I2C Devlib
#define MPU_XG_OFFS_TC       0x00 //[7] PWR_MODE, [6:1] XG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_YG_OFFS_TC       0x01 //[7] PWR_MODE, [6:1] YG_OFFS_TC, [0] OTP_BNK_VLD
#define MPU_ZG_OFFS_TC       0x02 //[7] PWR_MODE, [6:1] ZG_OFFS_TC, [0] OTP_BNK_VLD

#define MPU_ADDRESS           0x69 // mine is 
#define MPU_CONFIG            0x1A // config register
#define	MPU_SMPRT_DIV         0x19 // sample rate divider register
#define MPU_INT_EN            0x38 // interrupt enable register
#define MPU_ACCEL_CONFIG      0x1C // accelerometer configuration register
#define MPU_GYRO_CONFIG       0x1B // gyro configuration register
#define MPU_FIFO_EN           0x23 // FIFO enable register
#define MPU_INT_PIN_CFG       0x37 // Interrupt pin / bypass enable configuration register
#define MPU_SIGNAL_PATH_RESET 0x68 // signal path reset register, for reseting analog and digital accelerometer/gyro path 
#define MPU_USER_CTRL         0x6A // User control register (FIFO enable, reset, i2c master enable...)
#define MPU_PWR_MGT_1         0x6B // power management 1 : sleep mode, device reset, disable temp sensor, clock select
#define MPU_PWR_MGT_2         0x6C // power management 2 : put accelerometer / gyro in standby mode
#define MPU_FIFO_CNT_H        0x72 // FIFO count high
#define MPU_FIFO_CNT_L        0x73 // FIFO count low
#define MPU_FIFO_DATA         0x74 // FIFO data
#define MPU_WHO_AMI           0x75 // who am i for i2c
#define MPU_MOT_THR           0x1F // motion threshold register, not in data sheet
#define MPU_MOT_DUR           0x20 // motion detection duration register
#define MPU_ZRMOT_THR         0x21 // zero motion threshold register
#define MPU_ZRMOT_DUR         0x22 // zero motion duration
#define MPU_I2C_SLV0          0x25 // I2c 0 slave register address

#define MPU_ACCEL_XOUT_H      0x3B // accelerometer X out high
#define MPU_ACCEL_XOUT_L      0x3C // accelerometer X out low
#define MPU_ACCEL_YOUT_H      0x3D // accelerometer Y out high
#define MPU_ACCEL_YOUT_L      0x3E // accelerometer Y out low
#define MPU_ACCEL_ZOUT_H      0x3F // accelerometer Z out high
#define MPU_ACCEL_ZOUT_L      0x40 // accelerometer Z out low

#define MPU_GYRO_XOUT_H       0x43 // gyro X out high
#define MPU_GYRO_XOUT_L       0x44 // gyro X out high
#define MPU_GYRO_YOUT_H       0x45 // gyro X out high
#define MPU_GYRO_YOUT_L       0x46 // gyro X out high
#define MPU_GYRO_ZOUT_H       0x47 // gyro X out high
#define MPU_GYRO_ZOUT_L       0x48 // gyro X out high

#define MPU_INT_STATUS        0x3A // interrupt status register
/************************************************************************/
/* DMP register and PGM space address                                   */
/************************************************************************/
#define MPU_BANK_SEL         0x6D
#define MPU_MEM_START_ADDR   0x6E
#define MPU_MEM_R_W          0x6F
#define MPU_DMP_CFG_1        0x70
#define MPU_DMP_CFG_2        0x71

#endif /* ACCEL_REGISTERS_H_ */