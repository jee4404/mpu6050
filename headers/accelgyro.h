/*
 * accel.h
 *
 * Created: 2015-11-04 18:40:20
 *  Author: remy
 */ 


#ifndef ACCEL_H_
#define ACCEL_H_

#ifndef F_CPU
#define F_CPU 16000000L
#endif

#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "accel_registers.h"

/************************************************************************/
/* @brief clk selection different values                              */
/************************************************************************/
enum CLK_SEL {
    INTERNAL,
    PLL_X_GYRO,
    PLL_Y_GYRO,
    PLL_Z_GYRO,
    PLL_EXT_32,
    PLL_EXT_19,
    RESERVED,
    STOP
};

/************************************************************************/
/* @brief digital low pass filter config values                         */
/************************************************************************/
enum DLPF_CFG {
    DLPF_0,
    DLPF_1,
    DLPF_2,
    DLPF_3,
    DLPF_4,
    DLPF_5,
    DLPF_6
};

/************************************************************************/
/* @brief accelerometer available range values                          */
/************************************************************************/
enum ACCEL_RANGE {
    ACCEL_G2,
    ACCEL_G4,
    ACCEL_G5,
    ACCEL_G6    
};

/************************************************************************/
/* @brief gyroscope available range values                              */
/************************************************************************/
enum GYRO_RANGE {
    GYRO_DEG_250,
    GYRO_DEG_500,
    GYRO_DEG_1000,
    GYRO_DEG_2000
};

/************************************************************************/
/* @brief FIFO enable register available values                         */
/************************************************************************/
enum FIFO_ENABLE {
    SLV0_FIFO_EN   = 1,
    SLV1_FIFO_EN   = 2,
    SLV2_FIFO_EN   = 4,
    ACCEL_FIFO_EN  = 8,
    GYRO_Z_FIFO_EN = 16,
    GYRO_Y_FIFO_EN = 32,
    GYRO_X_FIFO_EN = 64,
    TEMP_FIFO_EN   = 128
};

/************************************************************************/
/* @brief interrupt flag from MPU                                       */
/************************************************************************/
uint8_t interrupt_accel_flag;

/************************************************************************/
/* @brief clear interrupt flag from MPU                                 */
/************************************************************************/
void clear_interrupt_accel_flag();

/************************************************************************/
/* @brief configure the MPU 6050 basic features                         */
/************************************************************************/
void configure_accelgyro();

/************************************************************************/
/* @brief configure the DMP                                             */
/************************************************************************/
void confiugre_dmp();

/************************************************************************/
/* @brief test connection with a who am i                               */
/************************************************************************/
uint8_t test_connection();

/************************************************************************/
/* @brief call test_connection, configure_accelgyro and configure_dmp   */
/************************************************************************/
uint8_t initialize_accelgyro();

/************************************************************************/
/* @brief set sample rate register                                      */
/************************************************************************/
void set_sample_rate(uint8_t value);

/************************************************************************/
/* @brief ake/sleep device                                              */
/************************************************************************/
void set_sleep_mode(uint8_t enable);

/************************************************************************/
/* @brief select clock source                                           */
/************************************************************************/
void set_clk_selection(enum CLK_SEL clk_value);

/************************************************************************/
/* @brief select DLPF bandwidth                                         */
/************************************************************************/
void set_dlpf(enum DLPF_CFG dlpf_value);

/************************************************************************/
/* @brief set accelerometer range                                       */
/************************************************************************/
void set_accel_range(enum ACCEL_RANGE range_value);

/************************************************************************/
/* @brief set gyroscope range                                           */
/************************************************************************/
void set_gyro_range(enum GYRO_RANGE range_value);

/************************************************************************/
/* @brief configure FIFO enable register                                */
/************************************************************************/
void set_fifo_enable_for(enum FIFO_ENABLE fifo_value);

/************************************************************************/
/* @brief enable FIFO operations in user control register               */
/************************************************************************/
void set_fifo_enabled(uint8_t enable);

/************************************************************************/
/* @brief reset FIFO buffer                                             */
/************************************************************************/
void reset_fifo();
#endif /* ACCEL_H_ */