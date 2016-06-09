/*
 * I2CTest.c
 *
 * Created: 2015-10-14 19:42:16
 * Author : remy
 */ 

#define DEBUG 1

#ifndef F_CPU
	#define F_CPU 16000000UL
#endif

#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

#include "general_setup.h"
#include "uart.h"
#include "i2cmaster.h"
#include "accelgyro.h"

int main(void)
{   
    uint8_t int_status;
    uint8_t fifo_packet_size = 12;
    uint16_t fifo_count;
    uint8_t fifo_buffer[64];
    
    int16_t accel_x, accel_y, accel_z; // Sensor values
    int16_t gyro_x, gyro_y, gyro_z;
        
    Quaternion quaternion;
    VectorFloat gravity;
    float ypr[3];
    
    // global int enable
    global_int_enable();
    
    // Initialize i2c
    i2c_init();
    
    // Initialize UART
    uart0_init();
    // and redirect stdin/out to UART 0 for debug via serial terminal
	stdout = &uartIO;
	stdin  = &uartIO;
    
    initialize_accelgyro();
//     configure_dmp();
//     set_dmp_enabled(1);
    accel_int_enable();
    
    while (1)
    {
		while(interrupt_accel_flag == 0)
            fifo_count = get_fifo_count();
        
        if(interrupt_accel_flag == 1)
        {
            get_fifo_bytes(fifo_buffer, fifo_packet_size);
            get_int_status(&int_status); // clear int status register
            clear_interrupt_accel_flag();
            if(int_status & 0x10)
            {
                //reset_fifo();
            }
            else if(int_status & 0x01) // FIFO data ready
            {
                // data in FIFO is ordered by register number :
                // byte 0, 1 -> accel x out (h, l)
                // byte 2, 3 -> accel y out
                // byte 4, 5 -> accel z out
                // byte 6, 7 -> gyro x out
                // byte 8, 9 -> gyro y out
                // byte 10, 11 -> gyro z out
                accel_x = (((int16_t)fifo_buffer[0]) << 8)|fifo_buffer[1];
                accel_y = (((int16_t)fifo_buffer[2]) << 8)|fifo_buffer[3];
                accel_z = (((int16_t)fifo_buffer[4]) << 8)|fifo_buffer[5];
                
                gyro_x = (((int16_t)fifo_buffer[6]) << 8)|fifo_buffer[7];
                gyro_y = (((int16_t)fifo_buffer[8]) << 8)|fifo_buffer[9];
                gyro_z = (((int16_t)fifo_buffer[10]) << 8)|fifo_buffer[11];
                
                printf("a/g %d %d %d %d %d %d\n", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z);
                _delay_ms(250);
            }
            else if(int_status & 0x02)
            {
                // DMP data ready
                get_quaternion(&quaternion, fifo_buffer);
                
                printf("quaternion values : %f.3 %f.3 %f.3 %f.3\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
                dmp_get_gravity(&gravity, &quaternion);
                
                dmp_get_ypr(ypr, &quaternion, &gravity);
                
                printf("YAW : [%f]\n", ypr[0]*180/M_PI);
                printf("PITCH : [%f]\n", ypr[1]*180/M_PI);
                printf("ROLL : [%f]\n", ypr[2]*180/M_PI);
            }
        }
    }
}

