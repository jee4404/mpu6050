/*
 * accel.c
 *
 * Created: 2015-11-04 18:39:56
 * Author: Remy Mourard
 * Credit for dmpMemory, dmpConfig and dmpUpadtes bytes array : J Rowberg / I2C Devlib
 */
#include "accelgyro.h"
#include "i2creadoperations.h"
#include "general_setup.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>

#undef DEBUG

/* ================================================================================================ *
 | Default MotionApps v2.0 42-byte FIFO packet structure:                                           |
 |                                                                                                  |
 | [QUAT W][      ][QUAT X][      ][QUAT Y][      ][QUAT Z][      ][GYRO X][      ][GYRO Y][      ] |
 |   0   1   2   3   4   5   6   7   8   9  10  11  12  13  14  15  16  17  18  19  20  21  22  23  |
 |                                                                                                  |
 | [GYRO Z][      ][ACC X ][      ][ACC Y ][      ][ACC Z ][      ][      ]                         |
 |  24  25  26  27  28  29  30  31  32  33  34  35  36  37  38  39  40  41                          |
 * ================================================================================================ */

// this block of memory gets written to the MPU on start-up, and it seems
// to be volatile memory, so it has to be done each time (it only takes ~1
// second though)
const uint8_t dmp_memory[MPU_DMP_CODE_SIZE] PROGMEM = {
    // bank 0, 256 bytes
    0xFB, 0x00, 0x00, 0x3E, 0x00, 0x0B, 0x00, 0x36, 0x00, 0x01, 0x00, 0x02, 0x00, 0x03, 0x00, 0x00,
    0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0xFA, 0x80, 0x00, 0x0B, 0x12, 0x82, 0x00, 0x01,
    0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x28, 0x00, 0x00, 0xFF, 0xFF, 0x45, 0x81, 0xFF, 0xFF, 0xFA, 0x72, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x03, 0xE8, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x7F, 0xFF, 0xFF, 0xFE, 0x80, 0x01,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x3E, 0x03, 0x30, 0x40, 0x00, 0x00, 0x00, 0x02, 0xCA, 0xE3, 0x09, 0x3E, 0x80, 0x00, 0x00,
    0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00,
    0x41, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x0B, 0x2A, 0x00, 0x00, 0x16, 0x55, 0x00, 0x00, 0x21, 0x82,
    0xFD, 0x87, 0x26, 0x50, 0xFD, 0x80, 0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00, 0x05, 0x80, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x6F, 0x00, 0x02, 0x65, 0x32, 0x00, 0x00, 0x5E, 0xC0,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0xFB, 0x8C, 0x6F, 0x5D, 0xFD, 0x5D, 0x08, 0xD9, 0x00, 0x7C, 0x73, 0x3B, 0x00, 0x6C, 0x12, 0xCC,
    0x32, 0x00, 0x13, 0x9D, 0x32, 0x00, 0xD0, 0xD6, 0x32, 0x00, 0x08, 0x00, 0x40, 0x00, 0x01, 0xF4,
    0xFF, 0xE6, 0x80, 0x79, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD0, 0xD6, 0x00, 0x00, 0x27, 0x10,

    // bank 1, 256 bytes
    0xFB, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00,
    0x00, 0x00, 0xFA, 0x36, 0xFF, 0xBC, 0x30, 0x8E, 0x00, 0x05, 0xFB, 0xF0, 0xFF, 0xD9, 0x5B, 0xC8,
    0xFF, 0xD0, 0x9A, 0xBE, 0x00, 0x00, 0x10, 0xA9, 0xFF, 0xF4, 0x1E, 0xB2, 0x00, 0xCE, 0xBB, 0xF7,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x02, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0C,
    0xFF, 0xC2, 0x80, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0xCF, 0x80, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x00, 0x14,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x03, 0x3F, 0x68, 0xB6, 0x79, 0x35, 0x28, 0xBC, 0xC6, 0x7E, 0xD1, 0x6C,
    0x80, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0xB2, 0x6A, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3F, 0xF0, 0x00, 0x00, 0x00, 0x30,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x25, 0x4D, 0x00, 0x2F, 0x70, 0x6D, 0x00, 0x00, 0x05, 0xAE, 0x00, 0x0C, 0x02, 0xD0,

    // bank 2, 256 bytes
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x54, 0xFF, 0xEF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x00, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x01, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x65, 0x00, 0x00, 0x00, 0x54, 0x00, 0x00, 0xFF, 0xEF, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00,
    0x00, 0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

    // bank 3, 256 bytes
    0xD8, 0xDC, 0xBA, 0xA2, 0xF1, 0xDE, 0xB2, 0xB8, 0xB4, 0xA8, 0x81, 0x91, 0xF7, 0x4A, 0x90, 0x7F,
    0x91, 0x6A, 0xF3, 0xF9, 0xDB, 0xA8, 0xF9, 0xB0, 0xBA, 0xA0, 0x80, 0xF2, 0xCE, 0x81, 0xF3, 0xC2,
    0xF1, 0xC1, 0xF2, 0xC3, 0xF3, 0xCC, 0xA2, 0xB2, 0x80, 0xF1, 0xC6, 0xD8, 0x80, 0xBA, 0xA7, 0xDF,
    0xDF, 0xDF, 0xF2, 0xA7, 0xC3, 0xCB, 0xC5, 0xB6, 0xF0, 0x87, 0xA2, 0x94, 0x24, 0x48, 0x70, 0x3C,
    0x95, 0x40, 0x68, 0x34, 0x58, 0x9B, 0x78, 0xA2, 0xF1, 0x83, 0x92, 0x2D, 0x55, 0x7D, 0xD8, 0xB1,
    0xB4, 0xB8, 0xA1, 0xD0, 0x91, 0x80, 0xF2, 0x70, 0xF3, 0x70, 0xF2, 0x7C, 0x80, 0xA8, 0xF1, 0x01,
    0xB0, 0x98, 0x87, 0xD9, 0x43, 0xD8, 0x86, 0xC9, 0x88, 0xBA, 0xA1, 0xF2, 0x0E, 0xB8, 0x97, 0x80,
    0xF1, 0xA9, 0xDF, 0xDF, 0xDF, 0xAA, 0xDF, 0xDF, 0xDF, 0xF2, 0xAA, 0xC5, 0xCD, 0xC7, 0xA9, 0x0C,
    0xC9, 0x2C, 0x97, 0x97, 0x97, 0x97, 0xF1, 0xA9, 0x89, 0x26, 0x46, 0x66, 0xB0, 0xB4, 0xBA, 0x80,
    0xAC, 0xDE, 0xF2, 0xCA, 0xF1, 0xB2, 0x8C, 0x02, 0xA9, 0xB6, 0x98, 0x00, 0x89, 0x0E, 0x16, 0x1E,
    0xB8, 0xA9, 0xB4, 0x99, 0x2C, 0x54, 0x7C, 0xB0, 0x8A, 0xA8, 0x96, 0x36, 0x56, 0x76, 0xF1, 0xB9,
    0xAF, 0xB4, 0xB0, 0x83, 0xC0, 0xB8, 0xA8, 0x97, 0x11, 0xB1, 0x8F, 0x98, 0xB9, 0xAF, 0xF0, 0x24,
    0x08, 0x44, 0x10, 0x64, 0x18, 0xF1, 0xA3, 0x29, 0x55, 0x7D, 0xAF, 0x83, 0xB5, 0x93, 0xAF, 0xF0,
    0x00, 0x28, 0x50, 0xF1, 0xA3, 0x86, 0x9F, 0x61, 0xA6, 0xDA, 0xDE, 0xDF, 0xD9, 0xFA, 0xA3, 0x86,
    0x96, 0xDB, 0x31, 0xA6, 0xD9, 0xF8, 0xDF, 0xBA, 0xA6, 0x8F, 0xC2, 0xC5, 0xC7, 0xB2, 0x8C, 0xC1,
    0xB8, 0xA2, 0xDF, 0xDF, 0xDF, 0xA3, 0xDF, 0xDF, 0xDF, 0xD8, 0xD8, 0xF1, 0xB8, 0xA8, 0xB2, 0x86,

    // bank 4, 256 bytes
    0xB4, 0x98, 0x0D, 0x35, 0x5D, 0xB8, 0xAA, 0x98, 0xB0, 0x87, 0x2D, 0x35, 0x3D, 0xB2, 0xB6, 0xBA,
    0xAF, 0x8C, 0x96, 0x19, 0x8F, 0x9F, 0xA7, 0x0E, 0x16, 0x1E, 0xB4, 0x9A, 0xB8, 0xAA, 0x87, 0x2C,
    0x54, 0x7C, 0xB9, 0xA3, 0xDE, 0xDF, 0xDF, 0xA3, 0xB1, 0x80, 0xF2, 0xC4, 0xCD, 0xC9, 0xF1, 0xB8,
    0xA9, 0xB4, 0x99, 0x83, 0x0D, 0x35, 0x5D, 0x89, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0xB5, 0x93, 0xA3,
    0x0E, 0x16, 0x1E, 0xA9, 0x2C, 0x54, 0x7C, 0xB8, 0xB4, 0xB0, 0xF1, 0x97, 0x83, 0xA8, 0x11, 0x84,
    0xA5, 0x09, 0x98, 0xA3, 0x83, 0xF0, 0xDA, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xD8, 0xF1, 0xA5,
    0x29, 0x55, 0x7D, 0xA5, 0x85, 0x95, 0x02, 0x1A, 0x2E, 0x3A, 0x56, 0x5A, 0x40, 0x48, 0xF9, 0xF3,
    0xA3, 0xD9, 0xF8, 0xF0, 0x98, 0x83, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0x97, 0x82, 0xA8, 0xF1,
    0x11, 0xF0, 0x98, 0xA2, 0x24, 0x08, 0x44, 0x10, 0x64, 0x18, 0xDA, 0xF3, 0xDE, 0xD8, 0x83, 0xA5,
    0x94, 0x01, 0xD9, 0xA3, 0x02, 0xF1, 0xA2, 0xC3, 0xC5, 0xC7, 0xD8, 0xF1, 0x84, 0x92, 0xA2, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0x93, 0xA3, 0x4D,
    0xDA, 0x2A, 0xD8, 0x48, 0x69, 0xD9, 0x2A, 0xD8, 0x68, 0x55, 0xDA, 0x32, 0xD8, 0x50, 0x71, 0xD9,
    0x32, 0xD8, 0x70, 0x5D, 0xDA, 0x3A, 0xD8, 0x58, 0x79, 0xD9, 0x3A, 0xD8, 0x78, 0xA8, 0x8A, 0x9A,
    0xF0, 0x28, 0x50, 0x78, 0x9E, 0xF3, 0x88, 0x18, 0xF1, 0x9F, 0x1D, 0x98, 0xA8, 0xD9, 0x08, 0xD8,
    0xC8, 0x9F, 0x12, 0x9E, 0xF3, 0x15, 0xA8, 0xDA, 0x12, 0x10, 0xD8, 0xF1, 0xAF, 0xC8, 0x97, 0x87,

    // bank 5, 256 bytes
    0x34, 0xB5, 0xB9, 0x94, 0xA4, 0x21, 0xF3, 0xD9, 0x22, 0xD8, 0xF2, 0x2D, 0xF3, 0xD9, 0x2A, 0xD8,
    0xF2, 0x35, 0xF3, 0xD9, 0x32, 0xD8, 0x81, 0xA4, 0x60, 0x60, 0x61, 0xD9, 0x61, 0xD8, 0x6C, 0x68,
    0x69, 0xD9, 0x69, 0xD8, 0x74, 0x70, 0x71, 0xD9, 0x71, 0xD8, 0xB1, 0xA3, 0x84, 0x19, 0x3D, 0x5D,
    0xA3, 0x83, 0x1A, 0x3E, 0x5E, 0x93, 0x10, 0x30, 0x81, 0x10, 0x11, 0xB8, 0xB0, 0xAF, 0x8F, 0x94,
    0xF2, 0xDA, 0x3E, 0xD8, 0xB4, 0x9A, 0xA8, 0x87, 0x29, 0xDA, 0xF8, 0xD8, 0x87, 0x9A, 0x35, 0xDA,
    0xF8, 0xD8, 0x87, 0x9A, 0x3D, 0xDA, 0xF8, 0xD8, 0xB1, 0xB9, 0xA4, 0x98, 0x85, 0x02, 0x2E, 0x56,
    0xA5, 0x81, 0x00, 0x0C, 0x14, 0xA3, 0x97, 0xB0, 0x8A, 0xF1, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9,
    0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x84, 0x0D, 0xDA, 0x0E, 0xD8, 0xA3, 0x29, 0x83, 0xDA,
    0x2C, 0x0E, 0xD8, 0xA3, 0x84, 0x49, 0x83, 0xDA, 0x2C, 0x4C, 0x0E, 0xD8, 0xB8, 0xB0, 0xA8, 0x8A,
    0x9A, 0xF5, 0x20, 0xAA, 0xDA, 0xDF, 0xD8, 0xA8, 0x40, 0xAA, 0xD0, 0xDA, 0xDE, 0xD8, 0xA8, 0x60,
    0xAA, 0xDA, 0xD0, 0xDF, 0xD8, 0xF1, 0x97, 0x86, 0xA8, 0x31, 0x9B, 0x06, 0x99, 0x07, 0xAB, 0x97,
    0x28, 0x88, 0x9B, 0xF0, 0x0C, 0x20, 0x14, 0x40, 0xB8, 0xB0, 0xB4, 0xA8, 0x8C, 0x9C, 0xF0, 0x04,
    0x28, 0x51, 0x79, 0x1D, 0x30, 0x14, 0x38, 0xB2, 0x82, 0xAB, 0xD0, 0x98, 0x2C, 0x50, 0x50, 0x78,
    0x78, 0x9B, 0xF1, 0x1A, 0xB0, 0xF0, 0x8A, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x8B, 0x29, 0x51, 0x79,
    0x8A, 0x24, 0x70, 0x59, 0x8B, 0x20, 0x58, 0x71, 0x8A, 0x44, 0x69, 0x38, 0x8B, 0x39, 0x40, 0x68,
    0x8A, 0x64, 0x48, 0x31, 0x8B, 0x30, 0x49, 0x60, 0xA5, 0x88, 0x20, 0x09, 0x71, 0x58, 0x44, 0x68,

    // bank 6, 256 bytes
    0x11, 0x39, 0x64, 0x49, 0x30, 0x19, 0xF1, 0xAC, 0x00, 0x2C, 0x54, 0x7C, 0xF0, 0x8C, 0xA8, 0x04,
    0x28, 0x50, 0x78, 0xF1, 0x88, 0x97, 0x26, 0xA8, 0x59, 0x98, 0xAC, 0x8C, 0x02, 0x26, 0x46, 0x66,
    0xF0, 0x89, 0x9C, 0xA8, 0x29, 0x51, 0x79, 0x24, 0x70, 0x59, 0x44, 0x69, 0x38, 0x64, 0x48, 0x31,
    0xA9, 0x88, 0x09, 0x20, 0x59, 0x70, 0xAB, 0x11, 0x38, 0x40, 0x69, 0xA8, 0x19, 0x31, 0x48, 0x60,
    0x8C, 0xA8, 0x3C, 0x41, 0x5C, 0x20, 0x7C, 0x00, 0xF1, 0x87, 0x98, 0x19, 0x86, 0xA8, 0x6E, 0x76,
    0x7E, 0xA9, 0x99, 0x88, 0x2D, 0x55, 0x7D, 0x9E, 0xB9, 0xA3, 0x8A, 0x22, 0x8A, 0x6E, 0x8A, 0x56,
    0x8A, 0x5E, 0x9F, 0xB1, 0x83, 0x06, 0x26, 0x46, 0x66, 0x0E, 0x2E, 0x4E, 0x6E, 0x9D, 0xB8, 0xAD,
    0x00, 0x2C, 0x54, 0x7C, 0xF2, 0xB1, 0x8C, 0xB4, 0x99, 0xB9, 0xA3, 0x2D, 0x55, 0x7D, 0x81, 0x91,
    0xAC, 0x38, 0xAD, 0x3A, 0xB5, 0x83, 0x91, 0xAC, 0x2D, 0xD9, 0x28, 0xD8, 0x4D, 0xD9, 0x48, 0xD8,
    0x6D, 0xD9, 0x68, 0xD8, 0x8C, 0x9D, 0xAE, 0x29, 0xD9, 0x04, 0xAE, 0xD8, 0x51, 0xD9, 0x04, 0xAE,
    0xD8, 0x79, 0xD9, 0x04, 0xD8, 0x81, 0xF3, 0x9D, 0xAD, 0x00, 0x8D, 0xAE, 0x19, 0x81, 0xAD, 0xD9,
    0x01, 0xD8, 0xF2, 0xAE, 0xDA, 0x26, 0xD8, 0x8E, 0x91, 0x29, 0x83, 0xA7, 0xD9, 0xAD, 0xAD, 0xAD,
    0xAD, 0xF3, 0x2A, 0xD8, 0xD8, 0xF1, 0xB0, 0xAC, 0x89, 0x91, 0x3E, 0x5E, 0x76, 0xF3, 0xAC, 0x2E,
    0x2E, 0xF1, 0xB1, 0x8C, 0x5A, 0x9C, 0xAC, 0x2C, 0x28, 0x28, 0x28, 0x9C, 0xAC, 0x30, 0x18, 0xA8,
    0x98, 0x81, 0x28, 0x34, 0x3C, 0x97, 0x24, 0xA7, 0x28, 0x34, 0x3C, 0x9C, 0x24, 0xF2, 0xB0, 0x89,
    0xAC, 0x91, 0x2C, 0x4C, 0x6C, 0x8A, 0x9B, 0x2D, 0xD9, 0xD8, 0xD8, 0x51, 0xD9, 0xD8, 0xD8, 0x79,

    // bank 7, 138 bytes (remainder)
    0xD9, 0xD8, 0xD8, 0xF1, 0x9E, 0x88, 0xA3, 0x31, 0xDA, 0xD8, 0xD8, 0x91, 0x2D, 0xD9, 0x28, 0xD8,
    0x4D, 0xD9, 0x48, 0xD8, 0x6D, 0xD9, 0x68, 0xD8, 0xB1, 0x83, 0x93, 0x35, 0x3D, 0x80, 0x25, 0xDA,
    0xD8, 0xD8, 0x85, 0x69, 0xDA, 0xD8, 0xD8, 0xB4, 0x93, 0x81, 0xA3, 0x28, 0x34, 0x3C, 0xF3, 0xAB,
    0x8B, 0xF8, 0xA3, 0x91, 0xB6, 0x09, 0xB4, 0xD9, 0xAB, 0xDE, 0xFA, 0xB0, 0x87, 0x9C, 0xB9, 0xA3,
    0xDD, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x95, 0xF1, 0xA3, 0xA3, 0xA3, 0x9D, 0xF1, 0xA3, 0xA3, 0xA3,
    0xA3, 0xF2, 0xA3, 0xB4, 0x90, 0x80, 0xF2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3,
    0xA3, 0xB2, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xA3, 0xB0, 0x87, 0xB5, 0x99, 0xF1, 0xA3, 0xA3, 0xA3,
    0x98, 0xF1, 0xA3, 0xA3, 0xA3, 0xA3, 0x97, 0xA3, 0xA3, 0xA3, 0xA3, 0xF3, 0x9B, 0xA3, 0xA3, 0xDC,
    0xB9, 0xA7, 0xF1, 0x26, 0x26, 0x26, 0xD8, 0xD8, 0xFF
};

const uint8_t dmp_config[MPU_DMP_CONFIG_SIZE] PROGMEM = {
    //  BANK    OFFSET  LENGTH  [DATA]
    0x03,   0x7B,   0x03,   0x4C, 0xCD, 0x6C,         // FCFG_1 inv_set_gyro_calibration
    0x03,   0xAB,   0x03,   0x36, 0x56, 0x76,         // FCFG_3 inv_set_gyro_calibration
    0x00,   0x68,   0x04,   0x02, 0xCB, 0x47, 0xA2,   // D_0_104 inv_set_gyro_calibration
    0x02,   0x18,   0x04,   0x00, 0x05, 0x8B, 0xC1,   // D_0_24 inv_set_gyro_calibration
    0x01,   0x0C,   0x04,   0x00, 0x00, 0x00, 0x00,   // D_1_152 inv_set_accel_calibration
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_accel_calibration
    0x03,   0x89,   0x03,   0x26, 0x46, 0x66,         // FCFG_7 inv_set_accel_calibration
    0x00,   0x6C,   0x02,   0x20, 0x00,               // D_0_108 inv_set_accel_calibration
    0x02,   0x40,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_00 inv_set_compass_calibration
    0x02,   0x44,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_01
    0x02,   0x48,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_02
    0x02,   0x4C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_10
    0x02,   0x50,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_11
    0x02,   0x54,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_12
    0x02,   0x58,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_20
    0x02,   0x5C,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_21
    0x02,   0xBC,   0x04,   0x00, 0x00, 0x00, 0x00,   // CPASS_MTX_22
    0x01,   0xEC,   0x04,   0x00, 0x00, 0x40, 0x00,   // D_1_236 inv_apply_endian_accel
    0x03,   0x7F,   0x06,   0x0C, 0xC9, 0x2C, 0x97, 0x97, 0x97, // FCFG_2 inv_set_mpu_sensors
    0x04,   0x02,   0x03,   0x0D, 0x35, 0x5D,         // CFG_MOTION_BIAS inv_turn_on_bias_from_no_motion
    0x04,   0x09,   0x04,   0x87, 0x2D, 0x35, 0x3D,   // FCFG_5 inv_set_bias_update
    0x00,   0xA3,   0x01,   0x00,                     // D_0_163 inv_set_dead_zone
    // SPECIAL 0x01 = enable interrupts
    0x00,   0x00,   0x00,   0x01, // SET INT_ENABLE at i=22, SPECIAL INSTRUCTION
    0x07,   0x86,   0x01,   0xFE,                     // CFG_6 inv_set_fifo_interupt
    0x07,   0x41,   0x05,   0xF1, 0x20, 0x28, 0x30, 0x38, // CFG_8 inv_send_quaternion
    0x07,   0x7E,   0x01,   0x30,                     // CFG_16 inv_set_footer
    0x07,   0x46,   0x01,   0x9A,                     // CFG_GYRO_SOURCE inv_send_gyro
    0x07,   0x47,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_9 inv_send_gyro -> inv_construct3_fifo
    0x07,   0x6C,   0x04,   0xF1, 0x28, 0x30, 0x38,   // CFG_12 inv_send_accel -> inv_construct3_fifo
    0x02,   0x16,   0x02,   0x00, 0x01                // D_0_22 inv_set_fifo_rate

    // This very last 0x01 WAS a 0x09, which drops the FIFO rate down to 20 Hz. 0x07 is 25 Hz,
    // 0x01 is 100Hz. Going faster than 100Hz (0x00=200Hz) tends to result in very noisy data.
    // DMP output frequency is calculated easily using this equation: (200Hz / (1 + value))

    // It is important to make sure the host processor can keep up with reading and processing
    // the FIFO output at the desired rate. Handling FIFO overflow cleanly is also a good idea.
};

const uint8_t dmp_updates[MPU_DMP_UPDATES_SIZE] PROGMEM = {
    0x01,   0xB2,   0x02,   0xFF, 0xFF,
    0x01,   0x90,   0x04,   0x09, 0x23, 0xA1, 0x35,
    0x01,   0x6A,   0x02,   0x06, 0x00,
    0x01,   0x60,   0x08,   0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00,   0x60,   0x04,   0x40, 0x00, 0x00, 0x00,
    0x01,   0x62,   0x02,   0x00, 0x00,
    0x00,   0x60,   0x04,   0x00, 0x40, 0x00, 0x00
};

volatile uint8_t interrupt_accel_flag = 0;

uint8_t* dmp_packet_buffer;

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void configure_accelgyro()
{
    _delay_ms(100);
    set_sleep_mode(0); // wake up device
    
    _delay_ms(10);
    
    set_clk_selection(PLL_X_GYRO); // set clock source to PLL X gyro axis
    set_dlpf(DLPF_3); // set digital low pass filter mode to 42Hz
    // sample rate, try 1khz ? seems like accelerometer output rate is 1khz
    // so why shouting a lot of bit...
    // sample rate = gyro output rate / (1+SMPLRT_DIV)
    set_sample_rate(0x09);
    set_accel_range(ACCEL_G2); // config accelerometer range to +- 4g
    set_gyro_range(GYRO_DEG_250); // 4.1 config gyro range to +-250deg/sec
    
    interrupt_enable(DATA_READY|FIFO_OFLOW);
    configure_interrupt();
    
    set_fifo_enabled(1); // enable FIFO operations
    set_fifo_enable_for(ACCEL_FIFO_EN|GYRO_Z_FIFO_EN|GYRO_Y_FIFO_EN|GYRO_X_FIFO_EN); // enable FIFO for accelerometer and gyro 
}

/************************************************************************/
/* handle interrupts thrown by accel                                    */
/************************************************************************/
ISR(INT2_vect)
{
    interrupt_accel_flag = 1;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void clear_interrupt_accel_flag()
{
    interrupt_accel_flag = 0;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
uint8_t test_connection()
{
    int8_t ret = 1;
    // read from who am i 
    uint8_t data;
    uint8_t who_ami = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_WHO_AMI, &data);
    
    //read bits 6:1
    if(data > 0)
        who_ami = read_bits_from_byte(1, 6, data);

    if(who_ami == WHO_AM_I)
    {
        ret = 0;
    }
    else
    {
        printf("found who am i : 0x%X\n", who_ami);
    }
    return ret;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
uint8_t initialize_accelgyro()
{
    printf("MPU6050 initialize...\n");

    uint8_t ret = 1;
    
    if(test_connection() == 0)
    {
        reset_device();
        // configuration phase
        printf("MPU6050 config...\n");
        configure_accelgyro();
        printf("...Configuration done and successful\n");
    }
    else
    {
        printf("connection error\n");
    }
    return ret;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_sample_rate(uint8_t value)
{
    i2c_write_byte(MPU_ADDRESS, MPU_SMPRT_DIV, &value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_sleep_mode(uint8_t enable)
{
    uint8_t register_value = 0;
    // sleep mode bit is 6th in pwr mgmt 1 register
    // read the register value
    i2c_read_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);
    
    if(enable == 1)
        register_value |= (1 << 6);
    else
        register_value &= ~(1 << 6);
    
    i2c_write_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_clk_selection(enum CLK_SEL clk_value)
{
    uint8_t register_value = 0;
    // clock selection bits are the first 3rd bits in pwr mgmt 1 register
    // read the register value
    i2c_read_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);

    register_value |= clk_value;
    i2c_write_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_dlpf(enum DLPF_CFG dlpf_value)
{
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_CONFIG, &register_value);

    register_value |= dlpf_value;
    i2c_write_byte(MPU_ADDRESS, MPU_CONFIG, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_fsync_config(uint8_t value)
{
    uint8_t register_value;
    i2c_read_byte(MPU_ADDRESS, MPU_CONFIG, &register_value);
    
    register_value |= (value << 3);
    
    i2c_write_byte(MPU_ADDRESS, MPU_CONFIG, &value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_accel_range(enum ACCEL_RANGE range_value)
{
    // gyro range is set by bit 3 and 4 of GYRO_CONFIG register
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_ACCEL_CONFIG, &register_value);

    // reset bits
    register_value &= ~(1 << 3);
    register_value &= ~(1 << 4);
    register_value |= (range_value << 3);    

    i2c_write_byte(MPU_ADDRESS, MPU_ACCEL_CONFIG, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_gyro_range(enum GYRO_RANGE range_value)
{
    // gyro range is set by bit 3 and 4 of GYRO_CONFIG register
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_GYRO_CONFIG, &register_value);

    // reset bits
    register_value &= ~(1 << 3);
    register_value &= ~(1 << 4);
    // and set according to passed value
    register_value |= (range_value << 3);
    i2c_write_byte(MPU_ADDRESS, MPU_GYRO_CONFIG, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_fifo_enable_for(enum FIFO_ENABLE fifo_value)
{
    // here we systematically rewrite the full register
    uint8_t register_value = fifo_value;
    i2c_write_byte(MPU_ADDRESS, MPU_FIFO_EN, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_fifo_enabled(uint8_t enable)
{
    // toggle Bit 6 of User control register
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
    
    if(enable == 1)
        register_value |= (1 << 6);
    else
        register_value &= ~(1 << 6);
    
    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void reset_fifo()
{
    // toggle Bit 2 of User control register
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);

    register_value |= (1 << 2); // set FIFO_RESET bit to 1
    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void reset_device()
{
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);
    
    register_value |= (1 << 7); // set reset bit to one
    i2c_write_byte(MPU_ADDRESS, MPU_PWR_MGT_1, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void interrupt_enable(enum INT_EN_SRC enable_from)
{
    uint8_t register_value = 0;
    register_value = enable_from;   
    i2c_write_byte(MPU_ADDRESS, MPU_INT_EN, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_otp_bank_flag(uint8_t enable)
{
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_XG_OFFS_TC, &register_value);
    
    if(enable == 1)
        register_value |= 1 ; // set 0 bit to 1
    else
        register_value &= ~(1 << 0); // clear 0 bit

    i2c_write_byte(MPU_ADDRESS, MPU_XG_OFFS_TC, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
uint16_t get_fifo_count()
{
    uint8_t register_value[2];
    i2c_read_bytes(MPU_ADDRESS, MPU_FIFO_CNT_H, 2, register_value);
    return (((uint16_t)register_value[0]) << 8) | register_value[1];
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void get_fifo_bytes(uint8_t* bytes, uint8_t length)
{
    i2c_read_bytes(MPU_ADDRESS, MPU_FIFO_DATA, length, bytes);
}

/************************************************************************/
/* @details set interrupt on low level for working with atmega2560 int2 */
/************************************************************************/
void configure_interrupt()
{
    uint8_t register_value;

    register_value = 0x80; // interrupt on low level, push pull
    i2c_write_byte(MPU_ADDRESS, MPU_INT_PIN_CFG, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void get_int_status(uint8_t* data)
{
    i2c_read_byte(MPU_ADDRESS, MPU_INT_STATUS, data);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_master_mode_enabled(uint8_t enable)
{
    uint8_t register_value;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
    
    if(enable == 1)
        register_value |= (1 << 5);
    else
        register_value &= ~(1 << 5);
        
    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void reset_master_mode()
{
    uint8_t register_value;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
    
    register_value |= (1 << 1);
    
    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_dmp_enabled(uint8_t enable)
{
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
    
    if(enable == 1)
        register_value |= (1 << 7); // DMP is enabled by bit 7
    else
        register_value &= ~(1 << 7);

    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void reset_dmp()
{
    uint8_t register_value = 0;
    i2c_read_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);

    register_value |= (1 << 3); // bit 3 is DMP reset (you wont see that in register map :()   
    i2c_write_byte(MPU_ADDRESS, MPU_USER_CTRL, &register_value);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_memory_bank(uint8_t bank, uint8_t prefetch_enabled, uint8_t in_user_bank)
{
    bank &= 0x1F;
    if (in_user_bank == 1)
        bank |= 0x20;
        
    if (prefetch_enabled == 1)
    bank |= 0x40;
    
    i2c_write_byte(MPU_ADDRESS, MPU_BANK_SEL, &bank);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void set_memory_start_address(uint8_t address)
{
    i2c_write_byte(MPU_ADDRESS, MPU_MEM_START_ADDR, &address);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void read_memory_block(uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address)
{
    set_memory_bank(bank, 0, 0);
    set_memory_start_address(address);
    uint8_t chunk_size;
    for (uint16_t i = 0; i < data_size;)
    {
        // determine correct chunk size according to bank position and data size
        chunk_size = MPU_DMP_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunk_size > data_size)
            chunk_size = data_size - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunk_size > 256 - address)
            chunk_size = 256 - address;

        // read the chunk of data as specified
        i2c_read_bytes(MPU_ADDRESS, MPU_MEM_R_W, chunk_size, data + i);

        // increase byte index by [chunkSize]
        i += chunk_size;

        // uint8_t automatically wraps to 0 at 256
        address += chunk_size;

        // if we aren't done, update bank (if necessary) and address
        if (i < data_size)
        {
            if (address == 0) bank++;
            set_memory_bank(bank, 0, 0);
            set_memory_start_address(address);
        }
    }
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
uint8_t write_memory_block(const uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address, uint8_t verify, uint8_t use_prog_mem)
{
    set_memory_bank(bank, 0, 0);
    set_memory_start_address(address);
    uint8_t chunk_size;
    uint8_t* verify_buffer = 0;
    uint8_t* prog_buffer = 0;
    uint16_t i;
    uint8_t j;
    
    if (verify) 
        verify_buffer = (uint8_t*) malloc(MPU_DMP_CHUNK_SIZE);
        
    if (use_prog_mem)
        prog_buffer = (uint8_t*) malloc(MPU_DMP_CHUNK_SIZE);
        
    for (i = 0; i < data_size;)
    {
        // determine correct chunk size according to bank position and data size
        chunk_size = MPU_DMP_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunk_size > data_size) chunk_size = data_size - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunk_size > 256 - address) chunk_size = 256 - address;

        if (use_prog_mem == 1)
        {
            // write the chunk of data as specified
            for (j = 0; j < chunk_size; j++)
                prog_buffer[j] = pgm_read_byte(data + i + j);
        }
        else
        {
            // write the chunk of data as specified
            prog_buffer = (uint8_t *)data + i;
        }

        i2c_write_bytes(MPU_ADDRESS, MPU_MEM_R_W, chunk_size, prog_buffer);

        // verify data if needed
        if (verify && verify_buffer)
        {
            set_memory_bank(bank, 0, 0);
            set_memory_start_address(address);
            i2c_read_bytes(MPU_ADDRESS, MPU_MEM_R_W, chunk_size, verify_buffer);
            
            if (memcmp(prog_buffer, verify_buffer, chunk_size) != 0)
            {
                free(verify_buffer);
                if (use_prog_mem)
                    free(prog_buffer);
                    
                return 0; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunk_size;

        // uint8_t automatically wraps to 0 at 256
        address += chunk_size;

        // if we aren't done, update bank (if necessary) and address
        if (i < data_size)
        {
            if (address == 0)
                bank++;
            set_memory_bank(bank, 0, 0);
            set_memory_start_address(address);
        }
    }
    if (verify)
        free(verify_buffer);
        
    if (use_prog_mem)
        free(prog_buffer);
    return 1;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
uint8_t write_dmp_configuration_set(const uint8_t *data, uint16_t data_size, uint8_t use_prog_mem)
{
    uint8_t* prog_buffer = 0;
    uint8_t success, special;
    uint16_t i, j;
    if (use_prog_mem)
    {
        prog_buffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < data_size;)
    {
        if (use_prog_mem)
        {
            bank   = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        }
        else
        {
            bank   = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0)
        {
            // regular block of data to write
            if (use_prog_mem)
            {
                if (sizeof(prog_buffer) < length)
                    prog_buffer = (uint8_t*) realloc(prog_buffer, length);
                
                for (j = 0; j < length; j++)
                    prog_buffer[j] = pgm_read_byte(data + i + j);
            }
            else
            {
                prog_buffer = (uint8_t*)data + i;
            }
            success = write_memory_block(prog_buffer, length, bank, offset, 1, 0);
            i += length;
        }
        else
        {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (use_prog_mem)
            {
                special = pgm_read_byte(data + i++);
            }
            else
            {
                special = data[i++];
            }
            
            if (special == 0x01)
            {
                // enable DMP-related interrupts

                //mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled
                //mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled
                //mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled
                uint8_t single_op = 0x32;
                i2c_write_byte(MPU_ADDRESS, MPU_INT_EN, &single_op);  // single operation
                success = 1;
            }
            else
            {
                // unknown special command
                success = 0;
            }
        }

        if (!success)
        {
            if (use_prog_mem)
                free(prog_buffer);
                
            return 0; // uh oh
        }
    }
    
    if (use_prog_mem)
        free(prog_buffer);
        
    return 1;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void configure_dmp()
{
    reset_device(); // don't ask me why, Rowberg and Gironi do this too
    _delay_ms(30);
    
    set_sleep_mode(0); // wake up device
    
    uint8_t a_buffer = 0;
    set_memory_bank(0x10, 1, 1);
    set_memory_start_address(0x06);
    i2c_read_byte(MPU_ADDRESS, MPU_MEM_R_W, &a_buffer); // read hardware revision
    printf("Hardware revision %X\n", a_buffer);
    set_memory_bank(0, 0, 0);
    
    // check otp bank valid
    i2c_read_byte(MPU_ADDRESS, MPU_XG_OFFS_TC, &a_buffer);
    printf("OTP bank valid : %d\n", a_buffer & 1);
    
    // set weird slave stuffs
    a_buffer = 0x7F;
    i2c_write_byte(MPU_ADDRESS, MPU_I2C_SLV0, &a_buffer);
    // disable i2c master mode
    set_master_mode_enabled(0);
    a_buffer = 0x68;
    i2c_write_byte(MPU_ADDRESS, MPU_I2C_SLV0, &a_buffer);
    // reset i2c master mode
    reset_master_mode();
    _delay_ms(20);
    
    if( write_memory_block(dmp_memory, MPU_DMP_CODE_SIZE, 0, 0, 1, 1) == 1)
    {
        if(write_dmp_configuration_set(dmp_config, MPU_DMP_CONFIG_SIZE, 1) == 1)
        {
            set_clk_selection(PLL_Z_GYRO); // set clock selection
            
            //configure_interrupt(); // set interrupts active low
            
            interrupt_enable(DMP|FIFO_OFLOW);// enable DMP
            
            set_sample_rate(4); //set sample rate to 200Hz
            
            set_fsync_config(0x01); // set external fsync pin to TEMP_OUT_L.. why ?
            
            set_dlpf(DLPF_3); // set DLPF to 42Hz for Gyro, 44 for accelerometer
            
            set_gyro_range(GYRO_DEG_2000); // set gyro sensitivity to +- 2000 deg/sec

            // set DMP configuration bytes (function unknown)
            a_buffer = 0x03; 
            i2c_write_byte(MPU_ADDRESS, MPU_DMP_CFG_1, &a_buffer);
            a_buffer = 0x00;
            i2c_write_byte(MPU_ADDRESS, MPU_DMP_CFG_2, &a_buffer);
            
            set_otp_bank_flag(0); // clear OTP bank flag (??)
            
            // @todo set offset here if needed
            
            uint8_t dmp_update[16], j;
            uint16_t pos = 0;
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            if( write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 1/6");
                return;
            }

            //writing final memory update 2/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            if(write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 2/6");
                return;
            }
            
            reset_fifo(); //reset FIFO

            //reading FIFO count
            uint16_t fifo_count = get_fifo_count(); //HERE
            uint8_t fifo_buffer[128];
            
            get_fifo_bytes(fifo_buffer, fifo_count); //current FIFO count
            
            a_buffer = 2;
            i2c_write_byte(MPU_ADDRESS, MPU_MOT_THR, &a_buffer); //setting motion detection threshold to 2
            
            a_buffer = 156;
            i2c_write_byte(MPU_ADDRESS, MPU_ZRMOT_THR, &a_buffer); //setting zero-motion detection threshold to 156

            a_buffer = 80;
            i2c_write_byte(MPU_ADDRESS, MPU_MOT_DUR, &a_buffer); //setting motion detection duration to 80

            a_buffer = 0;
            i2c_write_byte(MPU_ADDRESS, MPU_ZRMOT_DUR, &a_buffer); //setting zero-motion detection duration to 0

            reset_fifo(); //reset FIFO

            set_fifo_enabled(1); //enabling FIFO

            //enabling DMP
            set_dmp_enabled(1);

            reset_dmp(); //resetting DMP
            
            _delay_ms(50);
            //waiting for FIFO count > 2
            while ((fifo_count = get_fifo_count()) < 3) _delay_ms(30);
            
            //writing final memory update 3/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            if(write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 3/6");
                return;
            }

            //writing final memory update 4/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            if(write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 4/6");
                return;
            }

            //writing final memory update 5/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            if(write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 5/6");
                return;
            }

            //reading FIFO data
            i2c_read_bytes(MPU_ADDRESS, MPU_FIFO_DATA, fifo_count, fifo_buffer);

            //reading final memory update 6/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
                
            read_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1]);
            
            _delay_ms(50);
            
            //waiting for FIFO count >= 2
            while ((fifo_count = get_fifo_count()) < 3) _delay_ms(30);

            //reading FIFO data
            get_fifo_bytes(fifo_buffer, fifo_count);
            
            get_int_status(&a_buffer);
            
            printf("int status : 0x%X\n", a_buffer);
            
            //writing final memory update 7/7 (function unknown)
            for (j = 0; j < 4 || j < dmp_update[2] + 3; j++, pos++)
                dmp_update[j] = pgm_read_byte(&dmp_updates[pos]);
         
            if(write_memory_block(dmp_update + 3, dmp_update[2], dmp_update[0], dmp_update[1], 1, 0) == 0)
            {
                printf("error while writing memory block 6/6");
                return;
            }
            
            set_dmp_enabled(0); //disabling DMP (you turn it on later)
            reset_fifo(); //resetting FIFO and clearing INT status one last time
            get_int_status(&a_buffer); // each call clear int flags
        }
        else
        {
            printf("DMP configuration write error, aborting \n");
        }
    }
    else
    {
        printf("DMP memory write error, aborting \n");
    }
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void get_quaternion_values_from_fifo(int16_t *data, const uint8_t* packet)
{
    if (packet == 0) packet = dmp_packet_buffer;
    data[0] = ((packet[0] << 8) | packet[1]);
    data[1] = ((packet[4] << 8) | packet[5]);
    data[2] = ((packet[8] << 8) | packet[9]);
    data[3] = ((packet[12] << 8) | packet[13]);
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void get_quaternion(Quaternion* quaternion , const uint8_t* packet)
{
    int16_t data[4];
    get_quaternion_values_from_fifo(data, packet);
    quaternion->w = (float)data[0] / 16384.0f;
    quaternion->x = (float)data[1] / 16384.0f;
    quaternion->y = (float)data[2] / 16384.0f;
    quaternion->z = (float)data[3] / 16384.0f;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void dmp_get_gravity(VectorFloat* vectorfloat, Quaternion* quaternion)
{
    vectorfloat->x = 2 * (quaternion->x*quaternion->z - quaternion->w*quaternion->y);
    vectorfloat->y = 2 * (quaternion->w*quaternion->x + quaternion->y*quaternion->z);
    vectorfloat->z = quaternion->w*quaternion->w - quaternion->x*quaternion->x - quaternion->y*quaternion->y + quaternion->z*quaternion->z;
}

/************************************************************************/
/* @details                                                             */
/************************************************************************/
void dmp_get_ypr(float* data, Quaternion* quaternion, VectorFloat* gravity)
{
    // yaw: (about Z axis)
    data[0] = atan2(2*quaternion->x*quaternion->y - 2*quaternion->w*quaternion->z, 2*quaternion->w*quaternion->w + 2*quaternion->x*quaternion->x - 1);
    // pitch: (nose up/down, about Y axis)
    data[1] = atan(gravity->x / sqrt(gravity->y*gravity->y + gravity->z*gravity->z));
    // roll: (tilt left/right, about X axis)
    data[2] = atan(gravity->y / sqrt(gravity->x*gravity->x + gravity->z*gravity->z));
}