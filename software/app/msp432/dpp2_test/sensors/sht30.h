/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich), PIN
 * All rights reserved, PIN
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1, PIN Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer, PIN
 * 2, PIN Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution, PIN
 * 3, PIN Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission, PIN
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED, PIN  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE, PIN
 *
 * Author:  Andreas Biri
 */

/*
 * Sensirion SHT30 temperature and humidity sensor
 */

#ifndef __SHT30_H
#define __SHT30_H

#include <stdint.h>

/*
 * D E F I N I T I O N S
 */

#ifndef SHT30_ADDR
// Address A - ADDR to VSS
#define SHT30_ADDR 0x44
// Address B - ADDR to VDD
//#define SHT30_ADDR 0x45
#endif /* SHT30_ADDR */

// Commands

// Single-shot
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_EN    0x2C
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_H  0x06
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_M  0x0D
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_L  0x10

#define SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS   0x24
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_H 0x00
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_M 0x0B
#define SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_L 0x16

// Periodic
#define SHT30_CMD_MEAS_PERIODIC_MSP_0_5     0x20
#define SHT30_CMD_MEAS_PERIODIC_MSP_0_5_H   0x32
#define SHT30_CMD_MEAS_PERIODIC_MSP_0_5_M   0x24
#define SHT30_CMD_MEAS_PERIODIC_MSP_0_5_L   0x2F

#define SHT30_CMD_MEAS_PERIODIC_MSP_1       0x21
#define SHT30_CMD_MEAS_PERIODIC_MSP_1_H     0x30
#define SHT30_CMD_MEAS_PERIODIC_MSP_1_M     0x26
#define SHT30_CMD_MEAS_PERIODIC_MSP_1_L     0x2D

#define SHT30_CMD_MEAS_PERIODIC_MSP_2       0x22
#define SHT30_CMD_MEAS_PERIODIC_MSP_2_H     0x36
#define SHT30_CMD_MEAS_PERIODIC_MSP_2_M     0x20
#define SHT30_CMD_MEAS_PERIODIC_MSP_2_L     0x2B

#define SHT30_CMD_MEAS_PERIODIC_MSP_4       0x23
#define SHT30_CMD_MEAS_PERIODIC_MSP_4_H     0x34
#define SHT30_CMD_MEAS_PERIODIC_MSP_4_M     0x22
#define SHT30_CMD_MEAS_PERIODIC_MSP_4_L     0x29

#define SHT30_CMD_MEAS_PERIODIC_MSP_10      0x27
#define SHT30_CMD_MEAS_PERIODIC_MSP_10_H    0x37
#define SHT30_CMD_MEAS_PERIODIC_MSP_10_M    0x21
#define SHT30_CMD_MEAS_PERIODIC_MSP_10_L    0x2A

#define SHT30_CMD_MEAS_PERIODIC_READ_MSB    0xE0
#define SHT30_CMD_MEAS_PERIODIC_READ_LSB    0x00

#define SHT30_CMD_MEAS_PERIODIC_ART_MSB     0x2B
#define SHT30_CMD_MEAS_PERIODIC_ART_LSB     0x32

#define SHT30_CMD_MEAS_PERIODIC_STOP_MSB    0x30
#define SHT30_CMD_MEAS_PERIODIC_STOP_LSB    0x93

// Reset
#define SHT30_CMD_RESET_SOFT_MSB            0x30
#define SHT30_CMD_RESET_SOFT_LSB            0xA2

#define SHT30_CMD_RESET_GENERAL_ADDR        0x00
#define SHT30_CMD_RESET_GENERAL             0x06

// Heater
#define SHT30_CMD_HEATER_MSB                0x30
#define SHT30_CMD_HEATER_EN_LSB             0x6D
#define SHT30_CMD_HEATER_DIS_LSB            0x66

// Status
#define SHT30_CMD_STATUS_READ_MSB           0xF3
#define SHT30_CMD_STATUS_READ_LSB           0x2D

#define SHT30_STATUS_ALERT_PENDING          (1 << 15)
#define SHT30_STATUS_HEATER_EN              (1 << 13)
#define SHT30_STATUS_RH_ALERT               (1 << 11)
#define SHT30_STATUS_T_ALERT                (1 << 10)
#define SHT30_STATUS_RESET_DETECTED         (1 <<  4)
#define SHT30_STATUS_CMD_UNSUCCESSFUL       (1 <<  1)
#define SHT30_STATUS_CRC_FAILED             (1 <<  0)

#define SHT30_CMD_STATUS_CLEAR_MSB          0x30
#define SHT30_CMD_STATUS_CLEAR_LSB          0x41

/*
 * M A C R O S
 */
#define HUMIDITY(hum_d) ((float)(((uint16_t) hum_d) * 100.0f / 65535.0f)        )
#define TEMP_C(temp_d)  ((float)(((uint16_t)temp_d) * 175.0f / 65535.0f) - 45.0f)
#define TEMP_F(temp_d)  ((float)(((uint16_t)temp_d) * 315.0f / 65535.0f) - 49.0f)

/*
 * T Y P E D E F S
 */
typedef enum {
    HIGH   = 0,
    MEDIUM = 1,
    LOW    = 2
} repeatability_mode_t;

typedef enum {
    MSP_0_5 = 0,
    MSP_1   = 1,
    MSP_2   = 2,
    MSP_4   = 3,
    MSP_10  = 4
} measurements_per_second_t;

/*
 * P R O T O T Y P E S
 */

uint8_t  sht30_init(void);
uint8_t  sht30_reset(void);
uint8_t  sht30_reset_general(void);
uint8_t  sht30_measurement_single_shot(repeatability_mode_t repeatability, uint8_t clock_stretching_enabled);
uint8_t  sht30_measurement_start(repeatability_mode_t repeatability, measurements_per_second_t mps);
uint16_t sht30_measurement_read(void);
uint8_t  sht30_measurement_accelerated(void);
uint8_t  sht30_measurement_stop(void);
uint8_t  sht30_heater_set_status(uint8_t enable);
uint16_t sht30_get_status(void);
uint8_t  sht30_clear_status(void);


#endif //__SHT30_H
