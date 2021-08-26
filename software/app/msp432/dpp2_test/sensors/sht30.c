/*
 * Copyright (c) 2019, Swiss Federal Institute of Technology (ETH Zurich).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author:  Andreas Biri
 */

/*
 * Sensirion SHT30 driver
 */


/*
 * I N C L U D E S
 */

#include "sht30.h"

#include "config.h"
#include "serial.h"
#include "msp432_init.h"


/*
 * T Y P E D E F S   &   G L O B A L S
 */



/*
 * F U N C T I O N S
 */

uint8_t sht30_init(void) {
    // I2C already initialized
    return sht30_reset();
}

uint8_t sht30_reset(void) {

    const uint16_t cmd = (SHT30_CMD_RESET_SOFT_MSB << 8) | SHT30_CMD_RESET_SOFT_LSB;

    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 reset");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint8_t sht30_reset_general(void) {

    const uint8_t cmd = SHT30_CMD_RESET_GENERAL;

    if (i2c_write_cmd8(I2C_MODULE, SHT30_CMD_RESET_GENERAL_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 reset using general reset");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint8_t sht30_measurement_single_shot(repeatability_mode_t repeatability, uint8_t clock_stretching_enabled) {

    uint8_t result[6];
    uint16_t cmd = 0;

    if (clock_stretching_enabled) {
        cmd = (SHT30_CMD_MEAS_SINGLE_CLK_STR_EN << 8);

        switch (repeatability) {
            case HIGH: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_H;
                break;
            }
            case MEDIUM: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_M;
                break;
            }
            case LOW: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_EN_L;
                break;
            }
            default:
                DBG_PRINT("WARNING: SHT30 - invalid input");
        }
    }
    else {
        cmd = (SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS << 8);

        switch (repeatability) {
            case HIGH: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_H;
                break;
            }
            case MEDIUM: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_M;
                break;
            }
            case LOW: {
                cmd |= SHT30_CMD_MEAS_SINGLE_CLK_STR_DIS_L;
                break;
            }
            default:
                DBG_PRINT("WARNING: SHT30 - invalid input");
        }
    }


    if (i2c_read(I2C_MODULE, SHT30_ADDR, cmd, 6, result))
    {
        /* check crc */
        if (crc8(result, 2, 0xff) != result[2] || crc8(&result[3], 2, 0xff) != result[5])
        {
            DBG_PRINT("WARNING: SHT30 invalid data (CRC)");
            return 1;
        } else {
            float t_conv = TEMP_C(  ((uint16_t)result[0] << 8) + result[1]);
            float h_conv = HUMIDITY(((uint16_t)result[3] << 8) + result[4]);
            DBG_PRINT("INFO: SHT30 temperature: %.1f°C, humidity: %.1f%%", t_conv, h_conv);
            return 0;
        }
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_read failed");
        return 1;
    }
}

uint8_t sht30_measurement_start(repeatability_mode_t repeatability, measurements_per_second_t mps) {

    uint16_t cmd = 0;

    switch (mps) {

        case MSP_0_5: {
            cmd = (SHT30_CMD_MEAS_PERIODIC_MSP_0_5 << 8);

            switch (repeatability) {
                case HIGH: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_0_5_H;
                    break;
                }
                case MEDIUM: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_0_5_M;
                    break;
                }
                case LOW: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_0_5_L;
                    break;
                }
                default:
                    DBG_PRINT("WARNING: SHT30 - invalid input");
            }
        }
        case MSP_1: {
            cmd = (SHT30_CMD_MEAS_PERIODIC_MSP_1 << 8);

            switch (repeatability) {
                case HIGH: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_1_H;
                    break;
                }
                case MEDIUM: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_1_M;
                    break;
                }
                case LOW: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_1_L;
                    break;
                }
                default:
                    DBG_PRINT("WARNING: SHT30 - invalid input");
            }
        }
        case MSP_2: {
            cmd = (SHT30_CMD_MEAS_PERIODIC_MSP_2 << 8);

            switch (repeatability) {
                case HIGH: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_2_H;
                    break;
                }
                case MEDIUM: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_2_M;
                    break;
                }
                case LOW: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_2_L;
                    break;
                }
                default:
                    DBG_PRINT("WARNING: SHT30 - invalid input");
            }
        }
        case MSP_4: {
            cmd = (SHT30_CMD_MEAS_PERIODIC_MSP_4 << 8);

            switch (repeatability) {
                case HIGH: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_4_H;
                    break;
                }
                case MEDIUM: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_4_M;
                    break;
                }
                case LOW: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_4_L;
                    break;
                }
                default:
                    DBG_PRINT("WARNING: SHT30 - invalid input");
            }
        }
        case MSP_10: {
            cmd = (SHT30_CMD_MEAS_PERIODIC_MSP_10 << 8);

            switch (repeatability) {
                case HIGH: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_10_H;
                    break;
                }
                case MEDIUM: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_10_M;
                    break;
                }
                case LOW: {
                    cmd |= SHT30_CMD_MEAS_PERIODIC_MSP_10_L;
                    break;
                }
                default:
                    DBG_PRINT("WARNING: SHT30 - invalid input");
            }
        }
        default:
            DBG_PRINT("WARNING: SHT30 - invalid input");
    }


    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 started measurement");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint16_t sht30_measurement_read(void) {

    uint8_t result[6];
    const uint16_t cmd = (SHT30_CMD_MEAS_PERIODIC_READ_MSB << 8) | SHT30_CMD_MEAS_PERIODIC_READ_LSB;

    if (i2c_read(I2C_MODULE, SHT30_ADDR, cmd, 6, result))
    {
        /* check crc */
        if (crc8(result, 2, 0xff) != result[2] || crc8(&result[3], 2, 0xff) != result[5])
        {
            DBG_PRINT("WARNING: SHT30 invalid data (CRC)");
            return 1;
        } else {
            float t_conv = TEMP_C(  ((uint16_t)result[0] << 8) + result[1]);
            float h_conv = HUMIDITY(((uint16_t)result[3] << 8) + result[4]);
            DBG_PRINT("INFO: SHT30 temperature: %.1f°C, humidity: %.1f%%", t_conv, h_conv);
            return 0;
        }
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_read failed");
        return 1;
    }
}

uint8_t sht30_measurement_accelerated(void) {

    // Same as measurement with 4 Hz
    //sht_measurement_start(MSP_4, HIGH);

    const uint16_t cmd = (SHT30_CMD_MEAS_PERIODIC_ART_MSB << 8) | SHT30_CMD_MEAS_PERIODIC_ART_LSB;

    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 started measurement in ART mode");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint8_t sht30_measurement_stop(void) {

    const uint16_t cmd = (SHT30_CMD_MEAS_PERIODIC_STOP_MSB << 8) | SHT30_CMD_MEAS_PERIODIC_STOP_LSB;

    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 stopped measurement");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint8_t sht30_heater_set_status(uint8_t enable) {

    enable = (enable > 0);
    uint16_t cmd = (SHT30_CMD_HEATER_MSB << 8);

    if (enable) {
        cmd |= SHT30_CMD_HEATER_EN_LSB;
    }
    else {
        cmd |= SHT30_CMD_HEATER_DIS_LSB;
    }

    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 set heater to %i", enable);
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}

uint16_t sht30_get_status(void) {

    uint8_t result[3];
    const uint16_t cmd = (SHT30_CMD_STATUS_READ_MSB << 8) | SHT30_CMD_STATUS_READ_LSB;

    if (i2c_read(I2C_MODULE, SHT30_ADDR, cmd, 3, result))
    {
        /* check crc */
        if (crc8(result, 2, 0xff) != result[2])
        {
            DBG_PRINT("WARNING: SHT30 invalid data (CRC)");
            return 1;
        } else {
            DBG_PRINT("INFO: SHT30 status read");

            uint16_t status = (result[0] << 8) | result[1];
            if (status & SHT30_STATUS_ALERT_PENDING)
                DBG_PRINT("INFO: SHT30 status - Alert pending");
            if (status & SHT30_STATUS_HEATER_EN)
                DBG_PRINT("INFO: SHT30 status - Heater on");
            if (status & SHT30_STATUS_RH_ALERT)
                DBG_PRINT("INFO: SHT30 status - RH tracking alert");
            if (status & SHT30_STATUS_T_ALERT)
                DBG_PRINT("INFO: SHT30 status - T tracking alert");
            if (status & SHT30_STATUS_RESET_DETECTED)
                DBG_PRINT("INFO: SHT30 status - System reset detected");
            if (status & SHT30_STATUS_CMD_UNSUCCESSFUL)
                DBG_PRINT("INFO: SHT30 status - Last command not processed");
            if (status & SHT30_STATUS_CRC_FAILED)
                DBG_PRINT("INFO: SHT30 status - Checksum failed");

            return 0;
        }
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_read failed");
        return 1;
    }
}

uint8_t sht30_clear_status(void) {

    const uint16_t cmd = (SHT30_CMD_STATUS_CLEAR_MSB << 8) | SHT30_CMD_STATUS_CLEAR_LSB;

    if (i2c_write(I2C_MODULE, SHT30_ADDR, cmd))
    {
        DBG_PRINT("INFO: SHT30 status cleared");
        return 0;
    } else
    {
        DBG_PRINT_CONST("ERROR: i2c_write failed");
        return 1;
    }
}
