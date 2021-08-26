/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich), PIN
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
 * Author:  Reto Da Forno
 */

/*
 * DPP2 MSP432 config
 *
 * pin mappings and default clock/peripheral settings
 */

#ifndef __HAL_H__
#define __HAL_H__



/*
 * P I N   D E F I N I T I O N S
 */

/* when changing pin definitions, don't forget to adjust gpio_init() */

#define PIN_APP_BSL             PORT1, PIN0
#define PIN_APP_RXD             PORT1, PIN2
#define PIN_APP_TXD             PORT1, PIN3
#define PIN_APP_SCK             PORT1, PIN5
#define PIN_APP_MOSI            PORT1, PIN6
#define PIN_APP_MISO            PORT1, PIN7
#define PIN_APP_LED1            PORT2, PIN0
#define PIN_APP_LED2            PORT2, PIN1
#define PIN_APP_GPIO1           PORT2, PIN2
#define PIN_APP_GPIO2           PORT2, PIN3

#define PIN_APP_EXT0            PORT3, PIN0
#define PIN_APP_EXT1            PORT3, PIN1
#define PIN_APP_EXT2            PORT3, PIN2
#define PIN_APP_EXT3            PORT3, PIN3
#define PIN_APP_EXT4            PORT3, PIN4
#define PIN_APP_EXT5            PORT3, PIN5
#define PIN_APP_EXT6            PORT3, PIN6
#define PIN_APP_EXT7            PORT3, PIN7

#define PIN_COM_TREQ            PORT4, PIN2
#define PIN_COM_IND             PORT4, PIN3
#define PIN_APP_IND             PORT4, PIN4
#define PIN_APP_MODE            PORT4, PIN5
#define PIN_APP_REQ             PORT4, PIN6
#define PIN_APP_ACK             PORT4, PIN7

#define PIN_APP_EXT13           PORT5, PIN0
#define PIN_APP_EXT12           PORT5, PIN1
#define PIN_APP_EXT11           PORT5, PIN2
#define PIN_APP_EXT10           PORT5, PIN3
#define PIN_APP_EXT9            PORT5, PIN6
#define PIN_APP_EXT8            PORT5, PIN7

#define PIN_APP_VBAT            PORT5, PIN5

#define PIN_APP_SD0             PORT7, PIN0
#define PIN_APP_SD1             PORT7, PIN1
#define PIN_APP_SD2             PORT7, PIN2
#define PIN_APP_SD3             PORT7, PIN3

#define PIN_APP_SENSOR_SDA      PORT6, PIN6
#define PIN_APP_SENSOR_SCL      PORT6, PIN7

#define PIN_APP_COM_EN          PORT8, PIN0
#define PIN_APP_PERIPH_EN       PORT8, PIN1


/*
 * P I N   M A P P I N G
 */

/* when changing pin mappings, don't forget to adjust gpio_init() */

/* Debug UART */
#define DBG_UART_RXD            PIN_APP_RXD
#define DBG_UART_TXD            PIN_APP_TXD

/* Debug pins */
#define DBG_PIN1                PIN_APP_GPIO1
#define DBG_PIN2                PIN_APP_GPIO2

/* LEDs */
#define LED_STATUS              PIN_APP_LED1
#define LED_ERROR               PIN_APP_LED2

/* Power Switch */
#define PWRSW_COM               PIN_APP_COM_EN
#define PWRSW_PERIPH            PIN_APP_PERIPH_EN

/* I2C Sensors */
#define SENSOR_SDA              PORT6, PIN6
#define SENSOR_SCL              PORT6, PIN7

/* Voltage monitor */
#define VIN_MON                 PIN_APP_VBAT

/* SD Card */
#define SDCARD_SPI_CS           PIN_APP_SD3
#define SDCARD_SPI_MOSI         PIN_APP_SD2
#define SDCARD_SPI_MISO         PIN_APP_SD0
#define SDCARD_SPI_SCK          PIN_APP_SD1

/* BOLT */
#define BOLT_REQ                PIN_APP_REQ
#define BOLT_ACK                PIN_APP_ACK
#define BOLT_MODE               PIN_APP_MODE
#define BOLT_IND                PIN_APP_IND
#define BOLT_IND_OUT            PIN_COM_IND
#define BOLT_TREQ               PIN_COM_TREQ
#define BOLT_SPI_MOSI           PIN_APP_MOSI
#define BOLT_SPI_MISO           PIN_APP_MISO
#define BOLT_SPI_SCK            PIN_APP_SCK


/*
 * D E F A U L T   C O N F I G
 */

/* Clocks */
#define HFXTCLK_SPEED           48000000                // external high-frequency crystal oscillator clock speed [Hz]
#define LFXTCLK_SPEED           32768                   // external low-frequency crystal oscillator clock speed [Hz]
#define DCOCLK_SPEED            3000000                 // internal digitally-controlled oscillator
#define REFOCLK_SPEED           32768                   // 32 or 128 kHz
#define MCLK_SRC                CS_HFXTCLK_SELECT       // main/master clock source
#define MCLK_DIV                CS_CLOCK_DIVIDER_1      // main clock divider
#define MCLK_SPEED              HFXTCLK_SPEED           // main clock speed [Hz]
#define SMCLK_SRC               CS_HFXTCLK_SELECT       // sub-system master clock
#define SMCLK_DIV               CS_CLOCK_DIVIDER_8      //
#define SMCLK_SPEED             (HFXTCLK_SPEED / 8)     // 6 MHz
#define HSMCLK_SRC              CS_HFXTCLK_SELECT       // high-speed sub-system master clock
#define HSMCLK_DIV              CS_CLOCK_DIVIDER_4      //
#define HSMCLK_SPEED            (HFXTCLK_SPEED / 4)     // 12 MHz
#define ACLK_SRC                CS_LFXTCLK_SELECT       // auxiliary clock
#define ACLK_DIV                CS_CLOCK_DIVIDER_1      //
#define ACLK_SPEED              LFXTCLK_SPEED           // max, PIN 128 kHz
#define BCLK_SRC                CS_LFXTCLK_SELECT       // low-speed backup domain clock, LFXT or REFO
#define BCLK_DIV                CS_CLOCK_DIVIDER_1      //
#define BCLK_SPEED              LFXTCLK_SPEED           // max, PIN 32 kHz

/* SVS (supply voltage supervisor) */
#define SVS_ENABLE              0                       // enable SVSHM and SVSL
#define SVS_LOW_PERF            1                       // low performance mode

/* Debug UART */
#define UART_BAUDRATE           115200
#define UART_CLK_SRC            EUSCI_A_CTLW0_SSEL__SMCLK
#define UART_CLK_SPEED          SMCLK_SPEED
#define UART_MODULE             EUSCI_A0

/* SPI */
#define SPI_CLK_SRC             UCSSEL__SMCLK           // must be UCSSEL__ACLK or UCSSEL__SMCLK
#define SPI_CLK_SPEED           SMCLK_SPEED             // adjust this according to the SPI_CLK_SRC selection
#define SPI_FAST_READ           0
#define SPI_DUMMY_BYTE          0xff

/* I2C */
#define I2C_SCL_SPEED           100000
#define I2C_CLK_SRC             EUSCI_B_CTLW0_SSEL__SMCLK
#define I2C_CLK_SPEED           SMCLK_SPEED
#define I2C_MODULE              EUSCI_B3                // default module for I2C sensors
#define I2C_MODULE_ADDR         EUSCI_B3_BASE           // default module for I2C sensors

/* SD Card */
#define SDCARD_SPI_MODULE       EUSCI_A1_SPI
#define SDCARD_SPI_CPOL         0                       // active low
#define SDCARD_SPI_CPHA         0                       // data changed on first edge, captured on next (note: use UCBx module if phase = 1 is required! see erratasheet)
#define SDCARD_SPI_SPEED        400000                  // for compatibility reasons, initial clock should be 400kHz
#define SDCARD_SPI_SPEED_FAST   SMCLK_SPEED
#define SDCARD_BLOCK_SIZE       512

/* BOLT */
#define BOLT_USE_DMA            0
#define BOLT_SPI_SPEED          (SMCLK_SPEED / 2)
#define BOLT_SPI_MODULE         EUSCI_B0_SPI
#define BOLT_SPI_CPOL           0                       // default clock polarity (0 = inactive low)
#define BOLT_SPI_CPHA           0                       // default clock phase (1 = data changed on first edge, captured on next)

/* Systick timer */
#define SYSTICK_PERIOD          (MCLK_SPEED / 100)      // max, PIN value is 24-bit (~16M), this yiels a min. divider of 3 and a period of ~350ms (with a 48MHz MCLK); the divider should be an even number

/* Timer */
#define TIMER_A0_SRC            TASSEL__SMCLK
#define TIMER_A1_SRC            TASSEL__ACLK

/* PWM timer */
#define PWM_PERIOD              0xffff

/* 32-bit timer */
#define TIMER32_PRESCALER       TIMER32_PRESCALER_16    // select 1, 16 or 256
#define TIMER32_SPEED           (MCLK_SPEED / ((TIMER32_PRESCALER == TIMER32_PRESCALER_256) ? 256 : (TIMER32_PRESCALER == TIMER32_PRESCALER_16 ? 16 : 1)))  // number of ticks per second

/* Flash memory */
#define FLASH_MEMORY_START      0x00000000              // first valid word in the flash memory
#define FLASH_MEMORY_SIZE       (262144 / 2)            // total size of the flash memory

/* ADC */
#define ADC_NUM_CH              12                      //  24 channels for the 100-pin version, 12 channels for the 64-pin version

/* Sensors */
#define SHT3x_I2C_ADDR          0x44                    // I2C slave address
#define SHT3x_I2C_CMD           0x2C06                  // measurement command for single shot data acquisition mode (high repeatability, clock stretching enabled)
#define BME280_I2C_ADDR         0x77                    // I2C slave address = device ID
#define BME280_I2C_CMD          0xD0                    // just read the device ID (should be 0x60)


#endif /* __HAL_H__ */
