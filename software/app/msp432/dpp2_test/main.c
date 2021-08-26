/*
 * Copyright (c) 2017, Swiss Federal Institute of Technology (ETH Zurich).
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
 * Author:  Reto Da Forno
 */

/*
 * DPP2 Dev Board: Test/demo program for the MSP432 (rev.C)
 */

#include "main.h"

void test_bolt(void);
void test_bolt_read(void);
void test_sdcard(void);
void test_sht3x(void);
void test_bme280(void);
void test_lpm(void);

const RTC_C_Calendar curr_time = { 0, 0, 10, 0, 1, 7, 2017 };

volatile float  t_mcu  = 0;
volatile uint16_t v_in = 0;



/*
 * M A I N
 */

void main(void)
{
  /* --- init msp432 core --- */
  msp432_init();
  LED_ON(LED_STATUS);                     /* turn on status LED */

  /* --- init peripherals --- */
  uart_init();
  rtc_set_time(&curr_time);               /* set RTC time */
  timer32_init();                         /* enable 32-bit timer */
  //systick_enable();                     /* enable the SysTick interrupt */
#if PERIPH_ENABLE
  i2c_init(I2C_MODULE, I2C_SCL_SPEED);    /* i2c sensors */
 #ifdef DPP_REV1                          /* rev1 does not support multi channel readout */
  adc_init_tempsensor();                  /* internal temperature sensor only */
  //adc_init_simple(0);                   /* simple ADC channel readout only */
 #else /* DPP_REV1 */
  adc_init_multi_ch(0, ADC_CH_TEMPSENSOR);  /* read 2 ADC channels */
 #endif /* DPP_REV1 */
#endif /* PERIPH_ENABLE */
  PWM_TA0_CFG(LED_STATUS, 1);             /* PWM timer to regulate LED brightness */

  /* --- init done --- */
  DBG_PRINT("msp432 initialized!\r\n"
            "\t\tdevice ID: 0x%x (rev 0x%x)\r\n"
            "\t\tclock speed: %u MHz\r\n"
            "\t\tflash memory: %u of %u kB used, %u wait states\r\n"
            "\t\treset source: %s", *(uint32_t*)0x0020100C,
                                    *(uint32_t*)0x00201010,
                                    MCLK_SPEED / 1000000,
                                    flash_code_size() >> 10,
                                    FLASH_MEMORY_SIZE >> 10,
                                    FLASH_WAIT_STATES,
                                    rst_src());

  /* --- enable interrupts --- */
  INTERRUPT_ENABLE(INT_EUSCIA0);  /* UART RXD interrupt */
  INTERRUPT_ENABLE(INT_WDT_A);    /* WDT interrupt */
  INTERRUPT_ENABLE(INT_ADC14);    /* ADC interrupt */
  INTERRUPT_ENABLE(INT_TA0_0);      /* Timer A0 interrupt */
  __eint();

  /* --- Demo/test program starts --- */

  test_bolt();          /* try to write a message to BOLT */

#if PERIPH_ENABLE
  PWRSW_ON(PWRSW_PERIPH); /* enable the power for the peripherals */
  WAIT_MS(2);
  //test_bme280();  /* read sensor values from BME280 via i2c */
  test_sht3x();   /* read sensor values from SHT30 via i2c */
  test_sdcard();

  /* poll sensors */
  LED_ON(PIN_APP_LED2);
  DBG_PRINT("RTC %s", rtc_get_time(1));   /* print RTC value */
  ADC_TRIGGER_SC;                         /* read supply voltage and internal temperature sensor */
  WAIT_MS(10);
  DBG_PRINT("voltage monitor: %umV, int. temperature sensor: %.1f°C", v_in, t_mcu);
  test_bolt_read();                       /* read all available messages from BOLT */
  LED_OFF(PIN_APP_LED2);

#endif /* PERIPH_ENABLE */

  //test_lpm();   /* go to low-power mode */
  serial_console('\r');

  /* after wakeup from LPM, wait in a loop indefinitely */
  while (1)
  {
    LED_TOGGLE(LED_ERROR);
    WAIT_MS(200);
  }
}


void test_bolt(void)
{
  /* init BOLT */
  PIN_PULLDOWN_EN(BOLT_IND);  /* enable pulldown on IND line */
  if (!bolt_init())
  {
    DBG_PRINT_CONST("ERROR bolt_init failed");
    return;
  }
  uint16_t msg[64];
  msg[0] = 16;
  msg[1] = 0x00aa;
  msg[8] = crc16((uint8_t*)msg, 16, 0);
  /* write a message to BOLT */
  if (!bolt_write((uint8_t*)msg, 18))
  {
    DBG_PRINT_CONST("ERROR bolt_write failed");
  } else {
    DBG_PRINT_CONST("BOLT message written");
  }
}


void test_bolt_read(void)
{
  /* read from BOLT if data available */
  uint16_t rcvd_bytes = 1;
  while (BOLT_DATA_AVAILABLE && rcvd_bytes)
  {
    uint8_t  buffer[BOLT_MAX_MSG_LEN];
    rcvd_bytes = bolt_read(buffer);
    if (rcvd_bytes)
    {
      buffer[rcvd_bytes] = 0;
      DBG_PRINT("message received from BOLT: '%s'", (char*)buffer);
    }
  }
}


void test_sdcard(void)
{
#if SDCARD_USE_FATFS
  /* enable the SD card and write to a file (note: skip card detect, no detect pins on the PCB!) */
  FIL fil;
  FRESULT rc;
  SDCardLib sdCardLib;
  SDCardLib_init(&sdCardLib, &sdint_msp432p401);  /* mount the file system */

 #if SDCARD_FORMAT
  DBG_PRINT_CONST("formatting SD card...");
  rc = f_mkfs(0, 0, SDCARD_BLOCK_SIZE);    /* with default parameters */
  if (0 == rc)
  {
    DBG_PRINT_CONST("SD card formatted (FatFS)");
  } else
  {
    DBG_PRINT("ERROR f_mkfs failed with code 0x%x", rc);
  }
 #endif /* SDCARD_FORMAT */

  rc = f_open(&fil, "test.txt", FA_WRITE | FA_OPEN_ALWAYS);     /* try to open existing file */
  if (0 == rc)
  {
    //rc = f_write(&fil, "hello world", 11, 0);
    rc = f_lseek(&fil, f_size(&fil));
    f_puts("hello world! ", &fil);        // append some data
    DBG_PRINT_CONST("data written to SD card");
  } else
  {
    DBG_PRINT("ERROR f_open failed with code 0x%x (valid FAT32 SD card inserted?)", rc);
  }
  rc = f_close(&fil);
  SDCardLib_unInit(&sdCardLib);                   /* unmount the file system */
  sdcard_print_cid();

#else /* SDCARD_USE_FATFS */

  /* init the SD card */
  uint_fast8_t res = sdcard_init();
  DBG_PRINT("SD card init returned with code 0x%x", res);
  if (res == 0)
  {
    uint8_t test_data[SDCARD_BLOCK_SIZE];
 #if SDCARD_WRITE_BLOCK
    /* write 1 block of data, then read it back */
    memset(test_data, 0xa1, SDCARD_BLOCK_SIZE);
    res = sdcard_write_block(0x00000000, test_data, SDCARD_BLOCK_SIZE);
    DBG_PRINT("SD card write returned with code 0x%x", res);
 #endif /* SDCARD_WRITE_BLOCK */
    memset(test_data, 0, SDCARD_BLOCK_SIZE);
    res = sdcard_read_block(0x00000000, test_data);
    DBG_PRINT("SD card read returned with code 0x%x", res);
    /* check the data */
    uint32_t i;
    for (i = SDCARD_BLOCK_SIZE - 1; i > 0; i--)
    {
      if (test_data[i] != 0xa1)
      {
        DBG_PRINT_CONST("ERROR data read from SD card is invalid");
      }
    }
  }
#endif /* SDCARD_USE_FATFS */

  SPI_DISABLE(SDCARD_SPI_MODULE);
}


void test_sht3x(void)
{
  sht30_init();
  WAIT_MS(2);
  sht30_measurement_single_shot(HIGH, 1);
}


#ifndef INT8_C
  #define  INT8_C(value)  ((int_least8_t)(value))
  #define UINT8_C(value)  ((uint_least8_t)(value))
  #define  INT16_C(value) ((int_least16_t)(value))
  #define UINT16_C(value) ((uint_least16_t)(value))
  #define  INT32_C(value) ((int_least32_t)(value))
  #define UINT32_C(value) ((uint_least32_t)(value))
#endif

void test_bme280(void)
{
  struct bme280_dev  bme280_sensor;
  struct bme280_data bme280_sensor_data;
  int8_t result = BME280_OK;

  if (i2c_read_cmd8(I2C_MODULE, BME280_I2C_ADDR, BME280_I2C_CMD, 2, (uint8_t*)&result) && result == 0x60)
  {
    DBG_PRINT_CONST("BME280 sensor detected");
  } else
  {
    DBG_PRINT_CONST("no BME280 sensor detected!");
    return;
  }

  //sensor_init();
  I2C_ENABLE(I2C_MODULE);
  bme280_sensor.dev_id    = BME280_I2C_ADDR;
  bme280_sensor.intf      = BME280_I2C_INTF;
  bme280_sensor.read      = sensor_i2c_read;
  bme280_sensor.write     = sensor_i2c_write;
  bme280_sensor.delay_ms  = sensor_delay_ms;
  result = bme280_init(&bme280_sensor);
  if (result == BME280_OK)
  {
    bme280_sensor.settings.osr_h = BME280_OVERSAMPLING_1X;
    bme280_sensor.settings.osr_p = BME280_OVERSAMPLING_1X;
    bme280_sensor.settings.osr_t = BME280_OVERSAMPLING_1X;
    bme280_sensor.settings.filter = BME280_FILTER_COEFF_16;
    result = bme280_set_sensor_settings(BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL | BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &bme280_sensor);
    if (result != BME280_OK || BME280_OK != bme280_set_sensor_mode(BME280_FORCED_MODE, &bme280_sensor))
    {
      DBG_PRINT_CONST("ERROR failed to set sensor mode for BME280");
      return;
    }
    /* wait until the sensor goes back to sleep mode (alternatively: bme280_sensor.delay_ms(40)) */
    do
    {
      WAIT_MS(1);
      bme280_get_sensor_mode((uint8_t*)&result, &bme280_sensor);
    } while (result != 0);

    result = bme280_get_sensor_data(BME280_ALL, &bme280_sensor_data, &bme280_sensor);
    if (BME280_OK == result)
    {
      DBG_PRINT("BME280 sensor data: %.2f°C %.2f%% %.2fmbar", bme280_sensor_data.temperature, bme280_sensor_data.humidity, bme280_sensor_data.pressure / 100);
    } else
    {
      DBG_PRINT_CONST("ERROR failed to read data from BME280 sensor");
    }
  } else
  {
    DBG_PRINT_CONST("ERROR BME280 init failed");
  }

  I2C_DISABLE(I2C_MODULE);
}


void test_lpm(void)
{
  /* --- prepare for LPM --- */
  DBG_PRINT_CONST("entering low-power mode...");
  /* turn off peripherals */
#if PERIPH_ENABLE
  PWRSW_OFF(PWRSW_PERIPH);              /* disable power rails */
#endif /* PERIPH_ENABLE */
  PWRSW_OFF(PWRSW_COM);
  UART_DISABLE;
  ADC_DISABLE;
  REF_DISABLE;
  /* disable timers */
  TIMER_A0_STOP;
  TIMER_A1_STOP;
  TIMER32_STOP;
  SYSTICK_STOP;
  RTC_STOP;
  /* reconfigure GPIO */
  PIN_CLR(BOLT_MODE);
  PIN_UNSEL(LED_STATUS);
  LED_OFF(LED_STATUS);
  P1DIR  = 0xff;
  P1OUT  = 0;
  P1SEL0 = 0; P1SEL1 = 0;
  P6DIR  = 0xff;
  P6OUT  = 0;
  P6SEL0 = 0; P6SEL1 = 0;
  P7DIR  = 0xff;
  P7OUT  = 0;
  P7SEL0 = 0; P7SEL1 = 0;
  /* configure port interrupt and enter deepsleep mode */
  PIN_CFG_INT(PIN_APP_EXT0, 1);
  INTERRUPT_ENABLE(INT_PORT3);        /* enable port interrupt */
  PM_DISABLE_SLEEP_ON_ISR_EXIT;       /* wakeup after ISR */
  SVS_DISABLE;                        /* disable SVS */
  PM_ENTER_LPM45;                     /* enter low-power mode 4.5 (same as PCM_shutdownDevice(PCM_LPM45)) */
}



/*
 * I N T E R R U P T   S E R V I C E   R O U T I N E S
 *
 * Note: Nesting is enabled, an interrupt with higher group priority preempts an interrupt with lower group priority.
 * Which IRQ is served next depends on the group priority, the subpriority (within that group) and the IRQ number (lower is served first).
 */

void PORT3_IRQHandler(void)
{
  uint32_t int_status = P3IFG;
  P3IFG = 0;    /* clear */
  if (int_status & BIT0)
  {
    LED_ON(LED_STATUS);
  }
}


/* handle SysTick interrupt */
void SysTick_Handler(void)
{
  /* nothing to do ... */
}


/* Timer A0 (CCR0) interrupt handler */
void TA0_0_IRQHandler(void)
{
  if (TA0CCTL0 & TIMER_A_CCTLN_CCIFG)
  {
    static int32_t led_lum  = 0;
    static int32_t lum_step = 500;

    /* update LED brightness (min = 0, max = PWM_PERIOD) */
    if (led_lum > (PWM_PERIOD - lum_step))
    {
      lum_step = -lum_step;     /* invert direction */
    }
    led_lum += lum_step;
    if (led_lum == 0)           /* reached zero? */
    {
      lum_step = -lum_step;     /* invert direction */
      led_lum = lum_step;       /* make sure led_lum is never 0 (will cause led to stay on for a full cycle!) */
    }
    PWM_TA0_SET(1, led_lum);
    PIN_XOR(PIN_APP_GPIO1);

    TA0CCTL0 &= ~TIMER_A_CCTLN_CCIFG;
  }
}


/* Timer A0 (CCR1-4 + OVF) interrupt handler */
void TA0_N_IRQHandler(void)
{
  if (TA0CTL & TIMER_A_CCTLN_CCIFG)
  {
    /* timer overflow interrupt */
    TA0CTL &= ~TAIFG;
  }
}


void ADC14_IRQHandler(void)
{
  /* 38 interrupt sources: ADC14IFG0 to ADC14IFG31, ADC14OV, ADC14TOV, ADC14LOIFG, ADC14INIFG, ADC14HIIFG and ADC14RDYIFG */
  /* check flag */

  /* MEM1 interrupt */
  if (ADC14->IFGR0 & ADC14_IFGR0_IFG1)
  {
    int32_t val = ADC14->MEM[0];
    v_in = ADC_CONV_TO_MV(val) * 2;
    val = ADC14->MEM[1];
    /* temperature conversion (use calibration data from TLV for 2.5V reference) */
    t_mcu = 30.0f + ((val - (float)TLV->ADC14_REF2P5V_TS30C) * 55.0f / (float)(TLV->ADC14_REF2P5V_TS85C - TLV->ADC14_REF2P5V_TS30C));

  /* MEM0 interrupt */
  } else if (ADC14->IFGR0 & ADC14_IFGR0_IFG0)
  {
    int32_t val = ADC14->MEM[0];
    t_mcu = (((val * 2500 / 16383) - 685) * 10 / 19);
  }
}


void EUSCIA0_IRQHandler(void)
{
  uint8_t rcvd = UART_RXBUF;
#if CONSOLE_ENABLE
  serial_console(rcvd);
#else
  UART_WRITE_BYTE(rcvd);      /* simply echo the received character */
#endif /* CONSOLE_ENABLE */
}


void WDT_A_IRQHandler(void)
{
  DBG_PRINT_CONST("watchdog interrupt");
}

