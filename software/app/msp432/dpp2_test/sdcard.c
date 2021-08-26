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
 * SD card access in SPI mode
 *
 * based on the TI MSP430FR5994 Launchpad demo code
 * (link: http://www.ti.com/tool/msp-exp430fr5994)
 *
 * SD specs: https://www.sdcard.org/downloads/pls/index.html
 */

#include "main.h"
#include "sdcard.h"


/*
 * D E F I N E S  and  T Y P E D E F S
 */

/* set CS low (active state) */
#define SDCARD_CS_ASSERT          PIN_CLR(SDCARD_SPI_CS)
/* set CS high (inactive state) - note: according to SD specs, 8 more clock cycles must be provided to the card after each transaction */
#define SDCARD_CS_DEASSERT        PIN_SET(SDCARD_SPI_CS); WAIT_US(5); SPI_WRITE_BYTE(SDCARD_SPI_MODULE, 0xff); SPI_WAIT_BUSY(SDCARD_SPI_MODULE); WAIT_US(5)


/* prototype definition of internal functions */
void sdcard_spi_init(void);
uint_fast8_t sdcard_send_cmd(uint_fast8_t cmd, uint32_t arg, uint8_t* out_buffer, uint32_t buffer_size);

void sdcard_fastmode(void);
void sdcard_read_bytes(uint8_t *out_buffer, uint16_t size);
void sdcard_write_bytes(uint8_t *data, uint16_t size);
void sdcard_set_cs_high(void);
void sdcard_set_cs_low(void);
SDCardLib_Status sdcard_detect(void);
Calendar sdcard_get_rtctime(void);
void sdcard_set_rtctime(Calendar *curTime);


/*
 * G L O B A L S
 */

SDCardLib_Interface sdint_msp432p401 =
{
  SPI_CLK_SPEED,
  sdcard_spi_init,
  sdcard_fastmode,
  sdcard_read_bytes,
  sdcard_write_bytes,
  sdcard_set_cs_high,
  sdcard_set_cs_low,
  sdcard_detect,
  sdcard_get_rtctime,
  sdcard_set_rtctime
};

static uint32_t sdcard_size = 0;


/*
 * F U N C T I O N S
 */


void sdcard_spi_init(void)
{
  /* note: pin config/mapping is done in gpio_init() */
  PIN_SEL_PRI(SDCARD_SPI_SCK);
  PIN_SEL_PRI(SDCARD_SPI_MOSI);
  PIN_SEL_PRI(SDCARD_SPI_MISO);
  PIN_SET(SDCARD_SPI_CS);
  PIN_CFG_OUT(SDCARD_SPI_CS);
  PIN_PULLUP_EN(SDCARD_SPI_MISO);   /* some cards require a pullup on the MISO line! */

  /* init SPI module and enable it */
  spi_a_init(SDCARD_SPI_MODULE, SDCARD_SPI_SPEED, SDCARD_SPI_CPOL, SDCARD_SPI_CPHA);
  SPI_ENABLE(SDCARD_SPI_MODULE);
  /* note: for compatibility reasons, SPI clock speed should be set to 400kHz for initialization */
}


uint_fast8_t sdcard_init(void)
{
  uint_fast8_t res = 0;

  sdcard_spi_init();

  /* startup delay */
  //SDCARD_CS_ASSERT;
  spi_a_write_const(SDCARD_SPI_MODULE, 9, 0xff);
  //SDCARD_CS_DEASSERT;
  WAIT_MS(1);

  /* send CMD0 (go to idle state) */
  res = sdcard_send_cmd(SDCARD_CMD0, 0, 0, 0);
  if (res != 0x01)
  {
    return res;
  }
#if SDCARD_PRINT_INFO
  DBG_PRINT_CONST("SD card detected");
#endif /* SDCARD_PRINT_INFO */

#if SDCARD_SEND_CMD8
  /* send CMD8 (interface condition) */
  res = sdcard_send_cmd(SDCARD_CMD8, 0x000001aa, 0, 0);
  if (res == 0x01)
  {
    /* it's a Ver2.00 or later SD card */
 #if SDCARD_PRINT_INFO
    DBG_PRINT_CONST("card is v2.00 or later");
 #endif /* SDCARD_PRINT_INFO */
  }
#endif /* SDCARD_SEND_CMD8 */

  /* read OCR by sending CMD58 to get supported voltage range (optional, although some cards may require it) */
#if SDCARD_SEND_OPTIONAL_CMDS
  sdcard_send_cmd(SDCARD_CMD58, 0, 0, 0);
#endif /* SDCARD_SEND_OPTIONAL_CMDS */

  /* send ACMD49 (send operating condition) repeatedly until the idle state indicator bit is cleared (wait up to ~1s) */
  /* use 'power saving' mode (= up to 100mA) and indicate that only SDSC cards are supported, but not SDHC or SDXC */
  uint32_t cnt = SDCARD_MAX_ATTEMPTS;
  do
  {
#if SDCARD_USE_CMD1
    /* just send CMD1 */
    res = sdcard_send_cmd(SDCARD_CMD1, 0x00000000, 0, 0);
#else /* SDCARD_USE_CMD1 */

    /* send CMD55 to indicate that the next command will be application specific (ACMD49) */
    res = sdcard_send_cmd(SDCARD_CMD55, 0, 0, 0);
    if (res != 0x01)
    {
 #if SDCARD_PRINT_INFO
      DBG_PRINT_CONST("old SD card (CMD55 rejected)");
 #endif /* SDCARD_PRINT_INFO */
      return res;
    }
    res = sdcard_send_cmd(SDCARD_ACMD41, 0x40000000, 0, 0);
#endif /* SDCARD_USE_CMD1 */
    WAIT_MS(10);    /* should be at most 50ms */
    cnt--;
  } while (cnt && (res & 0x01));
  if (res != 0)
  {
    /* timeout -> not a valid SD card */
    return res;
  }

  /* no voltage negotiation necessary: all cards work with 2.7 - 3.6V */

  /* the card is now ready: switch to high-speed mode */
  spi_a_set_speed(SDCARD_SPI_MODULE, SDCARD_SPI_SPEED_FAST);

  /* disable CRC (optional, should be disabled by default) */
#if SDCARD_SEND_OPTIONAL_CMDS
  sdcard_send_cmd(SDCARD_CMD59, 0, 0, 0);
#endif /* SDCARD_SEND_OPTIONAL_CMDS */

  /* read the CSD register (card specific data) with CMD9 */
  uint8_t csd[18];
  memset(csd, 0, 18);
  res = sdcard_send_cmd(SDCARD_CMD9, 0, csd, 18);
  if (res != 0)
  {
#if SDCARD_PRINT_INFO
    DBG_PRINT("CSD register read failed");
#endif /* SDCARD_PRINT_INFO */
    return res;
  }
#if SDCARD_PRINT_INFO
  char strbuf[128];
  memcpy(strbuf, "CSD register: ", 14);
  to_hexstr(csd, 16, &strbuf[14], 128 - 14);
  DBG_PRINT_CONST(strbuf);
#endif /* SDCARD_PRINT_INFO */

  /* check card type */
  if (csd[0] & 0xc0)
  {
#if SDCARD_PRINT_INFO
    DBG_PRINT_CONST("card is SDHC or SDXC");
#endif /* SDCARD_PRINT_INFO */
  } else
  {
#if SDCARD_PRINT_INFO
    /* standard capacity card */
    DBG_PRINT("card command classes (CCC): 0x%x%x", csd[4], csd[5] >> 4);
#endif /* SDCARD_PRINT_INFO */

    /* C_SIZE (12 bit) */
    uint32_t mult = 1 << ((((csd[9] & 0x03) << 1) | csd[10] >> 7) + 2);
    uint32_t c_size = ((uint32_t)(csd[6] & 0x03) << 10) | ((uint32_t)csd[7] << 2) | (csd[8] >> 6);
    uint32_t read_bl_len = (uint32_t)1 << (csd[5] & 0x0f);
    sdcard_size = (read_bl_len * (c_size + 1) * mult) >> 20;
#if SDCARD_PRINT_INFO
    DBG_PRINT("blockcnt: %u, blocklen: %u, total capacity: %uMB", (c_size + 1) * mult, read_bl_len, sdcard_size);
#endif /* SDCARD_PRINT_INFO */
  }

  /* read and print the CID register data */
#if SDCARD_PRINT_INFO
  sdcard_print_cid();
#endif /* SDCARD_PRINT_INFO */

  /* check the card status */
  uint16_t status = sdcard_status();
#if SDCARD_PRINT_INFO
  DBG_PRINT("card status: 0x%x", status);
#endif /* SDCARD_PRINT_INFO */

  return 0;   /* success */
}


/* returns 0 if successful, error code otherwise */
uint_fast8_t sdcard_send_cmd(uint_fast8_t cmd, uint32_t arg, uint8_t* out_buffer, uint32_t buffer_size)
{
  uint_fast8_t res = 0;

  /* last bit set? => application specific command, send CMD55 first */
  if (cmd & 0x80)
  {
    cmd &= 0x7f;
    res = sdcard_send_cmd(SDCARD_CMD55, 0, 0, 0);
    if (res != 0) return res;
  }

  /* select the card */
  SDCARD_CS_ASSERT;

  /* compose and send a command packet */
  uint8_t buffer[6];
  buffer[0] = 0x40 | cmd;                     /* start bit '0' + TX bit '1' + command index */
  buffer[1] = (uint8_t)(arg >> 24);           /* argument[31..24] */
  buffer[2] = (uint8_t)(arg >> 16);           /* argument[23..16] */
  buffer[3] = (uint8_t)(arg >> 8);            /* argument[15..8]  */
  buffer[4] = (uint8_t)arg;                   /* argument[7..0]   */
  switch (cmd)
  {
  /* according to specs only CMD0 and CMD8 require a valid CRC, but some cards may even require a valid CRC for other commands */
  case SDCARD_CMD0:
    buffer[5] = 0x95;   /* valid CRC for CMD0(0)     */
    break;
  case SDCARD_CMD8:
    buffer[5] = 0x87;   /* valid CRC for CMD8(0x1AA) */
    break;
  case SDCARD_CMD1:
    buffer[5] = 0xf9;
    break;
  case SDCARD_CMD55:
    buffer[5] = 0x65;
    break;
  case SDCARD_ACMD41:
    buffer[5] = 0x77;   /* or 0xe5 for SDHC cards (argument 0x40000000) */
    break;
  default:
    buffer[5] = 0xff;   /* dummy CRC + stop/end bit */
    break;
  }
  spi_a_write(SDCARD_SPI_MODULE, 6, buffer);

  /* wait for the response */
  uint_fast8_t cnt = SDCARD_MAX_ATTEMPTS;
  do
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
    cnt--;
  } while (cnt && (res & 0x80));

  /* check whether command has been accepted */
  if (res != 0)
  {
    /* error or timeout */
    SDCARD_CS_DEASSERT;
    return res;
  }

  /* retrieve the response? */
  if (buffer_size && out_buffer)
  {
    /* wait for the start block */
    cnt = SDCARD_MAX_ATTEMPTS;
    do
    {
      SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
      cnt--;
    } while ((res == 0xff) && cnt);

    /* read the response */
    if (SDCARD_DATA_START == res)
    {
      while (buffer_size)
      {
        SPI_READ_BYTE(SDCARD_SPI_MODULE, *out_buffer);
        out_buffer++;
        buffer_size--;
      }
      SDCARD_CS_DEASSERT;
      return 0;
    }
    /* else: error token received */
  }
  /* else: not interested in response */
  SDCARD_CS_DEASSERT;

  return res;
}


uint_fast8_t sdcard_set_blocklen(uint32_t len)
{
  return sdcard_send_cmd(SDCARD_CMD16, len, 0, 0);
}


uint_fast16_t sdcard_status(void)
{
  uint_fast16_t status = 0;

  /* select the card */
  SDCARD_CS_ASSERT;

  /* compose and send a command packet */
  uint8_t buffer[6];
  memset(buffer, 0, 6);
  buffer[0] = 0x40 | SDCARD_CMD13;            /* start + command index */
  buffer[5] = 0xff;
  spi_a_write(SDCARD_SPI_MODULE, 6, buffer);

  /* wait for the response (R2 format, 2 bytes long) */
  uint_fast8_t cnt = SDCARD_MAX_ATTEMPTS;
  uint_fast16_t rcvd;
  do
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, rcvd);
    cnt--;
  } while (cnt && (rcvd & 0x80));

  SPI_READ_BYTE(SDCARD_SPI_MODULE, status);
  status |= (rcvd << 8);

  SDCARD_CS_DEASSERT;
  return status;
}


/* returns 0 if successful, error code otherwise */
uint_fast8_t sdcard_write_block(uint32_t address, const uint8_t* data, uint32_t len)
{
  uint_fast8_t res = 0;

  if (len > SDCARD_BLOCK_SIZE) return 0xff;   /* invalid argument */

#if SDCARD_SET_BLOCKLEN_BEFORE_RW
  /* set the block length to SDCARD_BLOCK_SIZE bytes (typically 512 bytes) */
  res = sdcard_set_blocklen(SDCARD_BLOCK_SIZE);
  if (res != 0) return res;
#endif /* SDCARD_SET_BLOCKLEN_BEFORE_RW */

  /* select the card */
  SDCARD_CS_ASSERT;

  /* compose and send a command packet */
  uint8_t buffer[6];
  buffer[0] = 0x40 | SDCARD_CMD24;            /* start bit '0' + TX bit '1' + command index */
  buffer[1] = (uint8_t)(address >> 24);       /* argument[31..24] */
  buffer[2] = (uint8_t)(address >> 16);       /* argument[23..16] */
  buffer[3] = (uint8_t)(address >> 8);        /* argument[15..8]  */
  buffer[4] = (uint8_t)address;               /* argument[7..0]   */
  buffer[5] = 0xff;                           /* dummy CRC + stop */

  spi_a_write(SDCARD_SPI_MODULE, 6, buffer);

  /* wait for the response */
  uint_fast8_t cnt = SDCARD_MAX_ATTEMPTS;
  do
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
    cnt--;
  } while (cnt && (res & 0x80));

  /* check whether command has been accepted */
  if (res != 0)
  {
    /* error or timeout */
    SDCARD_CS_DEASSERT;
    return res;
  }

  /* send start token */
  SPI_WRITE_BYTE(SDCARD_SPI_MODULE, SDCARD_DATA_START);

  /* write block of data */
  spi_a_write(SDCARD_SPI_MODULE, len, data);
  /* fill the remaining bytes + add 2 more bytes (dummy CRC) */
  spi_a_write_const(SDCARD_SPI_MODULE, SDCARD_BLOCK_SIZE - len + 2, 0);
  /* get response: check whether data has been accepted */
  SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
  if (res != SDCARD_DATA_ACCEPTED)
  {
    DBG_PRINT("write failed: data rejected with code 0x%x", res);
    SDCARD_CS_DEASSERT;
    return res;
  }
  res = 0;

  /* wait for the MISO line to go high */
  while (PIN_GET(SDCARD_SPI_MISO) == 0)
  {
    SPI_WRITE_BYTE(SDCARD_SPI_MODULE, 0xff);
    WAIT_US(100);
  }
  SDCARD_CS_DEASSERT;

#if SDCARD_CHECK_STATUS_AFTER_WRITE
  /* check status */
  uint16_t status = sdcard_status();
  if (status != 0)
  {
    DBG_PRINT("card status error 0x%x detected", status);
    return 0xfe;
  }
#endif /* SDCARD_CHECK_STATUS_AFTER_WRITE */

  return res;
}


/* returns 0 if successful, error code otherwise
 * Note: output buffer must be large enough to hold SDCARD_BLOCK_SIZE bytes! */
uint_fast8_t sdcard_read_block(uint32_t address, uint8_t* out_data)
{
  uint_fast8_t res = 0;

#if SDCARD_SET_BLOCKLEN_BEFORE_RW
  /* set the block length to SDCARD_BLOCK_SIZE bytes (typically 512 bytes) */
  res = sdcard_set_blocklen(SDCARD_BLOCK_SIZE);
  if (res != 0) return res;
#endif /* SDCARD_SET_BLOCKLEN_BEFORE_RW */

  /* select the card */
  SDCARD_CS_ASSERT;

  /* compose and send a command packet */
  uint8_t buffer[6];
  buffer[0] = 0x40 | SDCARD_CMD17;            /* start + command index */
  buffer[1] = (uint8_t)(address >> 24);       /* argument[31..24] */
  buffer[2] = (uint8_t)(address >> 16);       /* argument[23..16] */
  buffer[3] = (uint8_t)(address >> 8);        /* argument[15..8]  */
  buffer[4] = (uint8_t)address;               /* argument[7..0]   */
  buffer[5] = 0xff;                           /* dummy CRC + stop */

  spi_a_write(SDCARD_SPI_MODULE, 6, buffer);

  /* wait for the response */
  uint_fast8_t cnt = SDCARD_MAX_ATTEMPTS;
  do
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
    cnt--;
  } while (cnt && (res & 0x80));

  /* check whether command has been accepted */
  if (res != 0)
  {
    /* error or timeout */
    SDCARD_CS_DEASSERT;
    return res;
  }

  /* wait for the start token */
  do
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
  } while ((res & 0x80) && res != SDCARD_DATA_START);

  /* read block of data */
  spi_a_read(SDCARD_SPI_MODULE, SDCARD_BLOCK_SIZE, out_data);

  /* read the CRC: type is CRC-CCITT (XModem) */
  uint16_t crc = 0;
  SPI_READ_BYTE(SDCARD_SPI_MODULE, res);
  SPI_READ_BYTE(SDCARD_SPI_MODULE, crc);
  crc |= ((uint_fast16_t)res << 8);

  SDCARD_CS_DEASSERT;

  /* check crc value */
  if (crc16_ccitt(out_data, SDCARD_BLOCK_SIZE, 0) != crc)
  {
    DBG_PRINT("WARNING invalid CRC 0x%x", crc);
  }

  return 0;
}


/* print the card identification register data */
uint_fast8_t sdcard_print_cid(void)
{
  uint8_t buf[18] = { 0 };
  char name[6] = { 0 };
  uint32_t serial;
  uint_fast8_t res = sdcard_send_cmd(SDCARD_CMD10, 0, buf, 18);
  if (res != 0)
  {
    DBG_PRINT_CONST("CID register read failed");
    return res;
  }
  memcpy(name, &buf[3], 5);
  memcpy(&serial, &buf[9], 4); // offset 9 to 12
  DBG_PRINT("CID register: man_id=%x oem_id=%c%c name=%s rev=%x serial=%x date=%u-%02u", buf[0], buf[1], buf[2], name, buf[8], serial, 2000 + (buf[13] << 4) | (buf[14] >> 4), buf[14] & 0x0f);
  return 0;
}


uint32_t sdcard_get_size(void)
{
  return sdcard_size;
}


/* switch to the fast mode */
void sdcard_fastmode(void)
{
  spi_a_set_speed(SDCARD_SPI_MODULE, SDCARD_SPI_SPEED_FAST);
}


/* atomic/blocking SPI read operation */
void sdcard_read_bytes(uint8_t *out_buffer, uint16_t size)
{
  uint_fast8_t ie = INTERRUPT_MASTER_ENABLED;
  __disable_interrupt();
  SPI_CLR_RXIFG(SDCARD_SPI_MODULE);
  while (size)
  {
    SPI_READ_BYTE(SDCARD_SPI_MODULE, *out_buffer);
    out_buffer++;
    size--;
  }
  if (ie) { __enable_interrupt(); }
}


/* atomic/blocking SPI write operation */
void sdcard_write_bytes(uint8_t *data, uint16_t size)
{
  uint_fast8_t ie = INTERRUPT_MASTER_ENABLED;
  __disable_interrupt();
  while (size)
  {
    SPI_WRITE_BYTE(SDCARD_SPI_MODULE, *data);
    data++;
    size--;
  }
  SPI_WAIT_BUSY(SDCARD_SPI_MODULE);
  (void)SDCARD_SPI_MODULE->RXBUF;   /* dummy read to clear RX buffer and overrun condition */
  if (ie) { __enable_interrupt(); }
}


void sdcard_set_cs_high(void)
{
  PIN_SET(SDCARD_SPI_CS);
}


void sdcard_set_cs_low(void)
{
  PIN_CLR(SDCARD_SPI_CS);
}


SDCardLib_Status sdcard_detect(void)
{
  /* detect pin not available in this design */
  return SDCARDLIB_STATUS_PRESENT;
}


Calendar sdcard_get_rtctime(void)
{
  static Calendar cal;
  RTC_C_Calendar rtc_cal = RTC_C_getCalendarTime();
  memcpy(&cal, &rtc_cal, sizeof(Calendar));
  return cal;
}


void sdcard_set_rtctime(Calendar *curTime)
{
  RTC_C_initCalendar((RTC_C_Calendar*)curTime, RTC_C_FORMAT_BINARY);
  RTC_C_startClock();
}
