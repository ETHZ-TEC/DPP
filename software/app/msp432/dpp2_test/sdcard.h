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

#ifndef __SDCARD_H__
#define __SDCARD_H__

/*
 * I N C L U D E S
 */

#include "SDCardLib/sdcardlib.h"  /* Texas Instruments SD card lib */


/*
 * D E F I N E S
 */

#define SDCARD_PRINT_INFO                 1     /* print out card info during initialization? */

#define SDCARD_MAX_ATTEMPTS               100
#define SDCARD_CHECK_STATUS_AFTER_WRITE   1     /* check the card for error flags after a write operation? */

#define SDCARD_USE_CMD1                   1     /* use CMD1 instead of ACMD41 during initialization? */
#define SDCARD_SEND_CMD8                  0     /* send CMD8 during initialization? */
#define SDCARD_SEND_OPTIONAL_CMDS         0     /* send optional commands during initialization phase? */

#define SDCARD_SET_BLOCKLEN_BEFORE_RW     1     /* set block length before a read/write operation? */


/* SD card commands (48 bits: command byte + 4 argument bytes + crc byte) */

#define SDCARD_CMD0       0           /* reset card (go to idle state, enter SPI mode) */
#define SDCARD_CMD1       1           /* send operating condition (for MMC cards) */
#define SDCARD_CMD8       8           /* send interface condition */
#define SDCARD_CMD9       9           /* send card status */
#define SDCARD_CMD10      10          /* send CID (card identification register) */
#define SDCARD_CMD12      12          /* stop transmission */
#define SDCARD_CMD13      13          /* send SD status */
#define SDCARD_CMD16      16          /* set block length */
#define SDCARD_CMD17      17          /* read single block */
#define SDCARD_CMD18      18          /* read multiple blocks */
#define SDCARD_CMD23      23          /* set block count */
#define SDCARD_CMD24      24          /* write single block */
#define SDCARD_CMD25      25          /* write multiple blocks */
#define SDCARD_CMD55      55          /* next command is an application specific command */
#define SDCARD_CMD58      58          /* read the OCR register */
#define SDCARD_CMD59      59          /* disable CRC (should be disabled by default in SPI mode) */

/* application specific commands */
#define SDCARD_ACMD41     41          /* SD send operating condition */


/* CSD (Card-Specific Data register) */

/* Card type flags */
#define SDCARD_CT_MMC     0x01        /* MMC ver 3 */
#define SDCARD_CT_SD1     0x02        /* SD ver 1 */
#define SDCARD_CT_SD2     0x04        /* SD ver 2 */
#define SDCARD_CT_SDC     (SDCARD_CT_SD1|SDCARD_CT_SD2)   /* SD */
#define SDCARD_CT_BLOCK   0x08        /* Block addressing */

/* data response token */
#define SDCARD_DATA_ACCEPTED  0xe5    /* 5 = accepted */
#define SDCARD_DATA_REJECTED  0xea    /* a = rejected due to CRC error */
#define SDCARD_DATA_REJECTED2 0xec    /* c = rejected due to write error */

#define SDCARD_DATA_START     0Xfe    /* start block token */


/*
 * R1 response from SD card:
 * - start bit (0)
 * - parameter error indicator
 * - address error indicator
 * - erase sequence error indicator
 * - CRC error indicator
 * - illegal command indicator
 * - erase reset indicator
 * - in idle state indicator
 *
 * If read operation fails, a read error token (1 byte) is returned:
 * - first 3 bits are '0'
 * - card locked indicator
 * - out of range indicator
 * - card ECC failed indicator
 * - card controller error indicator
 * - unspecified error indicator
 *
 * R2 response: Status register error indicators (see p.244), MSB to LSB:
 * - parameter error (0x4000)
 * - address error (0x2000)
 * - erase sequence error (0x1000)
 * - com crc error (0x0800)
 * - illegal command (0x0400)
 * - erase reset (0x0200)
 * - in idle state (0x0100)
 * - out of range / CSD overwrite (0x0080)
 * - erase param (0x0040)
 * - write protection violation (0x0020)
 * - card ECC failed (0x0010)
 * - internal card controller error (0x0008)
 * - general or unknown error (0x0004)
 * - write protection erase skip or lock/unlock CMD failed (0x0002)
 * - card is locked (0x0001)
 */


/*
 * G L O B A L S
 */

extern SDCardLib_Interface sdint_msp432p401;


/*
 * F U N C T I O N S
 */

uint_fast8_t sdcard_init(void);
uint_fast8_t sdcard_write_block(uint32_t address, const uint8_t* data, uint32_t len);
uint_fast8_t sdcard_read_block(uint32_t address, uint8_t* out_data);
uint_fast16_t sdcard_status(void);
uint_fast8_t sdcard_set_blocklen(uint32_t len);
uint_fast8_t sdcard_print_cid(void);
uint32_t sdcard_get_size(void);


#endif /* __SDCARD_H__ */
