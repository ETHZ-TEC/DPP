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
 * I N C L U D E S
 */

#include "main.h"


/*
 * T Y P E D E F S   &   G L O B A L S
 */

typedef enum
{
  BOLT_STATE_IDLE = 0,
  BOLT_STATE_READ,
  BOLT_STATE_WRITE,
  NUM_OF_STATES
} bolt_state_t;

typedef enum
{
  BOLT_OP_READ = 0,
  BOLT_OP_WRITE,
  NUM_OF_OPS
} bolt_op_mode_t;

static volatile bolt_state_t bolt_state = BOLT_STATE_IDLE;


/*
 * F U N C T I O N S
 */

/* init the GPIOs + SPI and check if BOLT is ready */
uint_fast8_t bolt_init(void)
{
  /* init GPIOs (already configured in gpio_init) */
  PIN_CFG_IN(BOLT_IND);
  //PIN_CFG_INT(BOLT_IND, 1);
  PIN_CFG_IN(BOLT_ACK);
  PIN_CFG_IN(BOLT_IND_OUT);
  PIN_CLR(BOLT_MODE);
  PIN_CLR(BOLT_TREQ);
  PIN_CLR(BOLT_REQ);
  PIN_CFG_OUT(BOLT_MODE);
  PIN_CFG_OUT(BOLT_REQ);
  PIN_CFG_OUT(BOLT_TREQ);
  PIN_SEL_PRI(BOLT_SPI_SCK);
  PIN_SEL_PRI(BOLT_SPI_MOSI);
  PIN_SEL_PRI(BOLT_SPI_MISO);

  /* init SPI module */
  spi_b_init(BOLT_SPI_MODULE, BOLT_SPI_SPEED, BOLT_SPI_CPOL, BOLT_SPI_CPHA);

  /* check whether BOLT is available */
  WAIT_MS(2);  /* make sure that at least 2ms have passed since MCU startup */
  if (!bolt_status())
  {
    return 0;
  }
  return 1;
}


void bolt_release(void)
{
  if (SPI_IS_ENABLED(BOLT_SPI_MODULE))
  {
    /* --- wait for BUSY flag --- */
    SPI_WAIT_BUSY(BOLT_SPI_MODULE);
    /* --- empty the RX buffer --- */
    SPI_CLR_RXIFG(BOLT_SPI_MODULE);
    SPI_DISABLE(BOLT_SPI_MODULE);
  }
  /* --- set REQ = 0 --- */
  PIN_CLR(BOLT_REQ);

  /* --- wait for ACK to go down --- */
  while (PIN_GET(BOLT_ACK));
  bolt_state = BOLT_STATE_IDLE;
}


uint_fast8_t bolt_acquire(bolt_op_mode_t mode)
{
  if (PIN_GET(BOLT_REQ) || PIN_GET(BOLT_ACK))
  {
    return 1;
  }
  if (BOLT_STATE_IDLE != bolt_state)
  {
    return 2;
  }

  /* --- MODE --- */
  if (BOLT_OP_READ == mode)
  {
    if (!BOLT_DATA_AVAILABLE)
    {
      return 3;
    }
    PIN_CLR(BOLT_MODE); /* 0 = READ */
  } else
  {
    PIN_SET(BOLT_MODE); /* 1 = WRITE */
  }

  /* --- set REQ = 1 --- */
  PIN_SET(BOLT_REQ);

  /* now wait for a rising edge on the ACK line (max. 100us) */
  uint_fast8_t cnt = 0;
  do
  {
    WAIT_US(10);     /* wait 10 us */
    cnt++;
  } while (!PIN_GET(BOLT_ACK) && cnt < 10);

  if (!PIN_GET(BOLT_ACK))
  {
    /* ack is still low -> failed */
    bolt_state = BOLT_STATE_IDLE;
    PIN_CLR(BOLT_REQ);
    return 4;
  }
  bolt_state = (mode == BOLT_OP_READ) ? BOLT_STATE_READ : BOLT_STATE_WRITE;

  /* make sure SPI is enabled */
  SPI_ENABLE(BOLT_SPI_MODULE);

  return 0;
}


/* returns 1 if BOLT is active/ready (= responds to a write request), 0 otherwise */
uint_fast8_t bolt_status(void)
{
  if (bolt_acquire(BOLT_OP_WRITE) == 0)
  {
    bolt_release();
    return 1;
  }
  return 0;
}


/* clear the bolt queue (read and drop all messages) */
void bolt_flush(void)
{
  uint8_t buffer[BOLT_MAX_MSG_LEN];
  while (bolt_read(buffer));
}


uint32_t bolt_read(uint8_t *out_data)
{
  if (!out_data)
  {
    DBG_PRINT_CONST("ERROR invalid parameter");
    return 0;
  }
  uint_fast8_t res = bolt_acquire(BOLT_OP_READ);
  if (res != 0)
  {
    DBG_PRINT("ERROR request failed (code %u)", res);
    return 0;
  }
  SPI_CLR_RXIFG(BOLT_SPI_MODULE);   /* first, clear the RX buffer */
#if SPI_FAST_READ
  /* transmit 1 byte ahead for faster read speed (fills RXBUF faster) */
  SPI_TRANSMIT_BYTE(0x00);
#endif
  uint32_t count = 0;
  while ((count < BOLT_MAX_MSG_LEN) && PIN_GET(BOLT_ACK))
  {
    SPI_READ_BYTE(BOLT_SPI_MODULE, *out_data);
    out_data++;
    count++;
  }
  bolt_release();
  return count;
}


uint_fast8_t bolt_write(const uint8_t *data, uint32_t num_bytes)
{
  if (!data || num_bytes == 0)
  {
    DBG_PRINT_CONST("ERROR invalid parameter");
    return 0;
  }
  uint_fast8_t res = bolt_acquire(BOLT_OP_WRITE);
  if (res != 0)
  {
    DBG_PRINT("ERROR request failed (code %u)", res);
    return 0;
  }
  while (num_bytes)
  {
    SPI_WRITE_BYTE(BOLT_SPI_MODULE, *data);
    data++;
    num_bytes--;
  }
  bolt_release();
  return 1;
}

