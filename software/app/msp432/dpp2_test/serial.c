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

/* serial console (UART) */

#include "main.h"


/*
 * G L O B A L S
 */

char debug_print_buffer[DEBUG_PRINT_MAX_LEN];


/*
 * F U N C T I O N S
 */


// CRC-8-Dallas/Maxim
// source: http://stackoverflow.com/questions/29214301/ios-how-to-calculate-crc-8-dallas-maxim-of-nsdata
uint8_t crc8(const uint8_t* data, uint32_t num_bytes, uint8_t init_val)
{
  uint32_t crc  = init_val;
  uint32_t poly = 0x131;

  while (num_bytes)
  {
    crc ^= *data;
    uint32_t bit = 8;
    while (bit)
    {
      if (crc & 0x80) {  crc = (crc << 1) ^ poly; }
      else { crc <<= 1; }
      bit--;
    }
    num_bytes--;
    data++;
  }
  return (uint8_t)crc;
}


uint8_t crc8_table(const uint8_t* data, uint32_t num_bytes)
{
  static unsigned char crc8_array[256] =
  {
    0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
    0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
    0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
    0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
    0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
    0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
    0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
    0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
    0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
    0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
    0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
    0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
    0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
    0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
    0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
    0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
    0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
    0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
    0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
    0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
    0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
    0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
    0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
    0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
    0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
    0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
    0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
    0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
    0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
    0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
    0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
    0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
  };
  uint8_t crc = 0;
  while (num_bytes)
  {
    crc = crc8_array[*data ^ crc];
    data++;
    num_bytes--;
  }
  return crc;
}


/* CRC-16-ANSI / IBM (reversed) */
uint16_t crc16(const uint8_t* data, uint16_t num_bytes, uint16_t init_value)
{
  uint16_t crc  = init_value,
           mask = 0xa001;
  while (num_bytes)
  {
    uint8_t ch = *data;
    int8_t bit = 0;
    while (bit < 8)
    {
      if ((crc & 1) ^ (ch & 1))
      {
        crc = (crc >> 1) ^ mask;
      } else
      {
        crc >>= 1;
      }
      ch >>= 1;
      bit += 1;
    }
    data++;
    num_bytes--;
  }
  return crc;
}


/* CCITT XModem */
uint16_t crc16_ccitt(const uint8_t* data, uint16_t num_bytes, uint16_t init_value)
{
  int i;
  uint32_t crc = init_value;

  for (; num_bytes > 0; num_bytes--)
  {
    crc = crc ^ ((uint32_t)*data++ << 8);
    for (i = 0; i<8; i++)
    {
      crc = crc << 1;
      if (crc & 0x10000)
      {
        crc = (crc ^ 0x1021) & 0xffff;
      }
    }
  }
  return crc;
}


uint32_t crc32(const uint8_t* data, uint32_t num_bytes)
{
  uint32_t i;
  CRC32_setSeed(0xffffffff, CRC32_MODE);
  for (i = 0; i < num_bytes; i++)
  {
    CRC32_set8BitData(data[i], CRC32_MODE);
  }
  return ~CRC32_getResultReversed(CRC32_MODE);
}


/* string must be zero-terminated! */
void debug_print(const char* msg, const char* filename, uint32_t line)
{
  char tmp_buffer[DEBUG_PRINT_MAX_LEN + 64];
  uint32_t now = (uint32_t)TIMESTAMP_MS;

  if (!msg) {
    return;
  }
#if DEBUG_PRINT_FILENAME
  if (!filename) {
    return;
  }
#endif /* DEBUG_PRINT_FILENAME */

#if DEBUG_PRINT_TIMESTAMP
  /* print with timestamp */
  #if DEBUG_PRINT_FILENAME && DEBUG_PRINT_LINENUMBER
    /* print with filename & line number */
    snprintf(tmp_buffer, DEBUG_PRINT_MAX_LEN + 64, "%6lu %s %u: ", now, filename, line);
  #elif DEBUG_PRINT_FILENAME
    snprintf(tmp_buffer, DEBUG_PRINT_MAX_LEN + 64, "%6lu %s: ", now, filename);
  #else
    snprintf(tmp_buffer, DEBUG_PRINT_MAX_LEN + 64, "%6lu ", now);
  #endif
#else
  /* print without timestamp */
  #if DEBUG_PRINT_FILENAME && DEBUG_PRINT_LINENUMBER
    snprintf(tmp_buffer, DEBUG_PRINT_MAX_LEN + 64, "%s %u: ", filename, line);
  #elif DEBUG_PRINT_FILENAME
    snprintf(tmp_buffer, DEBUG_PRINT_MAX_LEN + 64, "%s: ", filename);
  #else
    /* plain string */
    tmp_buffer[0] = 0;
  #endif
#endif

  uart_print(tmp_buffer);
  uart_println(msg);
  UART_WAIT_BUSY;         // wait until the transfer has completed
}


char* to_hexstr(const uint8_t* data, uint32_t len, char* out_buffer, uint32_t buffer_size)
{
  while (len && buffer_size >= 4)
  {
    if ((*data >> 4) > 9)
    {
      *out_buffer++ = ('a' + (*data >> 4) - 10);
    } else
    {
      *out_buffer++ = ('0' + (*data >> 4));
    }
    if ((*data & 0x0f) > 9)
    {
      *out_buffer++ = ('a' + (*data & 0x0f) - 10);
    } else
    {
      *out_buffer++ = ('0' + (*data & 0x0f));
    }
    *out_buffer++ = ' ';
    len--;
    buffer_size -= 3;
    data++;
  }
  *out_buffer = 0;
  return out_buffer;
}


// print data as hex string
void print_hex(const uint8_t* data, uint16_t len)
{
  while (len)
  {
    if ((*data >> 4) > 9)
    {
      UART_WRITE_BYTE('a' + (*data >> 4) - 10);
    } else
    {
      UART_WRITE_BYTE('0' + (*data >> 4));
    }
    if ((*data & 0x0f) > 9)
    {
      UART_WRITE_BYTE('a' + (*data & 0x0f) - 10);
    } else
    {
      UART_WRITE_BYTE('0' + (*data & 0x0f));
    }
    UART_WRITE_BYTE(' ');
    len--;
    data++;
  }
  UART_WRITE_BYTE('\r');
  UART_WRITE_BYTE('\n');
}


// prints unsigned 16-bit integer values over UART A0
void print_uint16(uint16_t* data, uint32_t num_values, char delimiter)
{
  if (!data) { return; }

  while (num_values)
  {
    uint32_t d = *data;
    uint32_t div = d / 10000;
    UART_WRITE_BYTE('0' + div);
    d -= div * 10000;
    div = d / 1000;
    UART_WRITE_BYTE('0' + div);
    d -= div * 1000;
    div = d / 100;
    UART_WRITE_BYTE('0' + div);
    d -= div * 100;
    div = d / 10;
    UART_WRITE_BYTE('0' + div);
    d -= div * 10;
    UART_WRITE_BYTE('0' + d);

    UART_WRITE_BYTE(delimiter);
    num_values--;
    data++;
  }
}


/* print a byte (char) as hex string */
void print_byte_as_hex(uint8_t b)
{
  if ((b >> 4) > 9)
  {
    UART_WRITE_BYTE('a' + (b >> 4) - 10);
  } else
  {
    UART_WRITE_BYTE('0' + (b >> 4));
  }
  if ((b & 0x0f) > 9)
  {
    UART_WRITE_BYTE('a' + (b & 0x0f) - 10);
  } else
  {
    UART_WRITE_BYTE('0' + (b & 0x0f));
  }
}


void print_byte_as_dec(uint8_t b)
{
  if (b > 99) { UART_WRITE_BYTE('0' + (b / 100) % 10); }
  if (b > 9) { UART_WRITE_BYTE('0' + (b / 10) % 10); }
  UART_WRITE_BYTE('0' + (b % 10));
}


/* try to convert the string into a number */
uint32_t str_to_uint32(const char* s)
{
  uint32_t res = 0;
  while (*s >= '0' && *s <= '9')
  {
    res = res * 10 + (*s - '0');
    s++;
  }
  return res;
}


/* try to convert the string into a number */
uint64_t str_to_uint64(const char* s)
{
  uint64_t res = 0;
  /* skip whitespaces */
  while (*s == ' ') s++;
  while (*s >= '0' && *s <= '9')
  {
    res = res * 10 + (*s - '0');
    s++;
  }
  return res;
}


/* converts two hex characters into one byte */
uint_fast16_t hexstr_to_uint16(const char* data)
{
  uint_fast16_t b;
  if (*data >= 'A')
  {
    b = *data - 'A' + 10;
  } else
  {
    b = *data - '0';
  }
  b <<= 4;
  data++;
  if (*data >= 'A')
  {
    b += *data - 'A' + 10;
  } else
  {
    b += *data - '0';
  }
  return b;
}


uint_fast8_t str_starts_with(const char* str, const char* start)
{
  while (str != 0)
  {
    if (*start == 0) { return 1; }
    if (*str != *start) { return 0; }
    str++;
    start++;
  }
  return 0;
}


/* blocking call, sends a data packet over UART */
void serial_send_packet(uint8_t* data, uint16_t len)
{
  static uint16_t packet_id = 0;
  uint16_t i = 0;
  uint32_t crc = 0;
  uint8_t header[6];

  // calculate the CRC checksum
  CRC32_setSeed(0xffffffff, CRC32_MODE);
  for (i = 0; i < len; i++)
  {
    CRC32_set8BitData(data[i], CRC32_MODE);
  }
  crc = ~CRC32_getResultReversed(CRC32_MODE);

  // compose header
  header[0] = packet_id >> 8 & 0xff;
  header[1] = packet_id & 0xff;
  header[2] = (crc >> 24) & 0xff;
  header[3] = (crc >> 16) & 0xff;
  header[4] = (crc >> 8) & 0xff;
  header[5] = crc & 0xff;

  packet_id++;

  // send framing byte (start of packet)
  UART_WRITE_BYTE(PACKET_FRAMING_BYTE);
  // send header
  for (i = 0; i < sizeof(header); i++)
  {
    // escape all 'special' bytes (framing and escape byte)
    if (header[i] == PACKET_FRAMING_BYTE || header[i] == PACKET_ESCAPE_BYTE)
    {
      UART_WRITE_BYTE(PACKET_ESCAPE_BYTE);
    }
    UART_WRITE_BYTE(header[i]);  // send one byte
  }
  // send data (payload)
  for (i = 0; i < len; i++)
  {
    if (data[i] == PACKET_FRAMING_BYTE || data[i] == PACKET_ESCAPE_BYTE)
    {
      UART_WRITE_BYTE(PACKET_ESCAPE_BYTE);
    }
    UART_WRITE_BYTE(data[i]);
  }
  // send framing byte (end of packet)
  UART_WRITE_BYTE(PACKET_FRAMING_BYTE);

  UART_WAIT_BUSY;
}


void console_cmd_led(char* cmd)
{
  if (strstr(cmd, "?") || *cmd == 0)
  {
    uart_println("usage:  led on/off/fade/l=0..255");
  } else
  {
    char* arg = strtok(cmd, " ");
    if (arg != 0)
    {
      if (strstr(arg, "on"))
      {
        PIN_UNSEL(LED_STATUS);
        PIN_SET(LED_STATUS);
      } else if (strstr(arg, "off"))
      {
        PIN_UNSEL(LED_STATUS);
        PIN_CLR(LED_STATUS);
      } else if (strstr(arg, "l="))
      {
        PIN_SEL_PRI(LED_STATUS);
        SYSTICK_INT_DISABLE;
        uint32_t luminance = str_to_uint32(arg + 2) * PWM_PERIOD / 255;
        TA0CCR1 = MIN(PWM_PERIOD, luminance);
      } else if (strstr(arg, "fade"))
      {
        PIN_SEL_PRI(LED_STATUS);
        SYSTICK_INT_ENABLE;
        TA0CCR1 = 0;
      }
    }
  }
}


// toggle a pin
void console_cmd_blink(char* cmd)
{
  char buffer[128];

  if (strstr(cmd, "?") || *cmd == 0)
  {
    uart_println("usage:  blink n=[number of times] f=[frequency]");
  } else
  {
    uint32_t frequency = 5;
    uint32_t count = 10;
    char* arg = strtok(cmd, " ");
    while (arg)
    {
      if (strstr(arg, "f="))
      {
        frequency = str_to_uint32(arg + 2);
        if (frequency < 1 || frequency > 1000000)
        {
          uart_println("invalid frequency (range: 1..100000 / 500000 / 1000000)");
          return;
        }
      } else if (strstr(arg, "n="))
      {
        count = str_to_uint32(arg + 2);
        if (count < 1 || count > 1000000)
        {
          uart_println("invalid count (range: 1..1000000)");
          return;
        }
      }
      arg = strtok(0, " ");
    }
    // make sure the total duration does not exceed 10s
    if (count / frequency > 10)
    {
      uart_println("invalid number / frequency combination (must be <10s total)");
      return;
    }
    // do the toggling (LED and GPIO!)
    PIN_UNSEL(LED_STATUS);
    PIN_CLR(DBG_PIN2);
    PIN_CLR(LED_STATUS);
    count *= 2;

    // handle fast frequencies separately
    if (frequency == 1000000)
    {
      while (count)   // 0.5us (24 instructions) per loop pass
      {
        PIN_XOR(DBG_PIN2);
        PIN_XOR(LED_STATUS);
        __nop();
        count--;
      }

    } else if (frequency == 500000)
    {
      while (count)   // 2.5us (120 instructions) per loop pass
      {
        PIN_XOR(DBG_PIN2);
        PIN_XOR(LED_STATUS);
        __delay_cycles(MCLK_SPEED / 2000000 - 12);
        count--;
      }
    } else if (frequency == 100000)
    {
      while (count)
      {
        PIN_XOR(DBG_PIN2);
        PIN_XOR(LED_STATUS);
        __delay_cycles(MCLK_SPEED / 500000 - 20);
        count--;
      }
    } else if (frequency < 100000)
    {
      frequency = 1333000 / frequency - 1;
      while (count)
      {
        PIN_XOR(DBG_PIN2);
        PIN_XOR(LED_STATUS);
        volatile uint32_t c = frequency;
        while (c) c--;
        count--;
      }
    } else
    {
      uart_println("invalid frequency (range: 1..100000 / 500000 / 1000000)");
    }
  }
}


void console_cmd_bolt(char* cmd)
{
  while (*cmd == ' ' && *cmd != 0) { cmd++; }
  if (strstr(cmd, "?") || *cmd == 0)
  {
    uart_println("usage:  bolt [write/read/status/trq/readmsg/flush] [data]");
  } else
  {
    char* arg = strtok(cmd, " ");
    char buffer[128];
    while (arg != 0)
    {
      if (strstr(arg, "status"))
      {
        // issue a write request to check if BOLT is active
        uint8_t state = bolt_status();
        sprintf(buffer, "input queue: %u\r\noutput queue: %u\r\nbolt ready: %u", BOLT_DATA_AVAILABLE, PIN_GET(BOLT_IND_OUT) > 0, state);
        uart_println(buffer);

      } else if (strstr(arg, "trq"))
      {
        uint64_t timestamp = 0;
        arg = strtok(0, "");  // no delimiter = get remainder
        if (arg != 0) {
          if (strstr(arg, "t")) {
            timestamp = timer32_now() / 3;      // use local time in microseconds
          } else {
            timestamp = str_to_uint64(arg);
          }
        }
        if (timestamp > 0) {
          // send a timestamp
          PIN_SET(BOLT_TREQ);
          PIN_CLR(BOLT_TREQ);
          memset(buffer, 0, 128);
          *(uint16_t*)buffer = 1234;    // sender ID
          buffer[2] = 1;                // message type
          buffer[3] = 8;
          *((uint16_t*)buffer + 2) = 2; // target ID

          memcpy(&buffer[16], &timestamp, 8);
          uint16_t crc = crc16((uint8_t*)buffer, 24, 0);
          *((uint16_t*)&buffer[24]) = crc;
          if (!bolt_write((uint8_t*)buffer, 26)) {
            uart_println("failed to send timestamp");
          } else {
            sprintf(buffer, "timestamp %llu sent", timestamp);
            uart_println(buffer);
          }

        } else {
          // receive a timestamp
          PIN_SET(BOLT_TREQ);
          PIN_CLR(BOLT_TREQ);
          uart_println("timestamp request sent");
        }

      } else if (strstr(arg, "write"))
      {
        // the third argument will be the data
        arg = strtok(0, "");  // no delimiter = get remainder
        if (arg != 0)
        {
          // skip whitespaces at the beginning
          while (*arg == ' ' && *arg != 0) { arg++; }
          // remove the quotation marks (if any)
          if (*arg == '"') { arg++; }
          // the last quotation mark in the string marks the end
          char* mark = strrchr(arg, '"');
          if (mark != 0) { *mark = 0; }
          uint8_t len = strlen(arg);
          if (len)
          {
            sprintf(buffer, "writing message '%s' (%d bytes)", arg, len);
            uart_println(buffer);
            if (!bolt_write((uint8_t*)arg, len)) {
              uart_println("failed to write to bolt");
            }
          } else
          {
            uart_println("invalid argument (empty string)");
          }
        }
        break;

      } else if (strstr(arg, "readmsg"))
      {
        if (BOLT_DATA_AVAILABLE)
        {
          uint8_t len = bolt_read((uint8_t*)buffer);
          if (len)
          {
            uint8_t msg_type = buffer[2];
            uint8_t msg_len  = buffer[3];
            if (len >= 12 && (msg_type == (0x80 | 0x5) || msg_type == (0x80 | 0x1))) {
              uint64_t timestamp;
              memcpy(&timestamp, &buffer[4], 8);
              sprintf(buffer, "message read: type %u, len %u, payload %u, timestamp: %llu", msg_type, len, msg_len, timestamp);
            } else {
              sprintf(buffer, "message read: type %u, len %u, payload %u", msg_type, len, msg_len);
            }
            uart_println(buffer);
          } else
          {
            uart_println("empty message");
          }
          break;
        } else
        {
          uart_println("no data to read (queue empty)");
        }

      } else if (strstr(arg, "read"))
      {
        // is there a third argument?
        arg = strtok(0, "");  // no delimiter = get remainder
        if (arg != 0 && strstr(arg, "-c"))
        {
          // start continuous read (will freeze the console until Ctrl + C is pressed)
          uart_println("waiting for messages from BOLT... (press ctrl + c to abort)");

          while (1)
          {
            if (MAP_UART_getInterruptStatus(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG))
            {
              MAP_UART_clearInterruptFlag(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG);
              if (UCA0RXBUF == 0x03)
              {
                break;
              }
            }
            if (BOLT_DATA_AVAILABLE)
            {
              uint8_t len = bolt_read((uint8_t*)buffer);
              if (len)
              {
                buffer[len] = 0;
                // display the received message in ASCII and hex
                print_byte_as_dec(len);
                uart_println("bytes received:");
                uart_println(buffer);
                print_hex((const uint8_t*)buffer, len);
                uart_print("\r\n");
              }
            }
          }
        } else if (BOLT_DATA_AVAILABLE)
        {
          uint8_t len = bolt_read((uint8_t*)buffer);
          if (len)
          {
            buffer[len] = 0;
            // display the received message in ASCII and hex
            print_byte_as_dec(len);
            uart_println("bytes received:");
            uart_println(buffer);
            print_hex((const uint8_t*)buffer, len);
            uart_print("\r\n");
          } else
          {
            uart_println("empty message");
          }
          break;
        } else
        {
          uart_println("no data to read (queue empty)");
        }

      } else if (strstr(arg, "flush"))
      {
        uint32_t cnt = 0;
        while (BOLT_DATA_AVAILABLE)
        {
          uint8_t len = bolt_read((uint8_t*)buffer);
          if (!len) {
            break;
          }
          cnt++;
        }
        sprintf(buffer, "bolt queue cleared (%u message removed)", cnt);
        uart_println(buffer);

      } else
      {
        uart_println("unknown parameter");
      }
      arg = strtok(0, " ,");   // continue where the previous successful call to strtok() ended
    }
  }
}


/* a very simple command line console */
uint8_t serial_console(uint8_t rcvd_byte)
{
  #define CONSOLE_NUM_CMDS  (4 + CONSOLE_BOLT_CMD + CONSOLE_LWB_CMD + CONSOLE_FW_CMD)
  static const char* commands[CONSOLE_NUM_CMDS] = { "led", "bolt", "exit", "reset", "blink" };
  static const char console_info[] = "[msp432] >";
  static char cmd_history[(CONSOLE_BUFFER_SIZE) * CONSOLE_HISTORY_LEN] = { 0 };
  static char buffer[CONSOLE_BUFFER_SIZE] = { 0 };
  static uint_fast8_t rcv_cnt = 0;
  static uint8_t curr_cmd = 0;    // to 'browse' through the command history
  static uint8_t next_cmd = 0;    // next write index in the command history buffer
  static uint8_t last_byte = 0;
  static uint8_t special = 0;
  uint16_t i = 0;

  // carriage return received?
  if (rcvd_byte == '\r')
  {
    MAP_UART_transmitData(EUSCI_A0_BASE, '\r');
    MAP_UART_transmitData(EUSCI_A0_BASE, '\n');
    buffer[rcv_cnt] = 0;  // make sure the string is zero-terminated
    if (rcv_cnt > 0)
    {
      // save this command in the history
      if (rcv_cnt < CONSOLE_BUFFER_SIZE)
      {
        memcpy(&cmd_history[CONSOLE_BUFFER_SIZE * next_cmd], buffer, rcv_cnt + 1);
        curr_cmd = next_cmd;
        next_cmd = (next_cmd + 1) % CONSOLE_HISTORY_LEN;
      }

      // evaluate the received command, parse the parameters (-> slow linear search..)
      if (str_starts_with(buffer, "reset"))
      {
        uart_println("triggering reset...");
        WAIT_MS(250);
        RESET_TRIGGER_SOFT;
      } else if (str_starts_with(buffer, "blink"))
      {
        console_cmd_blink(buffer + 6);

  #if CONSOLE_BOLT_CMD
      } else if (str_starts_with(buffer, "bolt"))
      {
        console_cmd_bolt(buffer + 4);
  #endif /* CONSOLE_BOLT_CMD */
      } else if (str_starts_with(buffer, "led"))
      {
        console_cmd_led(buffer + 3);
  #if CONSOLE_FW_CMD
      } else if (str_starts_with(buffer, "fw"))
      {
        console_cmd_fw(buffer + 2);
  #endif /* CONSOLE_FW_CMD */
  #if CONSOLE_LWB_CMD
      } else if (str_starts_with(buffer, "lwb"))
      {
        console_cmd_lwb(buffer + 3);
  #endif /* CONSOLE_LWB_CMD */
      } else if (str_starts_with(buffer, "exit"))
      {
        memset(buffer, 0, CONSOLE_BUFFER_SIZE);
        rcv_cnt  = 0;
        return 0;
      } else
      {
        uart_println("unknown command");
      }

      memset(buffer, 0, CONSOLE_BUFFER_SIZE);
      rcv_cnt  = 0;
    }
    // print the info text
    uart_print(console_info);

  // tab received?
  } else if (rcvd_byte == '\t')
  {
    // double tab? show all commands
    if (last_byte == '\t' && rcv_cnt == 0)
    {
      i = 0;
      uart_print("\r\n");
      while (i < CONSOLE_NUM_CMDS)
      {
        uart_print("\t");
        uart_print(commands[i]);
        i++;
      }
      uart_print("\r\n");
      uart_print(console_info);
      last_byte = 0;
      return 1;

    // single tab: try auto-completion
    } else if (rcv_cnt > 0)
    {
      i = 0;
      while (i < CONSOLE_NUM_CMDS)
      {
        if (str_starts_with(commands[i], buffer) && strlen(commands[i]) > rcv_cnt)
        {
          uart_print(commands[i] + rcv_cnt);
          memcpy(buffer, commands[i], strlen(commands[i]));
          rcv_cnt = strlen(commands[i]);
          break;
        }
        i++;
      }
    }

  // backspace
  } else if (rcvd_byte == '\b')
  {
    if (rcv_cnt > 0)
    {
      MAP_UART_transmitData(EUSCI_A0_BASE, '\b');
      MAP_UART_transmitData(EUSCI_A0_BASE, ' ');
      MAP_UART_transmitData(EUSCI_A0_BASE, '\b');
      rcv_cnt--;
      buffer[rcv_cnt] = 0;
    }

  // just a regular character
  } else if (rcvd_byte >= 0x20 && rcvd_byte <= 0x7e)
  {
    // was the last byte a special character?
    if (last_byte == 0x1b)
    {
      special = 1;
    } else if (special && last_byte == 0x5b)
    {
      if (rcvd_byte == 0x41)  // arrow up
      {
        // remove all chars
        while (rcv_cnt)
        {
          MAP_UART_transmitData(EUSCI_A0_BASE, '\b');
          MAP_UART_transmitData(EUSCI_A0_BASE, ' ');
          MAP_UART_transmitData(EUSCI_A0_BASE, '\b');
          rcv_cnt--;
        }
        // print the last command
        i = CONSOLE_BUFFER_SIZE * curr_cmd;
        while (cmd_history[i] != 0)
        {
          MAP_UART_transmitData(EUSCI_A0_BASE, cmd_history[i]);
          i++;
        }
        strcpy(buffer, &cmd_history[CONSOLE_BUFFER_SIZE * curr_cmd]);
        rcv_cnt = strlen(buffer);
        curr_cmd = ((curr_cmd == 0) ? (CONSOLE_HISTORY_LEN - 1) : (curr_cmd - 1));
      }
      special = 0;
    } else
    {
      if (CONSOLE_BUFFER_SIZE == (rcv_cnt + 1))
      {
        // overwrite the last character
        MAP_UART_transmitData(EUSCI_A0_BASE, '\b');
        buffer[rcv_cnt - 1] = rcvd_byte;
      } else
      {
        buffer[rcv_cnt++] = rcvd_byte;
      }
      special = 0;

      // echo the character
      MAP_UART_transmitData(EUSCI_A0_BASE, rcvd_byte);
    }
  }
  last_byte = rcvd_byte;
  return 1;
}
