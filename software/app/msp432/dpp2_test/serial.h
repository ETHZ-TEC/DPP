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

/* serial console und debug prints (UART) */

#ifndef __SERIAL_H__
#define __SERIAL_H__


/*
 * D E F I N I T I O N S
 */

#ifndef DEBUG_PRINT_ENABLE
#define DEBUG_PRINT_ENABLE      1
#endif /* DEBUG_PRINT_ON */

#ifndef DEBUG_PRINT_MAX_LEN
#define DEBUG_PRINT_MAX_LEN     512
#endif /* DEBUG_PRINT_MAX_LEN */

#ifndef DEBUG_PRINT_TIMESTAMP
#define DEBUG_PRINT_TIMESTAMP   1
#endif /* DEBUG_PRINT_TIMESTAMP */

#ifndef DEBUG_PRINT_FILENAME
#define DEBUG_PRINT_FILENAME    1
#endif /* DEBUG_PRINT_FILENAME */

/* can only be enabled when DEBUG_PRINT_FILENAME is enabled */
#ifndef DEBUG_PRINT_LINENUMBER
#define DEBUG_PRINT_LINENUMBER  1
#endif /* DEBUG_PRINT_LINENO */

#ifndef CONSOLE_BUFFER_SIZE
#define CONSOLE_BUFFER_SIZE     64      /* no more than 64 characters per command */
#endif /* CONSOLE_BUFFER_SIZE */

#ifndef CONSOLE_HISTORY_LEN
#define CONSOLE_HISTORY_LEN     4       /* store the last x commands (should be power of 2) */
#endif /* CONSOLE_HISTORY_LEN */

#ifndef CONSOLE_LWB_CMD
#define CONSOLE_LWB_CMD         0
#endif /* CONSOLE_LWB_CMD */

#ifndef CONSOLE_FW_CMD
#define CONSOLE_FW_CMD          0
#endif /* CONSOLE_FW_CMD */

#ifndef CONSOLE_BOLT_CMD
#define CONSOLE_BOLT_CMD        1
#endif /* CONSOLE_BOLT_CMD */

#ifndef CONSOLE_ENABLE
#define CONSOLE_ENABLE          0
#endif /* CONSOLE */


/* framing byte for binary data transmission over UART */
#define PACKET_FRAMING_BYTE     0x7e
#define PACKET_ESCAPE_BYTE      0x7d


/*
 * M A C R O S
 */

#define FILENAME                (strrchr(__FILE__, '/') ? \
                                 strrchr(__FILE__, '/') + 1 : __FILE__)       // strrchr() will be evaluated at compile time and the file name inlined

#if DEBUG_PRINT_ENABLE
  #define DBG_PRINT(...)        snprintf(debug_print_buffer, DEBUG_PRINT_MAX_LEN, __VA_ARGS__); \
                                debug_print(debug_print_buffer, FILENAME, __LINE__)
  #define DBG_PRINT_CONST(str)  debug_print(str, FILENAME, __LINE__)
#else /* DEBUG_PRINT_ENABLE */
  #define DBG_PRINT(...)
  #define DBG_PRINT_CONST(str)
#endif /* DEBUG_PRINT_ENABLE */


/*
 * P R O T O T Y P E S
 */

// CRC functions
uint8_t  crc8(const uint8_t* data, uint32_t num_bytes, uint8_t init_val);
uint8_t  crc8_table(const uint8_t* data, uint32_t num_bytes);
uint16_t crc16(const uint8_t* data, uint16_t num_bytes, uint16_t init_value);
uint16_t crc16_ccitt(const uint8_t* data, uint16_t num_bytes, uint16_t init_value);
uint32_t crc32(const uint8_t* data, uint32_t num_bytes);

// debug print functions
void debug_print(const char* msg, const char* filename, uint32_t line_no);
void print_hex(const uint8_t* data, uint16_t len);
void print_uint16(uint16_t* data, uint32_t num_values, char delimiter);
void print_byte_as_hex(uint8_t b);
void print_byte_as_dec(uint8_t b);

// conversion functions, utils
uint32_t      str_to_uint32(const char* s);
uint64_t      str_to_uint64(const char* s);
uint_fast16_t hexstr_to_uint16(const char* data);
uint_fast8_t  str_starts_with(const char* str, const char* start);
char*         to_hexstr(const uint8_t* data, uint32_t len, char* out_buffer, uint32_t buffer_size);

void serial_send_packet(uint8_t* data, uint16_t len);
uint8_t serial_console(uint8_t rcvd_byte);



/*
 * G L O B A L S
 */

extern char debug_print_buffer[DEBUG_PRINT_MAX_LEN];


#endif /* __SERIAL_H__ */
