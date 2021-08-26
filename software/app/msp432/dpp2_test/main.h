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
 * DPP2 Dev Board: Test program for the MSP432 (rev.C)
 */

#ifndef __MAIN_H__
#define __MAIN_H__


/*
 * A P P L I C A T I O N   C O N F I G
 */

#define CONSOLE_ENABLE      1
#define PERIPH_ENABLE       1     /* enable peripherals? (includes ADC) */
#define SDCARD_USE_FATFS    0     /* FatFS library or normal block-based access over SPI? Card must be formatted as FAT32 if FatFS is to be used. */
#define SDCARD_FORMAT       0     /* format SD card? can take up to 15 minutes! only works if SDCARD_USE_FATFS is enabled */
#define SDCARD_WRITE_BLOCK  0     /* write 1 block of dummy data to the SD card; this flag is 'don't care' if SDCARD_USE_FATFS is enabled */


/*
 * I N C L U D E S
 */

#include <stdint.h>
#include <string.h>

/* include all necessary files of this project */
#include "config.h"               /* hardware abstraction layer (pin mappings) */
#include "msp432_init.h"          /* initialization routines */
#include "sdcard.h"               /* SD Card driver/interface */
#include "bolt.h"                 /* BOLT driver */
#include "serial.h"               /* serial console */
#include "sensors/sensor.h"       /* I2C communication for sensors */


/*
 * M A C R O S
 */

#define PWRSW_ON(p)       PIN_CFG_OUT_I(p)    /* output low = on */
#define PWRSW_OFF(p)      PIN_CFG_IN_I(p)     /* high impedance = off */



/*
 * G L O B A L S
 */




#endif /* __MAIN_H__ */
