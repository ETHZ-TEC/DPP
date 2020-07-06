/*
 * Copyright (c) 2018, Swiss Federal Institute of Technology (ETH Zurich).
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

#ifndef __MAIN_H
#define __MAIN_H


#define USE_SWD                     0         /* set to 1 to use SWD port (SWDIO and SWDCLK pins) for debugging */
#define USE_SWO                     0         /* set to 1 to use SWO pin for data tracing (DWT) */
#define CPU_SPEED                   80000000
#define UART_BAUDRATE               115200

#define LOW_POWER_MODE              PWR_CR1_LPMS_STOP2    /* other possible values: PWR_CR1_LPMS_STANDBY, PWR_CR1_LPMS_SHUTDOWN */

/* GREEN LED */
#define LED1_PIN                    GPIO_PIN_8
#define LED1_GPIO_PORT              GPIOB
/* RED LED */
#define LED2_PIN                    GPIO_PIN_9
#define LED2_GPIO_PORT              GPIOB
#define LED1_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOB_CLK_ENABLE()
#define LED1_GPIO_CLK_DISABLE()     __HAL_RCC_GPIOB_CLK_DISABLE()

#define FLOCKLAB_LED1_PIN           GPIO_PIN_3
#define FLOCKLAB_LED1_PORT          GPIOH
#define FLOCKLAB_LED2_PIN           GPIO_PIN_3
#define FLOCKLAB_LED2_PORT          GPIOB
#define FLOCKLAB_LED3_PIN           GPIO_PIN_13
#define FLOCKLAB_LED3_PORT          GPIOA
#define FLOCKLAB_INT1_PIN           GPIO_PIN_3
#define FLOCKLAB_INT1_PORT          GPIOA
#define FLOCKLAB_INT2_PIN           GPIO_PIN_14
#define FLOCKLAB_INT2_PORT          GPIOA
#define FLOCKLAB_GPIO_CLK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOH_CLK_ENABLE()
#define FLOCKLAB_GPIO_CLK_DISABLE() __HAL_RCC_GPIOA_CLK_DISABLE(); __HAL_RCC_GPIOB_CLK_DISABLE(); __HAL_RCC_GPIOH_CLK_DISABLE()

#define UART_RXD_PIN                GPIO_PIN_10
#define UART_RXD_PORT               GPIOA
#define UART_TXD_PIN                GPIO_PIN_9
#define UART_TXD_PORT               GPIOA
#define UART_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()


#include "stm32l4xx.h"
#include "stm32l4xx_ll_system.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32l4xx_ll_rcc.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_utils.h"
#include "stm32l4xx_ll_usart.h"
#include "stm32l4xx_hal.h"        /* includes hal config */


void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);


#endif /* __MAIN_H */
