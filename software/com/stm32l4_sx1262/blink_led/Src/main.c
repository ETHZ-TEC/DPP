/*
 * Copyright (c) 2020, Swiss Federal Institute of Technology (ETH Zurich).
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

/* a simple blink LED example for the STM32L443 MCU (DPP2 LoRa Comboard on FlockLab 2) */

#include "main.h"


void clock_init(void);
void gpio_init(void);
void uart_init(void);
void uart_println(const char* str);
void delay(uint32_t loops);
void test_lpm(void);


static volatile uint32_t counter = 0;


int main(void)
{
  //test_lpm();

  /* initialize clocks, pins and peripherals */
  clock_init();
  gpio_init();
  uart_init();

  /* set output high */
  LL_GPIO_SetOutputPin(LED1_GPIO_PORT, LED1_PIN);

  while (1)
  {
    /* toggle the green and red LED */
    LL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
    LL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN);

    /* toggle all FlockLab tracing pins */
    LL_GPIO_TogglePin(FLOCKLAB_LED1_PORT, FLOCKLAB_LED1_PIN);
  #if !USE_SWO
    LL_GPIO_TogglePin(FLOCKLAB_LED2_PORT, FLOCKLAB_LED2_PIN);
  #endif /* USE_SWO */
  #if !USE_SWD
    LL_GPIO_TogglePin(FLOCKLAB_LED3_PORT, FLOCKLAB_LED3_PIN);
    LL_GPIO_TogglePin(FLOCKLAB_INT2_PORT, FLOCKLAB_INT2_PIN);
  #endif /* USE_SWD */
    LL_GPIO_TogglePin(FLOCKLAB_INT1_PORT, FLOCKLAB_INT1_PIN);

    uart_println("hello world!");
    counter++;

    delay(SystemCoreClock / 10);    /* wait ~1s */
  }
}


void test_lpm(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();

  /* configure all GPIOs for minimal current drain */

  /* enable clocks */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* apply a default config (ensures that all unused / unconnected pins are in analog mode) */
  GPIO_InitStruct.Pin  = GPIO_PIN_All;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* configure all pins that are connected to an external circuit as inputs with pull resistors where necessary */
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_11 | GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_12;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_13;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  GPIO_InitStruct.Pin  = GPIO_PIN_3;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /* disable clocks */
  __HAL_RCC_GPIOA_CLK_DISABLE();
  __HAL_RCC_GPIOB_CLK_DISABLE();
  __HAL_RCC_GPIOC_CLK_DISABLE();
  __HAL_RCC_GPIOH_CLK_DISABLE();

  /* enter low-power mode */
  MODIFY_REG(PWR->CR1, PWR_CR1_LPMS, LOW_POWER_MODE);
  SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));
  __WFI();
}


void gpio_init(void)
{
  LED1_GPIO_CLK_ENABLE();
  FLOCKLAB_GPIO_CLK_ENABLE();
  LL_GPIO_SetPinMode(LED1_GPIO_PORT, LED1_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(FLOCKLAB_LED1_PORT, FLOCKLAB_LED1_PIN, LL_GPIO_MODE_OUTPUT);
#if !USE_SWO
  /* do not configure the SWO pin as output if SWO is used to output debug information */
  LL_GPIO_SetPinMode(FLOCKLAB_LED2_PORT, FLOCKLAB_LED2_PIN, LL_GPIO_MODE_OUTPUT);
#endif /* USE_SWO */
#if !USE_SWD
  /* do not configure the SWDIO / SWDCLK pins as outputs if SWD debugging is to be used */
  LL_GPIO_SetPinMode(FLOCKLAB_LED3_PORT, FLOCKLAB_LED3_PIN, LL_GPIO_MODE_OUTPUT);
  LL_GPIO_SetPinMode(FLOCKLAB_INT2_PORT, FLOCKLAB_INT2_PIN, LL_GPIO_MODE_OUTPUT);
#endif /* USE_SWD */
  LL_GPIO_SetPinMode(FLOCKLAB_INT1_PORT, FLOCKLAB_INT1_PIN, LL_GPIO_MODE_OUTPUT);
}


/* configure USART1 module in UART mode with 115200 baud */
void uart_init(void)
{
  /* configure pins */
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_GPIO_SetPinMode(UART_RXD_PORT, UART_RXD_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetPinMode(UART_TXD_PORT, UART_TXD_PIN, LL_GPIO_MODE_ALTERNATE);
  LL_GPIO_SetAFPin_8_15(UART_RXD_PORT, UART_RXD_PIN, GPIO_AF7_USART1);
  LL_GPIO_SetAFPin_8_15(UART_TXD_PORT, UART_TXD_PIN, GPIO_AF7_USART1);
  LL_GPIO_SetPinSpeed(UART_RXD_PORT, UART_RXD_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinSpeed(UART_TXD_PORT, UART_TXD_PIN, LL_GPIO_SPEED_FREQ_HIGH);
  LL_GPIO_SetPinOutputType(UART_RXD_PORT, UART_RXD_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinOutputType(UART_TXD_PORT, UART_TXD_PIN, LL_GPIO_OUTPUT_PUSHPULL);
  LL_GPIO_SetPinPull(UART_RXD_PORT, UART_RXD_PIN, LL_GPIO_PULL_UP);
  LL_GPIO_SetPinPull(UART_TXD_PORT, UART_TXD_PIN, LL_GPIO_PULL_UP);

  /* enable clock and set clock source */
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_PCLK2);  /* = CPU_SPEED */

  /* configure module */
  LL_USART_Disable(USART1);
  LL_USART_ConfigAsyncMode(USART1);
  LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
  /* 8 data bit, 1 start bit, 1 stop bit, no parity */
  LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
  LL_USART_SetHWFlowCtrl(USART1, LL_USART_HWCONTROL_NONE);
  uint32_t mode = LL_USART_OVERSAMPLING_16;
  if (CPU_SPEED / UART_BAUDRATE < 16) {
    mode = LL_USART_OVERSAMPLING_8;
  }
  LL_USART_SetOverSampling(USART1, mode);
  LL_USART_SetBaudRate(USART1, SystemCoreClock, mode, UART_BAUDRATE);
  LL_USART_Enable(USART1);

  /* wait until the module is active */
  while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || (!(LL_USART_IsActiveFlag_REACK(USART1))));
}


void uart_println(const char* str)
{
  while (*str)
  {
    while (!LL_USART_IsActiveFlag_TXE(USART1));
    LL_USART_TransmitData8(USART1, *str);
    str++;
  }
  while (!LL_USART_IsActiveFlag_TXE(USART1));
  LL_USART_TransmitData8(USART1, '\r');
  while (!LL_USART_IsActiveFlag_TXE(USART1));
  LL_USART_TransmitData8(USART1, '\n');
}


void delay(volatile uint32_t loops)
{
  while (loops) loops--;
}


void clock_init(void)
{
  /* MSI configuration and activation */
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_4);
  LL_RCC_MSI_Enable();
  while (!LL_RCC_MSI_IsReady());

  /* Main PLL configuration and activation */
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_MSI, LL_RCC_PLLM_DIV_1, (uint32_t)40 * CPU_SPEED / 80000000, LL_RCC_PLLR_DIV_2);  // 40 = 80MHz, 24 = 48MHz
  LL_RCC_PLL_Enable();
  LL_RCC_PLL_EnableDomain_SYS();
  while (!LL_RCC_PLL_IsReady());

  /* Sysclk activation on the main PLL */
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

  /* Set APB1 & APB2 prescaler*/
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);

  LL_Init1msTick(CPU_SPEED);

  /* Update CMSIS variable (which can be updated also through SystemCoreClockUpdate function) */
  LL_SetSystemCoreClock(CPU_SPEED);
}


#ifdef  USE_FULL_ASSERT

void assert_failed(char *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

