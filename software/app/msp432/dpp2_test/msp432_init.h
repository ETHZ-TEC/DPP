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
 * a lightweight driver lib for the DPP MSP432
 */


#ifndef __MSP432_INIT_H__
#define __MSP432_INIT_H__

/*
 * I N C L U D E S
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
//#include "msp.h"
#include "driverlib/MSP432P4xx/driverlib.h"      // use the TI driver lib
#include "config.h"


/*
 * D E F A U L T   C O N F I G
 */

#ifndef SVS_ENABLE
#define SVS_ENABLE      0
#endif /* SVS_ENABLE */

#ifndef DEBUG
#define DEBUG           1
#endif /* DEBUG */


/*
 * D E F I N I T I O N S
 */

#define PIN0    0
#define PIN1    1
#define PIN2    2
#define PIN3    3
#define PIN4    4
#define PIN5    5
#define PIN6    6
#define PIN7    7

#define PORT1   1
#define PORT2   2
#define PORT3   3
#define PORT4   4
#define PORT5   5
#define PORT6   6
#define PORT7   7
#define PORT8   8
#define PORT9   9
#define PORT10  10
#define PORTJ   11


/*
 * M A C R O S
 */

#define MAX(x, y)                       ( (x) > (y) ? (x) : (y) )
#define MIN(x, y)                       ( (x) < (y) ? (x) : (y) )
#define PIN_TO_BIT(pin)                 (1 << pin)
#define REGVAL32(x)                     (*((volatile uint32_t *)((uint32_t)x)))
#define REGVAL16(x)                     (*((volatile uint16_t *)((uint32_t)x)))
#define REGVAL8(x)                      (*((volatile uint8_t *)((uint32_t)x)))

/* note: there is a factor of 2 difference btw. the parameter and the actual
 * no. of delayed cycles */
#define WAIT(s)                         __delay_cycles(MCLK_SPEED / 3 * s)   //SysCtlDelay(MCLK_SPEED / 6 * s)
#define WAIT_MS(ms)                     __delay_cycles(MCLK_SPEED / 3000 * ms)   //SysCtlDelay(MCLK_SPEED / 6000 * ms)
#define WAIT_US(us)                     __delay_cycles(MCLK_SPEED / 3000000 * us)   //SysCtlDelay(MCLK_SPEED / 6000000 * us)  //__delay_cycles(MCLK_SPEED / 3000000 * us)


/* Note: all following macros ending with '_I' (immediate) can only be used
   when passing numbers directly (no defines or variables) */
#define PORT_XOR_I(port)                P##port##OUT ^= 0xff
#define PORT_SET_I(port)                P##port##OUT = 0xff
#define PORT_CLR_I(port)                P##port##OUT = 0x00
#define PORT_SEL_PRI_I(port)            { P##port##SEL0 = 0xff; P##port##SEL1 = 0x00; }
#define PORT_UNSEL_I(port)              { P##port##SEL0 = 0x00; P##port##SEL1 = 0x00; }
#define PORT_RES_EN_I(port)             P##port##REN = 0xff
#define PORT_CLR_IFG_I(port)            P##port##IFG = 0x00
#define PORT_CFG_OUT_I(port)            P##port##DIR = 0xff
#define PORT_CFG_IN_I(port)             P##port##DIR = 0x00
#define PIN_XOR_I(port, pin)            P##port##OUT ^= BIT##pin
#define PIN_SET_I(port, pin)            P##port##OUT |= BIT##pin
#define PIN_CLR_I(port, pin)            P##port##OUT &= ~BIT##pin
#define PIN_SEL_PRI_I(port, pin)        { P##port##SEL0 |= BIT##pin; P##port##SEL1 &= ~BIT##pin; }
#define PIN_UNSEL_I(port, pin)          { P##port##SEL0 &= ~BIT##pin; P##port##SEL1 &= ~BIT##pin; }
#define PIN_CFG_OUT_I(port, pin)        P##port##DIR |= BIT##pin
#define PIN_CFG_IN_I(port, pin)         P##port##DIR &= ~BIT##pin
#define PIN_CLR_IFG_I(port, pin)        P##port##IFG &= ~BIT##pin
#define PIN_RES_EN_I(port, pin)         P##port##REN |= BIT##pin
#define PIN_IES_RISING_I(port, pin)     P##port##IES &= ~BIT##pin
#define PIN_IES_FALLING_I(port, pin)    P##port##IES |= BIT##pin
#define PIN_IES_TOGGLE_I(port, pin)     P##port##IES ^= BIT##pin
#define PIN_INT_EN_I(port, pin)         P##port##IE |= BIT##pin
#define PIN_IFG_I(port, pin)            (P##port##IFG & (uint8_t)(BIT##pin))
#define PIN_GET_I(port, pin)            (P##port##IN & (uint8_t)(BIT##pin))
#define PIN_CFG_INT_I(port, pin, r)     { PIN_CFG_IN_I(port, pin); \
                                          if (r) { PIN_IES_RISING_I(port, pin); PIN_PULLDOWN_EN_I(port, pin); } \
                                          else { PIN_IES_FALLING_I(port, pin); PIN_PULLUP_EN_I(port, pin); } \
                                          PIN_CLR_IFG_I(port, pin); \
                                          PIN_INT_EN_I(port, pin); }
#define PIN_PULLUP_EN_I(port, pin)      { PIN_RES_EN_I(port, pin); PIN_SET_I(port, pin); }
#define PIN_PULLDOWN_EN_I(port, pin)    { PIN_RES_EN_I(port, pin); PIN_CLR_I(port, pin); }
#define PIN_MAP_I(port, pin, map)       { PMAPKEYID = PMAP_KEYID_VAL; \
                                          PMAPCTL |= PMAPRECFG; \
                                          *((volatile uint8_t*)&P##port##MAP01 + pin) = map; \
                                          PMAPKEYID = 0; }

#define PIN_XOR(p)                      PIN_XOR_I(p)                // toggle output bit/level
#define PIN_SET(p)                      PIN_SET_I(p)                // set output high
#define PIN_CLR(p)                      PIN_CLR_I(p)                // clear (set output low)
#define PIN_SEL_PRI(p)                  PIN_SEL_PRI_I(p)            // primary module function
#define PIN_UNSEL(p)                    PIN_UNSEL_I(p)              // unselect (configure port function)
#define PIN_CFG_OUT(p)                  PIN_CFG_OUT_I(p)            // configure pin as output
#define PIN_CFG_IN(p)                   PIN_CFG_IN_I(p)             // configure pin as input
#define PIN_CLR_IFG(p)                  PIN_CLR_IFG_I(p)            // clear interrupt flag
#define PIN_IES_RISING(p)               { PIN_IES_RISING_I(p); PIN_CLR_IFG_I(p); }    // interrupt edge select (always clear the IFG when changing PxIES)
#define PIN_IES_FALLING(p)              { PIN_IES_FALLING_I(p); PIN_CLR_IFG_I(p); }   // interrupt edge select
#define PIN_IES_TOGGLE(p)               { PIN_IES_TOGGLE_I(p); PIN_CLR_IFG_I(p); }    // interrupt edge toggle
#define PIN_INT_EN(p)                   PIN_INT_EN_I(p)             // enable port interrupt
#define PIN_CFG_INT(p, rising)          PIN_CFG_INT_I(p, rising)    // configure & enable port interrupt
#define PIN_IFG(p)                      PIN_IFG_I(p)                // get interrupt flag
#define PIN_GET(p)                      PIN_GET_I(p)                // get input bit/level
#define PIN_PULLUP_EN(p)                PIN_PULLUP_EN_I(p)          // enable pullup resistor (for input pins only)
#define PIN_PULLDOWN_EN(p)              PIN_PULLDOWN_EN_I(p)        // enable pulldown resistor (for input pins only)
#define PIN_MAP(p, map)                 PIN_MAP_I(p, map)

/* SPI */
#define SPI_ENABLE(module)              ((module)->CTLW0 &= ~UCSWRST)         //( UC##module##CTLW0 &= ~UCSWRST )
#define SPI_IS_ENABLED(module)          (((module)->CTLW0 & UCSWRST) == 0)    //( !(UC##module##CTLW0 & UCSWRST) )
#define SPI_DISABLE(module)             ((module)->CTLW0 |= UCSWRST)          //( UC##module##CTLW0 |= UCSWRST )
#define SPI_WAIT_TXE(module)            while (!((module)->IFG & UCTXIFG))    //while (!(UC##module##IFG & UCTXIFG))
#define SPI_WAIT_RXNE(module)           while (!((module)->IFG & UCRXIFG))    //while (!(UC##module##IFG & UCRXIFG))
/* important: in Rev.B devices, BUSY bit is NOT usable for USCIA module! */
#define SPI_WAIT_BUSY(module)           while ((module)->STATW & UCBUSY)  //while (UC##module##STATW & UCBUSY)
#define SPI_IS_BUSY(module)             ((module)->STATW & UCBUSY)        //( UC##module##STATW & UCBUSY )
#define SPI_WRITE_BYTE(module, b)       SPI_WAIT_TXE(module); (module)->TXBUF = (b)
#define SPI_READ_BYTE(module, b)        SPI_WRITE_BYTE(module, SPI_DUMMY_BYTE); SPI_WAIT_RXNE(module); (b) = (module)->RXBUF   //{ SPI_WAIT_RXNE(module); (b) = UC##module##RXBUF; }
#define SPI_CLR_RXIFG(module)           (module)->IFG &= (~UCRXIFG)       // or do a read from the RX buffer: (void)((module)->RXBUF)   or   ((void)UC##module##RXBUF)

/* I2C */
#define I2C_WAIT_TXE(module)            while(!((module)->IFG & UCTXIFG0))
#define I2C_WRITE_BYTE(module, b)       I2C_WAIT_TXE(module); (module)->TXBUF = (b)
#define I2C_WAIT_RXNE(module)           while(!((module)->IFG & UCRXIFG0))
#define I2C_WAIT_START_CLR(module)      while((module)->CTLW0 & UCTXSTT)    // note: flag is cleared before the ACK/NACK has been received!
#define I2C_WAIT_STOP_CLR(module)       while((module)->CTLW0 & UCTXSTP)
#define I2C_WAIT_BUSY(module)           while ((module)->STATW & UCBUSY)
#define I2C_READ_BYTE(module, b)        I2C_WAIT_RXNE(module); (b) = (module)->RXBUF
#define I2C_CLR_RXBUF(module)           (void)(module)->RXBUF
#define I2C_ENABLE(module)              ((module)->CTLW0 &= ~UCSWRST)
#define I2C_DISABLE(module)             ((module)->CTLW0 |= UCSWRST)

/* timer */
#define TIMESTAMP                       ~(TIMER32_1->VALUE)       // overflows every seconds (use timer32_now() instead to use the sw extension)
#define TIMESTAMP_MS                    (TIMESTAMP / (TIMER32_SPEED / 1000))
#define TIMER_A0_START                  TA0CTL = (TIMER_A0_SRC | MC__UP | TACLR)
#define TIMER_A1_START                  TA1CTL = (TIMER_A1_SRC | MC__UP | TACLR)
#define TIMER_A0_STOP                   TA0CTL &= ~MC_3
#define TIMER_A1_STOP                   TA1CTL &= ~MC_3
#define TIMER32_START                   TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE
#define TIMER32_STOP                    TIMER32_1->CONTROL &= ~TIMER32_CONTROL_ENABLE

/* RTC */
#define RTC_STOP                        RTC_C->CTL0 = (RTC_C->CTL0 & ~RTC_C_CTL0_KEY_MASK) | RTCKEY; RTC_C->CTL13 |= RTCHOLD;  RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK
#define RTC_START                       RTC_C->CTL0 = (RTC_C->CTL0 & ~RTC_C_CTL0_KEY_MASK) | RTCKEY; RTC_C->CTL13 &= ~RTCHOLD; RTC_C->CTL0 &= ~RTC_C_CTL0_KEY_MASK

/* PWM timer */
#define PWM_TA0_CFG(pin, ccr)           { pwm_init_ta0(ccr); PIN_MAP_I(pin, PM_TA0CCR##ccr##A); PIN_SEL_PRI_I(pin); }
#define PWM_TA1_CFG(pin, ccr)           { pwm_init_ta1(ccr); PIN_MAP_I(pin, PM_TA1CCR##ccr##A); PIN_SEL_PRI_I(pin); }
#define PWM_TA0_SET(ccr, val)           TA0CCR##ccr = (val)   // or use: TIMER_A0->CCR[ccr] = (val)
#define PWM_TA1_SET(ccr, val)           TA1CTL &= ~MC_M; TA1CCR##ccr = (val); TA1CTL |= MC__UP

/* interrupts */
#define INTERRUPT_MASTER_ENABLED        (CPU_primask() == 0)
#define INTERRUPT_ENABLE(i)             NVIC->ISER[(i - 16) / 32] = (1 << (((i) - 16) & 31))     // i <= 56 (e.g. INT_PORT3)

/* watchdog */
#define WATCHDOG_STOP                   WDTCTL = (WDTPW | WDTHOLD)
#define WATCHDOG_START                  WDTCTL = ((WDTCTL & ~(WDTHOLD)) + WDTPW)
#define WATCHDOG_RESET                  WDTCTL = ((WDTCTL | WDTCNTCL) + WDTPW)

/* power management */
#define PM_ENABLE_SLEEP_ON_ISR_EXIT     (SCB->SCR |= SCB_SCR_SLEEPONEXIT)
#define PM_DISABLE_SLEEP_ON_ISR_EXIT    (SCB->SCR &= ~SCB_SCR_SLEEPONEXIT)
#define PM_ENABLE_SRAM_BANKS            { __delay_cycles(10000); SYSCTL_SRAM_BANKEN |= SYSCTL_SRAM_BANKEN_BNK7_EN; }  // enable ALL the memory banks
#define PM_ENABLE_SRAM_STACK_RETENTION  (SYSCTL->SRAM_BANKRET |= SYSCTL_SRAM_BANKRET_BNK7_RET)
#define PM_ENABLE_FULL_SRAM_RETENTION   { __delay_cycles(10000); SYSCTL_SRAM_BANKRET |= (SYSCTL_SRAM_BANKRET_BNK1_RET | SYSCTL_SRAM_BANKRET_BNK2_RET | SYSCTL_SRAM_BANKRET_BNK3_RET | SYSCTL_SRAM_BANKRET_BNK4_RET | SYSCTL_SRAM_BANKRET_BNK5_RET | SYSCTL_SRAM_BANKRET_BNK6_RET | SYSCTL_SRAM_BANKRET_BNK7_RET); }
#define PM_ENTER_LPM0                   { SCB->SCR &= ~(SCB_SCR_SLEEPDEEP); __sleep(); __no_operation(); }
#define PM_ENTER_LPM3                   { SCB->SCR |= (SCB_SCR_SLEEPDEEP); __sleep(); __no_operation(); }
#define PM_ENTER_LPM4                   { RTC_STOP; WATCHDOG_STOP; PM_ENTER_LPM3; }    // hold RTC and Watchdog, then enter deep sleep
#define PM_ENTER_LPM45                  { SCB->SCR |= (1 << 2); \
                                          PCM->CTL0 = (0x695A0000 | PCM_CTL0_LPMR_12 | (PCM->CTL0 & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_LPMR_MASK))); \
                                          __asm("    wfi\n"); \
                                          __no_operation(); }
#define PM_DISABLE_SVS                  { PSSKEY = PSS_KEY_KEY_VAL; PSSCTL0 |= SVSMHOFF | SVSLOFF; PSSKEY = 0; }
#define PM_ENABLE_RUDE_MODE             { PCM->CTL1 = 0xa5960000 | PCM_CTL1_FORCE_LPM_ENTRY; }  // allows to enter LPM3 without waiting for peripherals

/* SVS */
#define SVS_DISABLE                     PSS->KEY = PSS_KEY_KEY_VAL; PSS->CTL0 |= (BIT0 | BIT8); PSS->KEY = 0

/* reset */
#define RESET_TRIGGER_HARD              (RSTCTL->RESET_REQ = (RESET_KEY | 0x00000002))
#define RESET_TRIGGER_SOFT              (RSTCTL->RESET_REQ = (RESET_KEY | 0x00000001))

/* flash */
#define FLASH_WAIT_STATES               ((FLCTL->BANK0_RDCTL & 0x0000f000) >> 12)

/* led */
#define LED_ON(pin)                     PIN_SET_I(pin)
#define LED_OFF(pin)                    PIN_CLR_I(pin)
#define LED_TOGGLE(pin)                 PIN_XOR_I(pin)

/* UART */
#define UART_WAIT_BUSY                  while (UART_MODULE->STATW & UCBUSY)
#define UART_WAIT_RXNE                  while (!(UART_MODULE->IFG & UCRXIFG))
#define UART_WAIT_TXE                   while (!(UART_MODULE->IFG & UCTXIFG))
#define UART_WRITE_BYTE(b)              UART_WAIT_BUSY; UART_MODULE->TXBUF = (b)
#define UART_READ_BYTE(b)               UART_WAIT_BUSY; (b) = UART_MODULE->RXBUF
#define UART_RXBUF                      UART_MODULE->RXBUF
#define UART_DISABLE                    UART_MODULE->CTLW0 |= UCSWRST
#define UART_ENABLE                     UART_MODULE->CTLW0 &= ~UCSWRST

/* ADC */
#define ADC_WAIT_BUSY                   while (ADC14->CTL0 & ADC14_CTL0_BUSY)
#define ADC_TOGGLE_SC                   ADC14->CTL0 ^= ADC14_CTL0_SC          // toggle sample conversion
#define ADC_TRIGGER_SC                  ADC14->CTL0 |= ADC14_CTL0_SC
#define ADC_DISABLE                     ADC14->CTL0 &= ~(ADC14_CTL0_SHP | ADC14_CTL0_ON | ADC14_CTL0_ENC)
#define ADC_ENABLE                      ADC14->CTL0 |= (ADC14_CTL0_SHP | ADC14_CTL0_ON | ADC14_CTL0_ENC)
#define ADC_START                       ADC14->CTL0 |= (ADC14_CTL0_ENC | ADC14_CTL0_SC)
#define ADC_GET_VALUE(res)              ADC_TRIGGER_SC; while (!(ADC14->IFGR0 & ADC14_IFGR0_IFG0)); (res) = ADC14->MEM[0]
#define ADC_CH_TEMPSENSOR               22                            // it is channel 22 on 100-pin and 64-pin package
#define ADC_CONV_TO_MV(raw_val)         (raw_val * 2500 / 16383)      // convert a raw ADC value to millivolt (14-bit ADC with int. 2.5V REF buffer)

/* Systick (STCSR register) */
#define SYSTICK_INT_ENABLE              SysTick->CTRL |= (0x00000002)
#define SYSTICK_INT_DISABLE             SysTick->CTRL &= ~(0x00000002)
#define SYSTICK_START                   SysTick->CTRL |= (0x00000001); SYSTICK_INT_ENABLE
#define SYSTICK_STOP                    SYSTICK_INT_DISABLE; SysTick->CTRL &= ~(0x00000001)
#define SYSTICK_SET_PERIOD(p)           SysTick->LOAD = ((p) - 1)
#if SYSTICK_PERIOD > 16777216
#error "invalid SYSTICK_PERIOD"
#endif

/* REF */
#define REF_TEMP_SENSOR_ENABLE          REF_A->CTL0 &= ~REF_A_CTL0_TCOFF
#define REF_TEMP_SENSOR_DISABLE         REF_A->CTL0 |= REF_A_CTL0_TCOFF
#define REF_SET_2V5                     REF_A->CTL0 |= REF_A_VREF2_5V
#ifdef DPP_REV1
#define REF_WAIT_BUSY                   // doesn't work on rev B devices
#else /* DPP_REV1 */
#define REF_WAIT_BUSY                   while (REFCTL0 & REFGENBUSY)
#endif /* DPP_REV1 */
#define REF_ENABLE                      REF_WAIT_BUSY; REFCTL0 |= REFON; while(!(REFCTL0 & REFGENRDY))
#define REF_DISABLE                     REF_WAIT_BUSY; REFCTL0 &= ~REFON

/* misc */
#ifndef SCB_SCR_SLEEPDEEP
#define SCB_SCR_SLEEPDEEP               (0x00000004)
#endif /* SCB_SCR_SLEEPDEEP */
#ifndef SCB_SCR_SLEEPONEXIT
#define SCB_SCR_SLEEPONEXIT             (0x00000002)
#endif /* SCB_SCR_SLEEPONEXIT */


#ifndef __eint
__attribute__( ( always_inline ) ) static inline void __eint(void)
{
__asm(" cpsie i");
}
__attribute__( ( always_inline ) ) static inline void __dint(void)
{
__asm(" cpsid i");
}
#endif /* __enable_interrupts */


/*
 * G L O B A L S
 */

extern void SysCtlDelay(uint32_t ms);


/*
 * F U N C T I O N   P R O T O T Y P E S
 */

/* misc */
void        uart_print(const char* str);
void        uart_println(const char* str);
uint32_t    flash_code_size(void);
const char* rst_src(void);

/* core */
void msp432_init(void);
void gpio_init(void);

/* peripherals */
void uart_init(void);
void spi_b_init(EUSCI_B_SPI_Type* spi_module, uint32_t sck_speed, uint_fast8_t cpol, uint_fast8_t cpha);
void spi_a_init(EUSCI_A_SPI_Type* spi_module, uint32_t sck_speed, uint_fast8_t cpol, uint_fast8_t cpha);
void spi_b_set_speed(EUSCI_B_SPI_Type* spi_module, uint32_t sck_speed);
void spi_a_set_speed(EUSCI_A_SPI_Type* spi_module, uint32_t sck_speed);
void spi_b_read(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t* out_data);
void spi_a_read(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t* out_data);
void spi_b_write(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, const uint8_t* data);
void spi_a_write(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, const uint8_t* data);
void spi_b_write_const(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t fill_value);
void spi_a_write_const(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t fill_value);
void i2c_init(EUSCI_B_Type* i2c_module, uint32_t scl_speed);
uint_fast8_t i2c_read(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint16_t cmd, uint32_t num_bytes, uint8_t* out_data);
uint_fast8_t i2c_read_cmd8(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint8_t cmd, uint32_t num_bytes, uint8_t* out_data);
uint_fast8_t i2c_write(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint16_t cmd);
uint_fast8_t i2c_write_cmd8(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint8_t cmd);
void adc_init_simple(uint_fast8_t adc_ch);
void adc_init_tempsensor(void);
void adc_init_multi_ch(uint_fast8_t adc_ch1, uint_fast8_t adc_ch2);

/* timers */
uint_fast8_t  rtc_set_time(const RTC_C_Calendar* rtc_cal_time);
const char*   rtc_get_time(uint_fast8_t with_date);
void          rtc_set_alarm(uint_fast8_t hour, uint_fast8_t minute, uint_fast8_t day_of_week, uint_fast8_t day_of_month);
void          systick_enable(void);
void          timer32_init(void);
uint64_t      timer32_now(void);
void          timer32_update(void);
void          pwm_init_ta0(uint_fast8_t ccr);
void          pwm_init_ta1(uint_fast8_t ccr);

/* obsolete TI driverlib functions */
void PSS_disableLowSide(void);


#endif /* __MSP432_INIT_H__ */
