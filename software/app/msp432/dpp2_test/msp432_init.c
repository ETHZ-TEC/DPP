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
 * Important Notes:
 * - Do not set PJSEL0 to 0, otherwise a factory reset will be necessary to reprogram the core.
 * - don't use Interrupt_enableSleepOnIsrExit() on Rev. B models, use Interrupt_enableSleepOnIsrExit() instead.
 * - Do not use PCM_gotoLPM3() or PCM_gotoLPM3(), both don't work. Use PM_ENTER_LPM3 instead.
 * - SRAM retention for banks 2 - 8 must be enabled before entering LPM3 or deeper to prevent stack corruption.
 * - SRAM retention is not guaranteed in LPM3 once the reset line was pulled low. A power cycle is then required to restore SRAM retention. (see Errata SRAM2)
 * - use the  prefix for all API (driver lib) calls, this makes switching between the ROM and Flash version of the driver library much easier
 * - there seems to be an error in the __delay_cycles() intrinsic: it takes twice as long as expected
 * - temperature sensor calibration values are not populated in the pre-release version of the MSP432 (Rev. B)
 * - PSS = power supply system, PCM = power control manager, SVS = supply voltage supervisor
 */

#include "msp432_init.h"


/*
 * G L O B A L S
 */

static uint32_t timer32_sw_ext = 0;
uint64_t        ta0_sw_ext = 0;


/*
 * H E L P E R   functions
 */

__asm("    .sect \".text:SysCtlDelay\"\n"
      "    .clink\n"
      "    .thumbfunc SysCtlDelay\n"
      "    .thumb\n"
      "    .global SysCtlDelay\n"
      "SysCtlDelay:\n"
      "    subs r0, #1\n"
      "    bne.n SysCtlDelay\n"
      "    bx lr\n");


void uart_print(const char* str)
{
  while (*str)
  {
    UART_WRITE_BYTE(*str);
    str++;
  }
}


void uart_println(const char* str)
{
  while (*str)
  {
    UART_WRITE_BYTE(*str);
    str++;
  }
  UART_WRITE_BYTE('\r');
  UART_WRITE_BYTE('\n');
}


const char* rst_src(void)
{
  /*
   * reset classification:
   * - there are 4 reset classes: class 0 to class 3   *
   *   class 0: power on/off reset (POR), e.g. voltage exception or LPMx.5 exit
   *   class 1: reboot reset, same as POR, but does not reset the debug components of the CPU
   *   class 2: hard reset, restart without reboot (e.g. due to catastrophic event), SRAM values are retained, registers and bus logic is reset
   *   class 3: soft reset, aborts all bus transactions in the M4 and resets WDT, but system-level bus transactions, peripheral config and SRAM are maintained
   * - a class x reset always causes a class x+1 reset (e.g. class 0 causes class 1, 2 and 3 resets)
   * - RSTCTL_RESET_REQ can be used to trigger hard (RESET_KEY | HARD_REQ) and soft resets (RESET_KEY | SOFT_REQ)
   */

  if (RSTCTL->PCMRESET_STAT & (RSTCTL_PCMRESET_STAT_LPM35 | RSTCTL_PCMRESET_STAT_LPM45))
  {
    return "LPMx5";
  // class 0
  } else if (RSTCTL->REBOOTRESET_STAT)
  {
    return "SYSCTL";
  // class 0: reset pin
  } else if (RSTCTL->PINRESET_STAT & RSTCTL_PINRESET_STAT_RSTNMI)
  {
    return "nRST";
  // class 2
  } else if (RSTCTL->PSSRESET_STAT & (RSTCTL_PSSRESET_STAT_VCCDET | RSTCTL_PSSRESET_STAT_BGREF | RSTCTL_PSSRESET_STAT_SVSMH | RSTCTL_PSSRESET_STAT_SVSL))
  {
    return "PSS";
  } else if (RSTCTL->HARDRESET_STAT)
  {
    return "HARD";
  // class 3: soft reset
  } else if (RSTCTL->SOFTRESET_STAT)
  {
    return "SOFT";
  }
  return "UNKNOWN";
}


uint32_t flash_code_size(void)
{
  // 32-bit word alignment
  uint32_t *curr_addr = (uint32_t *)((uint32_t)FLASH_MEMORY_START + FLASH_MEMORY_SIZE - 4);
  // assumptions: each unused word in the flash memory has the value 0xffff and flash programming starts at the smallest address
  while(((uint32_t)curr_addr > FLASH_MEMORY_START) && (*curr_addr == 0xffffffff)) { curr_addr--; }
  return (uint32_t)curr_addr - FLASH_MEMORY_START + 4;
}


void enter_lpm45(void)
{
  // configure pins for minimal current drain
  P1DIR  = 0xff;
  P1OUT  = 0;
  P1SEL0 = 0; P1SEL1 = 0;
  P6DIR  = 0xff;
  P6OUT  = 0;
  P6SEL0 = 0; P6SEL1 = 0;
  P7DIR  = 0xff;
  P7OUT  = 0;
  P7SEL0 = 0; P7SEL1 = 0;
  PJDIR  = 0xff & ~(BIT4 | BIT5);
  PJOUT  = 0;
  PJSEL0 &= (BIT4 | BIT5);
  PJSEL1 &= (BIT4 | BIT5);
  // disable SVS and enter the low-power mode
  SVS_DISABLE;
  PM_ENTER_LPM45;
}


// note: lpm3 is same as lpm4 if RTC and Watchdog are disabled
void enter_lpm3(void)
{
  // configure pins for minimal current drain
  P1DIR  = 0xff;
  P1OUT  = 0;
  P1SEL0 = 0; P1SEL1 = 0;
  P6DIR  = 0xff;
  P6OUT  = 0;
  P6SEL0 = 0; P6SEL1 = 0;
  P7DIR  = 0xff;
  P7OUT  = 0;
  P7SEL0 = 0; P7SEL1 = 0;
  PJDIR  = 0xff & ~(BIT0 | BIT1 | BIT4 | BIT5);
  PJOUT  = 0;
  PJSEL0 &= (BIT0 | BIT1 | BIT4 | BIT5);
  PJSEL1 &= (BIT0 | BIT1 | BIT4 | BIT5);
  PCM->CTL1 &= ~(PCM_CTL1_LOCKLPM5 | PCM_CTL1_LOCKBKUP);
  PM_ENABLE_SRAM_STACK_RETENTION;
  SVS_DISABLE;
  // enable LFXT and use it as clock source for ACLK and BCLK
  CS_setExternalClockSourceFrequency(LFXTCLK_SPEED, HFXTCLK_SPEED);
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION);
  CS_startLFXT(CS_LFXT_DRIVE0);
  CS_initClockSignal(CS_ACLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);
  CS_initClockSignal(CS_BCLK, CS_LFXTCLK_SELECT, CS_CLOCK_DIVIDER_1);

  PM_ENTER_LPM3;
}


/*
 * the following functions are from the TI driverlib; they have been removed from their lib as of v3 due to future hardware changes
 */


void PSS_setLowSidePerformanceMode(uint_fast8_t ui8PowerMode)
{
#define SVSLLP_OFS    9

  PSS->KEY = PSS_KEY_VALUE;
  if (ui8PowerMode == PSS_FULL_PERFORMANCE_MODE)
  {
    BITBAND_PERI(PSS->CTL0, SVSLLP_OFS) = 0;
  } else
  {
    BITBAND_PERI(PSS->CTL0, SVSLLP_OFS) = 1;
  }
  PSS->KEY = 0;
}


void PSS_disableLowSide(void)
{
#define SVSLOFF_OFS   8
  PSS->KEY = PSS_KEY_VALUE;
  BITBAND_PERI(PSS->CTL0, SVSLOFF_OFS) = 1;
  PSS->KEY = 0;
}


/*
 * --- C O R E ---
 */


void gpio_init(void)
{
  /*
   * notes:
   * - default for unused pins is output low
   * - for analog mode, set bits in SEL0 and SEL1 to 1
   */
  P1DIR  = 0xff & ~(BIT2 | BIT3 | BIT5 | BIT6 | BIT7);
  P1OUT  = 0;
  P1SEL0 = (BIT2 | BIT3 | BIT5 | BIT6 | BIT7); P1SEL1 = 0;
  P1REN  = 0; P1IFG  = 0;

  P2DIR  = 0xff;
  P2OUT  = 0;
  P2SEL0 = 0; P2SEL1 = 0;
  P2REN  = 0; P2IFG  = 0;

  P3DIR  = 0xff;
  P3OUT  = 0;
  P3SEL0 = 0; P3SEL1 = 0;
  P3REN  = 0; P3IFG  = 0;

  P4DIR  = 0xff & ~(BIT3 | BIT4 | BIT7);
  P4OUT  = 0;
  P4SEL0 = 0; P4SEL1 = 0;
  P4REN  = 0; P4IFG  = 0;

  P5DIR  = 0xff & ~(BIT5);
  P5OUT  = 0;
  P5SEL0 = BIT5; P5SEL1 = BIT5;
  P5REN  = 0; P5IFG  = 0;

  P6DIR  = 0xff & ~(BIT6 | BIT7);
  P6OUT  = 0;
  P6SEL0 = 0; P6SEL1 = (BIT6 | BIT7);
  P6REN  = 0; P6IFG  = 0;

  P7DIR  = 0xff & ~(BIT0 | BIT1 | BIT2);
  P7OUT  = 0;
  P7SEL0 = (BIT0 | BIT1 | BIT2); P7SEL1 = 0;
  P7REN  = 0;

  P8DIR  = 0xff & ~(BIT0 | BIT1);
  P8OUT  = 0;
  P8SEL0 = 0; P8SEL1 = 0;
  P8REN  = 0;

  P9DIR  = 0xff;
  P9OUT  = 0;
  P9SEL0 = 0; P9SEL1 = 0;
  P9REN  = 0;

  P10DIR  = 0xff;
  P10OUT  = 0;
  P10SEL0 = 0; P10SEL1 = 0;
  P10REN  = 0;

  PJDIR   = 0xff & ~(BIT4 | BIT5);
  PJOUT   = 0;
  PJSEL0 &= (BIT4 | BIT5);
  PJSEL1 &= (BIT4 | BIT5);

  /* custom mapping for pins on ports 2, 3 and 7 */
  PIN_MAP(SDCARD_SPI_SCK,  PM_UCA1CLK);
  PIN_MAP(SDCARD_SPI_MOSI, PM_UCA1SIMO);
  PIN_MAP(SDCARD_SPI_MISO, PM_UCA1SOMI);
}


// power control manager
void pcm_init(void)
{
  /*
   * Power Management
   *
   * There is an active mode (run mode) and there are 5 different low-power modes: LPM0, LPM3, LPM4, LPM3.5 and LPM4.5
   * AM offers 6 different modes of operation (power modes): PCM_AM_LDO_VCORE0, PCM_AM_LDO_VCORE1, PCM_AM_DCDC_VCORE0, PCM_AM_DCDC_VCORE1, PCM_AM_LF_VCORE0, PCM_AM_LF_VCORE1
   * LPM0 offers 6 different modes of operation: PCM_LPM0_LDO_VCORE0, PCM_LPM0_LDO_VCORE1, PCM_LPM0_DCDC_VCORE0, PCM_LPM0_DCDC_VCORE1, PCM_LPM0_LF_VCORE0, PCM_LPM0_LF_VCORE1
   * The other power modes only support one mode of operation: LPM3_LDO, LPM4_LDO, LPM35_LDO, LPM45_LDO_VCORE_OFF
   *
   * - After a reset, the device enters AM_LDO_VCORE0.
   * - Automatic clock requests: A peripheral module requests its clock source if required, regardless of the current power mode!
   *   If the global enable bit is cleared (e.g. ACLK_EN), conditional clock requests are ignored. Unconditional requests in contrary cannot be disabled, i.e. they always receive the clock (e.g. the watchdog timer).
   * - Important: DC-DC and LF mode are only supported in LPM0 and AM!
   * - Terminology used in the driver lib: power mode (LDO, DCDC, LF) and power state (e.g. AM_LDO_VCORE0)
   * - In principle, VCore could be operated at level 1 in LPM3 and 4, but it wouldn't make sense.
   *   -> don't forget to adjust VCore before entering LPM3 since VCore in LPM3 is kept the same as in AM
   * - In LPM3 ..
   *   .. LDO based operating modes at core level 0 or 1
   *   .. flash memory is disabled, so are all other peripherals but the RTC and WDT
   *   .. only 32 kHz clock can be used
   * - LPM4 is basically the same as LPM3 with all clocks off. Thus, LPM4 can be entered by disabling RTC and WDT before calling PCM_gotoLPM3().
   * - In LPM3.5 ..
   *   .. all peripherals except for the RTC and WDT are disabled (RTC and WDT can be optionally enabled)
   *   .. the core voltage is brought down to level 0
   *   .. peripheral register data is not retained, but bank 0 of the SRAM and I/O pin states are
   *   .. SVSMH is always in supervisor mode (issue a POR), monitoring mode (= trigger an interrupt) is not available
   * - In LPM4.5 ..
   *   .. all peripherals and clock sources are powered down and the internal voltage regulator is switched off
   *   .. no retention for SRAM and register data, only I/O pin states are retained
   *   .. SVSL is not available, but SVSMH is (enabling SVSMH will result in a faster wake-up time, but also higher current drain)
   *   .. SVSMH is always in supervisor mode (issue a POR), monitor mode (= trigger an interrupt) is not available
   * - LPM0 is referred to as sleep mode, LPM3 as deep sleep and LPM3.5 and 4.5 as stop or shut down modes
   * - Make sure all power state transitions are valid! Obviously, power mode transitions can only be performed in active mode. Valid AM transitions are:
   *   AM_LDO_VCORE0 (initial state) <-> AM_LDO_VCORE1
   *   AM_DCDC_VCORE1 <-> AM_LDO_VCORE1 <-> AM_LF_VCORE1
   *   AM_DCDC_VCORE0 <-> AM_LDO_VCORE0 <-> AM_LF_VCORE0
   *   AM_LDO_VCOREx <-> LPM3/LPM4
   * - SVS is enabled by default (SVSL is for Vcore and SVSMH for Vcc). The supervisors detect when a voltage falls under a specific threshold and trigger a reset event
   * - There is a full-performance (faster response times) and a low-power mode for both SVSs.
   * - If SVS is enabled, full-performance mode is forced by the PCM in AM and LPM0, regardless of the SVSHMLP and SVSLLP bits.
   * - The SVSMH can be configure in monitor mode such that an ISR is triggered on the detection of a voltage drop. The application can then configure the I/Os into a defined state before the device is shutdown.
   *
   */

  // set power mode (or state) if necessary
  //PCM_setPowerMode(PCM_LF_MODE);      // low frequency mode (LF mode supports frequencies up to 128 kHz)
  //PCM_setPowerMode(PCM_LDO_MODE);       // use the internal LDO to supply the core voltage
  //PCM_setPowerMode(PCM_DCDC_MODE);      // use the internal DC-DC converter to supply the core voltage
  //PCM_setPowerState(PCM_AM_DCDC_VCORE0);
  //PCM_setPowerState(PCM_AM_LF_VCORE0);    // suitable if e.g. MCLK fed by REFO

  // enable rude mode (allows transition into LPM even when a peripheral is still using a clock source)
  //PCM_enableRudeMode();

  // clear LPM bits to unlock GPIO config
  PCM->CTL1 &= ~(PCM_CTL1_LOCKLPM5 | PCM_CTL1_LOCKBKUP);

  // increase core voltage level if necessary
  if (MCLK_SPEED > 24000000)
  {
    PCM_setCoreVoltageLevel(PCM_VCORE1);    // set VCore to 1 to support a frequency of 48 MHz (without timeout, waits until the new voltage has been established)
  } // else: VCORE0 (default setting after reset)

  // enable SRAM retention
  PM_ENABLE_SRAM_STACK_RETENTION;

  // configure the supply voltage supervisor
#if SVS_LOW_PERF
  // set low performance mode to save power in the low-power modes
  PSS_setHighSidePerformanceMode(0);
  PSS_setLowSidePerformanceMode(0);
#endif /* SVS_LOW_PERF */
#if !SVS_ENABLE
  PSS_disableHighSide();  // SVSHM
  PSS_disableLowSide();  // SVSHM
#endif /* !SVS_ENABLE */
}


void flash_init(void)
{
  /*
   * Flash Memory:
   * - the two banks 0 and 1 (each 128kB) can be accessed 'simultaneously', i.e. code execution
   *   from one bank can continue while program/erase is ongoing on the other bank
   * - wait states must be configured appropriately before changing the MCLK frequency, otherwise results are non-deterministic
   * - there are special wait states for other than 'normal' read modes (see p.18 in the datasheet for details)
   */

  // set the flash wait states for normal read mode based on the selected MCLK frequency (assume VCORE0 for up to 24 MHz)
  uint_fast8_t wait_states = 0;
  if (MCLK_SPEED > 24000000)    // VCORE1 mode
  {
    if (MCLK_SPEED > 32000000)
    {
      wait_states = 2;
    } else {
      wait_states = 1;
    }
  } else if (MCLK_SPEED > 12000000)
  {
    wait_states = 1;      // VCORE0
  }
  FlashCtl_setWaitState(FLASH_BANK0, wait_states);    // set flash memory wait states
  FlashCtl_setWaitState(FLASH_BANK1, wait_states);
}


/* starts the external low-frequency crystal and assigns it as the clock source for ACLK and BCLK */
void clock_start_lfxt(void)
{
  /* configure PJ.0 & PJ.1 for XT1 */
  PJSEL0 |= BIT0 | BIT1;
  PJSEL1 &= ~(BIT0 | BIT1);
  /* start LFXT in non-bypass mode without a timeout. */
  CS->KEY   = CS_KEY_VAL;
  CS->CTL1 &= ~CS_CTL1_SELA_7;
  CS->CTL1 |= CS_CTL1_SELA__LFXTCLK;    // source LFXTCLK to ACLK & BCLK
  CS->CTL2 &= ~CS_CTL2_LFXTDRIVE_3;     // lowest drive-strength
  CS->CTL2 |= CS_CTL2_LFXT_EN;
  while (CS->IFG & CS_IFG_LFXTIFG) CS->CLRIFG |= CS_IFG_LFXTIFG;  // wait for error flag to be cleared
  CS->KEY   = 0;
}


void clock_init(void)
{
  /*
   * configure all clocks:
   * - MCLK  (master clock)
   * - ACLK  (auxillary clock, max. 128 kHz)
   * - HSMCLK  (high-speed sub-system master clock)
   * - SMCLK   (low-speed sub-system master clock, same source as HSMCLK)
   * - BCLK  (low-speed backup domain clock, LFXT or REFO, max. 32 kHz)
   *
   * available clock sources are:
   * - LFXTCLK (external low-freq. osc., max. 32768 Hz)
   * - HFXTCLK (external high-freq. osc., 1 - 48 MHz, only available in AM and LPM0)
   * - DCOCLK  (internal digitally controlled osc., max. 48 MHz)
   * - VLOCLK  (internal low-power osc., typ. 10 kHz)
   * - REFOCLK (internal low-power osc., typ. 32 or 128 kHz)
   * - MODCLK  (internal low-power osc., typ. 24 MHz, enabled only when required)
   * - SYSCLK  (internal osc., typ. 5 MHz, enabled only when required, e.g. for memory controller)
   *
   * default clock settings after a reset:
   * - MCLK = HSMCLK = SMCLK = DCOCLK
   * - ACLK = BCLK = LFXTCLK (if LFXT is available, otherwise or if osc fault is detected, REFOCLK is selected)
   * - SMCLK_EN, HSMCLK_EN, MCLK_EN and ACLK_EN bits are set
   *
   * clock requests:
   * - disable conditional clock requests by clearing the ACLK_EN, MCLK_EN, HSMCLK_EN, SMCLK_EN, VLO_EN, REFO_EN and MODOSC_EN bits in CSCLKEN
   */

  /*
   * simplest way to set 48 MHz clock using DCO:
   * PCM_setPowerState(PCM_AM_LDO_VCORE1);
   * CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_48);
   */

  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN0 | GPIO_PIN1, GPIO_PRIMARY_MODULE_FUNCTION); // PJ.0 and PJ.1: LF crystal in and out
  GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_PJ, GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION); // PJ.2 and PJ.3: HF crystal in and out

  CS_setReferenceOscillatorFrequency(CS_REFO_32KHZ);
  CS_setExternalClockSourceFrequency(LFXTCLK_SPEED, HFXTCLK_SPEED);  // this is necessary for CS_startHFXT() to work properly

  CS_setDCOFrequency(DCOCLK_SPEED);    // set DCO frequency (this function uses CS_setDCOCenteredFrequency and CS_tuneDCOFrequency and is not available on pre-release devices)
  CS_startLFXT(CS_LFXT_DRIVE0);     // enable LF crystal in non-bypass mode without a timeout (lowest current drain with drive level = 0)

  // enable the HF crystal:
  CS_startHFXT(0);            // start the HF crystal

  // select clock sources
  CS_initClockSignal(CS_MCLK, MCLK_SRC, MCLK_DIV);          // MCLK   = HFXTCLK
  CS_initClockSignal(CS_HSMCLK, HSMCLK_SRC, HSMCLK_DIV);    // HSMCLK = DCOCLK
  CS_initClockSignal(CS_SMCLK, SMCLK_SRC, SMCLK_DIV);       // SMCLK  = DCOCLK
  CS_initClockSignal(CS_ACLK, ACLK_SRC, ACLK_DIV);          // ACLK   = LFXTCLK
  CS_initClockSignal(CS_BCLK, BCLK_SRC, BCLK_DIV);          // BCLK   = REFOCLk
}


/* init the core functions of the MPS432 */
void msp432_init(void)
{
  WATCHDOG_STOP;
  //WDTCTL = WDTPW + WDTCNTCL + WDTHOLD + WDTSSEL__ACLK + WDTIS_3;

  gpio_init();    // set default configuration for all port pins

  //enter_lpm3();   // to test the minimal current drain
  //enter_lpm45();

  pcm_init();     // set the appropriate voltage level and power mode first (according to the selected frequencies)
  flash_init();   // set the flash wait states
  clock_init();   // configure the clocks
}


/*
 * --- P E R I P H E R A L S ---
 */

// configure EUSCI module for UART operation on pins P1.2 and P1.3, baud rate 9600
void uart_init(void)
{
  float N = (float)UART_CLK_SPEED / (float)UART_BAUDRATE;
  uint_fast8_t  br_generation_mode = 0;
  uint_fast16_t br_div = 0;
  uint_fast16_t br_fmod = 0;
  uint_fast16_t br_smod = 0;
  uint_fast16_t frac = 0;

  // Table for UCxBRS (see Ref. Manual p.721, Table 22-4):
  const uint_fast16_t UCxBRS[] = {  0,  529,  715,  835, 1001, 1252, 1430, 1670, 2147, 2224, 2503, 3000, 3335, 3575, 3753, 4003, 4286, 4378, 5002, 5715, 6003, 6254, 6432, 6667, 7001, 7147, 7503, 7861, 8004, 8333, 8464, 8572, 8751, 9004, 9170, 9288,
                                 0x00, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x11, 0x21, 0x22, 0x44, 0x25, 0x49, 0x4A, 0x52, 0x92, 0x53, 0x55, 0xAA, 0x6B, 0xAD, 0xB5, 0xB6, 0xD6, 0xB7, 0xBB, 0xDD, 0xED, 0xEE, 0xBF, 0xDF, 0xEF, 0xF7, 0xFB, 0xFD, 0xFE };
  if (N > 16)
  {
    br_div = (uint_fast16_t)(N / 16);
    br_fmod = (uint_fast16_t)(((N / 16.0f) - (float)br_div) * 16);
    br_generation_mode = 0x01;  // oversampling
  } else
  {
    br_div = (uint_fast16_t)N;
    br_generation_mode = 0x00;  // low-frequency
  }
  frac = (N - (uint_fast16_t)N) * 10000;   // fractional portion

  int_fast8_t i;
  for (i = 35; i >= 0; i--)
  {
    if (frac >= UCxBRS[i])
    {
      br_smod = UCxBRS[36 + i];
      break;
    }
  }

  /* disable module */
  UART_MODULE->CTLW0  = UCSWRST | UART_CLK_SRC;   /* LSB first, 1 stop bit, no parity, 8-bit characters, asynchronous mode*/
  UART_MODULE->BRW    = br_div;
  UART_MODULE->MCTLW  = (br_smod << 8) + (br_fmod << 4) + br_generation_mode;
  UART_MODULE->CTLW0 &= ~UCSWRST;                 /* enable module */
  UART_MODULE->IFG    = 0;                        /* &= ~(UCRXIFG | UCTXIFG); */
  UART_MODULE->IE     = UCRXIE;                   /* enable RX interrupt */

  /* write a newline */
  UART_WRITE_BYTE('\r');
  UART_WRITE_BYTE('\n');
}


/* initializes SPI module in 3-pin master mode */
void spi_b_init(EUSCI_B_SPI_Type* spi_module, uint32_t sck_speed, uint_fast8_t cpol, uint_fast8_t cpha)
{
  // 3-wire SPI, MSB first, master mode, 8-bit character length
  spi_module->CTLW0 = UCSWRST | (UCMSB + UCMST + UCSYNC + (cpol ? UCCKPL : 0) + (cpha ? 0 : UCCKPH) + SPI_CLK_SRC);
  spi_module->BRW   = (uint16_t)(SPI_CLK_SPEED / sck_speed);
  spi_module->IE   &= ~(UCTXIE + UCRXIE);   /* make sure interrupts are disabled */
  spi_module->IFG   = 0;                    /* clear IFGs */
}


/* initializes SPI module in 3-pin master mode */
void spi_a_init(EUSCI_A_SPI_Type* spi_module, uint32_t sck_speed, uint_fast8_t cpol, uint_fast8_t cpha)
{
  // 3-wire SPI, MSB first, master mode, 8-bit character length
  spi_module->CTLW0 = UCSWRST | (UCMSB + UCMST + UCSYNC + (cpol ? UCCKPL : 0) + (cpha ? 0 : UCCKPH) + SPI_CLK_SRC);
  spi_module->BRW   = (uint16_t)(SPI_CLK_SPEED / sck_speed);
  spi_module->IE   &= ~(UCTXIE + UCRXIE);   /* make sure interrupts are disabled */
  spi_module->IFG   = 0;                    /* clear IFGs */
}


void spi_b_set_speed(EUSCI_B_SPI_Type* spi_module, uint32_t sck_speed)
{
  uint32_t is_enabled = SPI_IS_ENABLED(spi_module);
  SPI_DISABLE(spi_module);
  spi_module->BRW   = (uint16_t)(SPI_CLK_SPEED / sck_speed);
  if (is_enabled)
  {
    SPI_ENABLE(spi_module);
  }
}


void spi_a_set_speed(EUSCI_A_SPI_Type* spi_module, uint32_t sck_speed)
{
  uint32_t is_enabled = SPI_IS_ENABLED(spi_module);
  SPI_DISABLE(spi_module);
  spi_module->BRW   = (uint16_t)(SPI_CLK_SPEED / sck_speed);
  if (is_enabled)
  {
    SPI_ENABLE(spi_module);
  }
}


void spi_b_read(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t* out_data)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  SPI_CLR_RXIFG(SDCARD_SPI_MODULE);
#if SPI_FAST_READ
  size--;
  SPI_WRITE_BYTE(spi_module, 0x00);      /* generate the clock */
#endif /* SPI_FAST_READ */
  while (num_bytes)
  {
    SPI_READ_BYTE(spi_module, *out_data);
    out_data++;
    num_bytes--;
  }
#if SPI_FAST_READ
  SPI_WAIT_RXNE(spi_module);
  *out_data = spi_module->RXBUF;
#endif /* SPI_FAST_READ */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void spi_a_read(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t* out_data)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  SPI_CLR_RXIFG(SDCARD_SPI_MODULE);
#if SPI_FAST_READ
  size--;
  SPI_WRITE_BYTE(spi_module, 0x00);      /* generate the clock */
#endif /* SPI_FAST_READ */
  while (num_bytes)
  {
    SPI_READ_BYTE(spi_module, *out_data);
    out_data++;
    num_bytes--;
  }
#if SPI_FAST_READ
  SPI_WAIT_RXNE(spi_module);
  *out_data = spi_module->RXBUF;
#endif /* SPI_FAST_READ */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void spi_b_write(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, const uint8_t* data)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  while (num_bytes)
  {
    SPI_WRITE_BYTE(spi_module, *data);
    data++;
    num_bytes--;
  }
  SPI_WAIT_BUSY(spi_module);
  SPI_CLR_RXIFG(spi_module);
  (void)spi_module->RXBUF;      /* dummy read to clear RX buffer and overrun flag */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void spi_a_write(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, const uint8_t* data)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  while (num_bytes)
  {
    SPI_WRITE_BYTE(spi_module, *data);
    data++;
    num_bytes--;
  }
  SPI_WAIT_BUSY(spi_module);
  SPI_CLR_RXIFG(spi_module);
  (void)spi_module->RXBUF;      /* dummy read to clear RX buffer and overrun flag */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void spi_b_write_const(EUSCI_B_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t fill_value)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  while (num_bytes)
  {
    SPI_WRITE_BYTE(spi_module, fill_value);
    num_bytes--;
  }
  SPI_WAIT_BUSY(spi_module);    /* wait until RX finished */
  SPI_CLR_RXIFG(spi_module);    /* clear RX flag */
  (void)spi_module->RXBUF;      /* dummy read to clear RX buffer and overrun flag */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void spi_a_write_const(EUSCI_A_SPI_Type* spi_module, uint_fast16_t num_bytes, uint8_t fill_value)
{
  uint_fast8_t spi_en = 0;
  /* if module is not enabled, then enable it (... and disable again it after the read operation) */
  if (!SPI_IS_ENABLED(spi_module))
  {
    SPI_ENABLE(spi_module);
    spi_en = 1;
  }
  while (num_bytes)
  {
    SPI_WRITE_BYTE(spi_module, fill_value);
    num_bytes--;
  }
  SPI_WAIT_BUSY(spi_module);    /* wait until RX finished */
  SPI_CLR_RXIFG(spi_module);    /* clear RX flag */
  (void)spi_module->RXBUF;      /* dummy read to clear RX buffer and overrun flag */
  if (spi_en)
  {
    SPI_DISABLE(spi_module);
  }
}


void i2c_init(EUSCI_B_Type* i2c_module, uint32_t scl_speed)
{
  /* master mode, 7-bit address */
  i2c_module->CTLW0 = UCSWRST | (UCMST | UCMODE_3 | I2C_CLK_SRC);
  i2c_module->BRW   = (uint16_t)(I2C_CLK_SPEED / scl_speed);
  i2c_module->IE    = 0;
}


uint_fast8_t i2c_read(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint16_t cmd, uint32_t num_bytes, uint8_t* out_data)
{
  I2C_ENABLE(i2c_module);
  i2c_module->I2CSA  = slave_addr;          /* set slave address (initiates master mode) */
  i2c_module->CTLW0 |= UCTR;                /* set to transmitter mode */
  i2c_module->CTLW0 |= UCTXSTT;             /* create START condition (write command = 0) */
  I2C_WAIT_START_CLR(i2c_module);           /* wait until address has been transmitted */
  __delay_cycles(MCLK_SPEED / I2C_CLK_SPEED);  /* wait for the ACK/NACK */
  if (i2c_module->IFG & UCNACKIFG)
  {
    // NACK received -> abort
    i2c_module->CTLW0 |= UCTXSTP;
    I2C_WAIT_STOP_CLR(i2c_module);
    I2C_DISABLE(i2c_module);
    return 0;
  }
  I2C_WRITE_BYTE(i2c_module, cmd >> 8);     /* write measurement command MSB */
  I2C_WRITE_BYTE(i2c_module, cmd & 0xff);   /* write measurement command LSB */
  I2C_WAIT_TXE(i2c_module);
  i2c_module->CTLW0 &= ~UCTR;               /* switch to receiver mode, send repeated START */
  i2c_module->CTLW0 |= UCTXSTT;             /* create START condition */

  while (num_bytes)
  {
    I2C_READ_BYTE(i2c_module, *out_data);
    out_data++;
    num_bytes--;
  }
  i2c_module->CTLW0 |= UCTXSTP;             /* generate STOP condition */
  I2C_WAIT_STOP_CLR(i2c_module);            /* wait for transmission to end, stop bit will be cleared automatically */
  I2C_CLR_RXBUF(i2c_module);

  I2C_DISABLE(i2c_module);

  return 1;
}


uint_fast8_t i2c_read_cmd8(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint8_t cmd, uint32_t num_bytes, uint8_t* out_data)
{
  I2C_ENABLE(i2c_module);
  i2c_module->I2CSA  = slave_addr;          /* set slave address (initiates master mode) */
  i2c_module->CTLW0 |= (UCTR | UCTXSTT);    /* set to transmitter mode and create START condition (write command = 0) */
  I2C_WAIT_START_CLR(i2c_module);           /* wait until address has been transmitted */
  __delay_cycles(MCLK_SPEED / I2C_SCL_SPEED);  /* wait for the ACK/NACK */
  if (i2c_module->IFG & UCNACKIFG)
  {
    // NACK received -> abort
    i2c_module->CTLW0 |= UCTXSTP;           /* STOP condition */
    I2C_WAIT_STOP_CLR(i2c_module);
    I2C_DISABLE(i2c_module);
    return 0;
  }
  I2C_WRITE_BYTE(i2c_module, cmd);          /* write command */
  I2C_WAIT_TXE(i2c_module);
  i2c_module->CTLW0 &= ~UCTR;               /* switch to receiver mode, send repeated START */
  i2c_module->CTLW0 |= UCTXSTT;             /* create START condition */

  while (num_bytes)
  {
    I2C_READ_BYTE(i2c_module, *out_data);
    out_data++;
    num_bytes--;
  }
  i2c_module->CTLW0 |= UCTXSTP;             /* generate STOP condition */
  I2C_WAIT_STOP_CLR(i2c_module);            /* wait for transmission to end, stop bit will be cleared automatically */
  I2C_CLR_RXBUF(i2c_module);

  I2C_DISABLE(i2c_module);

  return 1;
}

uint_fast8_t i2c_write(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint16_t cmd)
{
    I2C_ENABLE(i2c_module);
    i2c_module->I2CSA  = slave_addr;          /* set slave address (initiates master mode) */
    i2c_module->CTLW0 |= UCTR;                /* set to transmitter mode */
    i2c_module->CTLW0 |= UCTXSTT;             /* create START condition (write command = 0) */
    I2C_WAIT_START_CLR(i2c_module);           /* wait until address has been transmitted */
    __delay_cycles(MCLK_SPEED / I2C_CLK_SPEED);  /* wait for the ACK/NACK */
    if (i2c_module->IFG & UCNACKIFG)
    {
        // NACK received -> abort
        i2c_module->CTLW0 |= UCTXSTP;
        I2C_WAIT_STOP_CLR(i2c_module);
        I2C_DISABLE(i2c_module);
        return 0;
    }
    I2C_WRITE_BYTE(i2c_module, cmd >> 8);     /* write measurement command MSB */
    I2C_WRITE_BYTE(i2c_module, cmd & 0xff);   /* write measurement command LSB */
    I2C_WAIT_TXE(i2c_module);

    I2C_DISABLE(i2c_module);

    return 1;
}

uint_fast8_t i2c_write_cmd8(EUSCI_B_Type* i2c_module, uint8_t slave_addr, uint8_t cmd)
{
    I2C_ENABLE(i2c_module);
    i2c_module->I2CSA  = slave_addr;          /* set slave address (initiates master mode) */
    i2c_module->CTLW0 |= UCTR;                /* set to transmitter mode */
    i2c_module->CTLW0 |= UCTXSTT;             /* create START condition (write command = 0) */
    I2C_WAIT_START_CLR(i2c_module);           /* wait until address has been transmitted */
    __delay_cycles(MCLK_SPEED / I2C_CLK_SPEED);  /* wait for the ACK/NACK */
    if (i2c_module->IFG & UCNACKIFG)
    {
        // NACK received -> abort
        i2c_module->CTLW0 |= UCTXSTP;
        I2C_WAIT_STOP_CLR(i2c_module);
        I2C_DISABLE(i2c_module);
        return 0;
    }
    I2C_WRITE_BYTE(i2c_module, cmd);          /* write command */
    I2C_WAIT_TXE(i2c_module);

    I2C_DISABLE(i2c_module);

    return 1;
}

/* single channel, no interrupt */
void adc_init_simple(uint_fast8_t adc_ch)
{
  if (adc_ch > 31) { return; }

  /* enable REF module */
  REF_SET_2V5;
  REF_ENABLE;

  /* configure ADC */
  ADC14->CTL0    &= ~ADC14_CTL0_ENC;              /* disable conversion before modifying registers */
  ADC14->CTL0     = ADC14_CTL0_SSEL__MODCLK |     /* clock source */
                    ADC14_CTL0_PDIV__64     |     /* pre-divider (24 / 64 = 375kHz) */
                    ADC14_CTL0_DIV__1       |     /* divider */
                    ADC14_CTL0_SHS_0        |     /* sample and hold source select (0 = software trigger, 1..7 = TA0_C1/TA0_C2/TA1_C1/.../TA3_C1) */
                    ADC14_CTL0_SHT0__64     |     /* sample and hold time for ADC14MEM0 to ADC14MEM7 and ADC14MEM24 to ADC14MEM31 */
                    ADC14_CTL0_SHT1__64     |     /* sample and hold time for ADC14MEM8 to ADC14MEM23 */
                    ADC14_CTL0_CONSEQ_0     |     /* conversion sequence mode select (single-channel, single-conversion) */
                    ADC14_CTL0_SHP          |     /* sample and hold pulse mode select */
                    ADC14_CTL0_ON;                /* enable module */
  ADC14->CTL1     = ADC14_CTL1_PWRMD_0      |     /* power mode (0 = any sampling rate up to 1Msps, 2 = low-power mode up to 200ksps and 12-bit) */
                    ADC14_CTL1_RES__14BIT   |     /* resolution (14-bit, 16 clock cycles) */
                    (0 << ADC14_CTL1_CSTARTADD_OFS);   /* conversion memory register start address (0x00 to 0x1f for MEM0 to MEM31) */
  /* conversion memory control register (for MEM0) */
  ADC14->MCTL[0] |= adc_ch                  |     /* select input channel */
                    ADC14_MCTLN_VRSEL_1;          /* reference select (Vref=int_buf); use ADC14_MCTLN_VRSEL_0 for AVCC */
  ADC14->IER0    |= ADC14_IER0_IE0;               /* enable ADC conv complete interrupt */
  ADC14->CTL0    |= ADC14_CTL0_ENC;               /* enable conversion */
}


void adc_init_tempsensor(void)
{
  /* enable REF and temperature sensor */
  REF_SET_2V5;
  REF_ENABLE;
  REF_TEMP_SENSOR_ENABLE;

  /* configure ADC */
  ADC14->CTL0    &= ~ADC14_CTL0_ENC;              /* disable conversion before modifying registers */
  ADC14->CTL0     = ADC14_CTL0_SSEL__MODCLK |     /* clock source */
                    ADC14_CTL0_PDIV__1      |     /* pre-divider */
                    ADC14_CTL0_DIV__1       |     /* divider */
                    ADC14_CTL0_SHS_0        |     /* sample and hold source select (0 = software trigger, 1..7 = TA0_C1/TA0_C2/TA1_C1/.../TA3_C1) */
                    ADC14_CTL0_SHT0__192    |     /* sample and hold time for ADC14MEM0 to ADC14MEM7 and ADC14MEM24 to ADC14MEM31 */
                    ADC14_CTL0_SHT1__192    |     /* sample and hold time for ADC14MEM8 to ADC14MEM23 */
                    ADC14_CTL0_CONSEQ_0     |     /* conversion sequence mode select (single-channel, single-conversion) */
                    ADC14_CTL0_SHP          |     /* sample and hold pulse mode select */
                    ADC14_CTL0_ON;                /* enable module */
  ADC14->CTL1     = ADC14_CTL1_PWRMD_0      |     /* power mode (0 = any sampling rate up to 1Msps, 2 = low-power mode up to 200ksps and 12-bit) */
                    ADC14_CTL1_RES__14BIT   |     /* resolution (14-bit, 16 clock cycles) */
                    ADC14_CTL1_TCMAP;             /* select temperature sensor for channel ADC_CH_TEMPSENSOR */
  /* conversion memory control register (for MEM0) */
  ADC14->MCTL[0] |= ADC_CH_TEMPSENSOR       |     /* select input A22 (A10 on the 64-pin package) */
                    ADC14_MCTLN_VRSEL_1;          /* reference select (Vref=int_buf); use ADC14_MCTLN_VRSEL_0 for AVCC */
  ADC14->IER0    |= ADC14_IER0_IE0;               /* enable ADC conv complete interrupt */
  ADC14->CTL0    |= ADC14_CTL0_ENC;               /* enable conversion */
}


/* an example how multi-ch sampling could be used */
/* note: temperature sensor doesn't work on Rev.B devices in multi-CH mode (errata ADC46) */
void adc_init_multi_ch(uint_fast8_t adc_ch1, uint_fast8_t adc_ch2)
{
  if (adc_ch1 > 31 || adc_ch2 > 31) { return; }

  /* enable REF module */
  REF_SET_2V5;
  REF_ENABLE;

  /* enable temperature sensor if required */
  uint32_t tcmap = 0;
  if (adc_ch1 == ADC_CH_TEMPSENSOR || adc_ch2 == ADC_CH_TEMPSENSOR)
  {
    REF_TEMP_SENSOR_ENABLE;
    tcmap = ADC14_CTL1_TCMAP;
  }

  /* configure ADC */
  ADC14->CTL0    &= ~ADC14_CTL0_ENC;              /* disable conversion before modifying registers */
  ADC14->CTL0     = ADC14_CTL0_SSEL__MODCLK |     /* clock source */
                    ADC14_CTL0_PDIV__32     |     /* pre-divider */
                    ADC14_CTL0_DIV__1       |     /* divider */
                    ADC14_CTL0_MSC          |     /* multiple sample and conversion (automatic iteration) */
                    ADC14_CTL0_SHS_0        |     /* sample and hold source select (0 = software trigger, 1..7 = TA0_C1/TA0_C2/TA1_C1/.../TA3_C1) */
                    ADC14_CTL0_SHT0__128    |     /* sample and hold time for ADC14MEM0 to ADC14MEM7 and ADC14MEM24 to ADC14MEM31 */
                    ADC14_CTL0_SHT1__128    |     /* sample and hold time for ADC14MEM8 to ADC14MEM23 */
                    ADC14_CTL0_CONSEQ_1     |     /* conversion sequence mode select (sequence-of-channels) */
                    ADC14_CTL0_SHP          |     /* sample and hold pulse mode select */
                    ADC14_CTL0_ON;                /* enable module */
  ADC14->CTL1     = ADC14_CTL1_PWRMD_0      |     /* power mode (0 = any sampling rate up to 1Msps, 2 = low-power mode up to 200ksps and 12-bit) */
                    ADC14_CTL1_RES__14BIT   |     /* resolution */
                    (0 << ADC14_CTL1_CSTARTADD_OFS) |   /* conversion memory register start address (0x00 to 0x1f for MEM0 to MEM31) */
                    tcmap;                        /* select temperature sensor for ADC input channel ADC_CH_TEMPSENSOR */
  ADC14->MCTL[0] |= adc_ch1                 |     /* select input */
                    ADC14_MCTLN_VRSEL_1;          /* reference select (Vref=int_buf); use ADC14_MCTLN_VRSEL_0 for AVCC */
  ADC14->MCTL[1] |= adc_ch2                 |     /* select input */
                    ADC14_MCTLN_VRSEL_1     |     /* reference select (Vref=int_buf); use ADC14_MCTLN_VRSEL_0 for AVCC */
                    ADC14_MCTLN_EOS;              /* mark the end of the sequence */
  ADC14->IER0    |= ADC14_IER0_IE1;               /* enable ADC conv complete interrupt */
  ADC14->CTL0    |= ADC14_CTL0_ENC;               /* enable conversion */
}


/*
 * --- T I M E R S ---
 */

// 24-bit timer used for periodic interrupts (e.g. for an RTOS scheduler), clocked with CPU
void systick_enable(void)
{
  SYSTICK_SET_PERIOD(SYSTICK_PERIOD);
  SYSTICK_START;
}


uint_fast8_t rtc_set_time(const RTC_C_Calendar* rtc_cal_time)
{
  // check if valid
  if (rtc_cal_time->month > 12 || rtc_cal_time->hours > 23 || rtc_cal_time->minutes > 59 || rtc_cal_time->seconds > 59)
  {
    uart_println("invalid parameters for rtc_set_time()");
    return 0;
  }
  RTC_C_holdClock();
  RTC_C_initCalendar(rtc_cal_time, RTC_C_FORMAT_BINARY);
  RTC_C_startClock();
  return 1;
}


const char* rtc_get_time(uint_fast8_t with_date)
{
  static char buffer[32];
  RTC_C_Calendar cal = RTC_C_getCalendarTime();
  if (with_date)
  {
    sprintf(buffer, "%u-%02u-%02u %02u:%02u:%02u", cal.year, cal.month, cal.dayOfmonth, cal.hours, cal.minutes, cal.seconds);
  } else
  {
    sprintf(buffer, "%02u:%02u:%02u", cal.hours, cal.minutes, cal.seconds);
  }
  return buffer;
}


/*
 * The condition is a logical and of all of the parameters. For example if the minutes and hours alarm is
 * set, then the interrupt will only assert when the minutes AND the hours change to the specified setting
 */
void rtc_set_alarm(uint_fast8_t hour, uint_fast8_t minute, uint_fast8_t day_of_week, uint_fast8_t day_of_month)
{
    // pass RTC_C_ALARMCONDITION_OFF to disable the respective alarm
    RTC_C_setCalendarAlarm(minute, hour, day_of_week, day_of_month);
    RTC_C_clearInterruptFlag(RTC_C_getInterruptStatus());
    RTC_C_enableInterrupt(RTC_C_CLOCK_ALARM_INTERRUPT);
}


/* 32-bit ARM timer */
void timer32_init(void)
{
  /*
   * modes:
   * - free run mode: counts from UINT16_MAX or UINT32_MAX down to 0
   * - periodic mode: counts from a specified value down to 0
   * - one-shot option: if set to 0, the counter does not automatically restart when it reaches 0
   *
   * clock source: TIMCLK, which is an enabled version of HCLK (= MCLK freq.) with same frequency and is active when
   * one or more of the counters in Timer32 module are enabled.
   *
   * available prescalers: 1, 16 or 256
   */

  /* prescaler TIMER32_PRESCALER, 32-bit, free-running mode */
  TIMER32_1->CONTROL  = TIMER32_CONTROL_SIZE | TIMER32_PRESCALER | TIMER32_CONTROL_IE;
  TIMER32_1->LOAD     = 0xffffffff;
  TIMER32_1->CONTROL |= TIMER32_CONTROL_ENABLE;
}


uint64_t timer32_now(void)
{
  uint32_t hw;
  uint_fast8_t ie = INTERRUPT_MASTER_ENABLED;
  __disable_interrupt();

  do
  {
    timer32_update();
    /* no majority vote necessary, TIMCLK is synchronous to MCLK */
    /* invert the timestamp (it's a count-down timer) */
    hw = ~(TIMER32_1->VALUE);
  } while (Timer32_getInterruptStatus(TIMER32_BASE));
  uint64_t t = ((uint64_t)timer32_sw_ext << 32) | hw;

  if (ie)
  {
    __enable_interrupt();   /* restore previous interrupt state */
  }

  return t;
}


void timer32_update(void)
{
  if (Timer32_getInterruptStatus(TIMER32_BASE))
  {
    timer32_sw_ext++;
    Timer32_clearInterruptFlag(TIMER32_BASE);
  }
}


/* configure a PWM timer (ccr can be 1 - 4) */
void pwm_init_ta0(uint_fast8_t ccr)
{
  if (ccr == 0 || ccr > 4) return;

  TA0CCR0  = PWM_PERIOD;  /* set output when counter reaches PWM_PERIOD (and reset counter value to 0) */
  TA0CCTL0 = TIMER_A_CCTLN_OUTMOD_7 | TIMER_A_CCTLN_CCIE;
  TIMER_A0->CCTL[ccr] = TIMER_A_CCTLN_OUTMOD_7;  /* output is reset when the timer count reaches TA0CCRx */
  TIMER_A0->CCR[ccr]  = 0;           /* start value */

  /* start the timer (SMCLK as source, count up) */
  TA0CTL = TIMER_A0_SRC | MC__UP | TACLR;
}


/* configure a PWM timer */
void pwm_init_ta1(uint_fast8_t ccr)
{
  if (ccr == 0 || ccr > 4) return;

  TA1CCR0  = PWM_PERIOD;  /* set output when counter reaches PWM_PERIOD (and reset counter value to 0) */
  TA1CCTL0 = TIMER_A_CCTLN_OUTMOD_7;
  TIMER_A1->CCTL[ccr] = TIMER_A_CCTLN_OUTMOD_7;  /* 7: output is reset when the timer count reaches TA0CCRx */
  TIMER_A1->CCR[ccr]  = 0;

  /* start the timer (SMCLK as source, count up) */
  TA1CTL = TIMER_A1_SRC | MC__UP | TACLR;
}
