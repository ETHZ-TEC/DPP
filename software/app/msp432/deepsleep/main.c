/*
 * Copyright (c) 2016, Swiss Federal Institute of Technology (ETH Zurich).
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
 */

/*
 * puts the MSP432 on the DPP in a state of minimal power dissipation
 * (less than 100nA)
 */

#include <msp.h>


//#define DPP_REV1              // uncomment to compile for the old DPP (version 1)
#define GPIO_OUTPUT     1       // set to 1 to configure GPIOs as outputs (if set to 0, the pins will be configured as inputs w/ pull resistor enabled instead)


#ifdef DPP_REV1

void gpio_init(void)
{
#if GPIO_OUTPUT

  // set all GPIOs as outputs (low)
  P1DIR  = 0xFF;            P1OUT  = 0;
  P2DIR  = 0xFF;            P2OUT  = 0;
  P3DIR  = 0xFF;            P3OUT  = 0;
  P4DIR  = 0xFF;            P4OUT  = 0;
  P5DIR  = 0xFF;            P5OUT  = 0;
  P6DIR  = 0xFF;            P6OUT  = 0;
  P7DIR  = 0xFF;            P7OUT  = 0;
  P8DIR  = 0xFF;            P8OUT  = 0;
  P9DIR  = 0xFF;            P9OUT  = 0;
  P10DIR = 0xFF;            P10OUT = 0;
  PJDIR  = (BIT2 | BIT3);   PJOUT = 0;
  PJREN  = ~(BIT2 | BIT3);

  P1DIR &= ~BIT1;     // P1.1 and P10.1 must be inputs (BOLT IND lines)
  P10DIR &= ~BIT1;
  P1DIR &= ~BIT2;     // P1.2 must be an input (UART RXD) or output high!

#else /* GPIO_OUTPUT */

  // configure all GPIOs as inputs (with pulldown or pullup)
  P1DIR  = 0x0;  P1OUT  = 0; P1REN  = ~BIT1;    // no pull for BOLT IND
  P2DIR  = 0x0;  P2OUT  = 0; P2REN  = 0xFF;
  P3DIR  = 0x0;  P3OUT  = 0; P3REN  = 0xFF;
  P4DIR  = BIT4; P4OUT  = 0; P4REN  = 0xFF;     // LED as output
  P5DIR  = 0x0;  P5OUT  = 0; P5REN  = 0xFF;
  P6DIR  = 0x0;  P6OUT  = 0; P6REN  = 0xFF;
  P7DIR  = 0x0;  P7OUT  = 0; P7REN  = 0xFF;
  P8DIR  = 0x0;  P8OUT  = 0; P8REN  = 0xFF;
  P9DIR  = 0x0;  P9OUT  = 0; P9REN  = 0xFF;
  P10DIR = 0x0;  P10OUT = 0; P10REN = 0xFF;
  PJDIR  = 0x0;  PJOUT  = 0; PJREN  = 0xFF;

  // let LED blink twice
  P2OUT |= BIT4;
  __delay_cycles(100000);
  P2OUT &= ~BIT4;
  __delay_cycles(100000);

#endif /* GPIO_OUTPUT */

  // blink LED
  P2OUT |= BIT4;
  __delay_cycles(100000);
  P2OUT &= ~BIT4;
}

#else /* DPP_REV1 */

void gpio_init(void)
{
#if GPIO_OUTPUT

  // set all GPIOs as outputs (low)
  P1DIR  = ~(BIT2 | BIT3);  P1OUT  = 0;       // UART RXD/TXD
  P2DIR  = 0xFF;            P2OUT  = 0;
  P3DIR  = 0xFF;            P3OUT  = 0;
  P4DIR  = ~(BIT4 | BIT3);  P4OUT  = 0;       // BOLT IND lines
  P5DIR  = ~BIT5;           P5OUT  = 0;
  P6DIR  = (BIT6 | BIT7);   P6OUT  = 0;
  P7DIR  = 0xFF;            P7OUT  = 0;
  P8DIR  = ~(BIT0 | BIT1);  P8OUT  = 0;
  P9DIR  = 0xFF;            P9OUT  = 0;
  P10DIR = 0xFF;            P10OUT = 0;
  PJDIR  = ~(BIT4 | BIT5);  PJOUT  = 0;

#else /* GPIO_OUTPUT */

  // configure all GPIOs as inputs (with pulldown or pullup)
  P1DIR  = 0x0;  P1OUT  = (BIT2 | BIT3); P1REN  = 0xFF;             // pullup for UART RXD/TXD
  P2DIR  = BIT0; P2OUT  = 0;             P2REN  = ~BIT0;            // LED as output
  P3DIR  = 0x0;  P3OUT  = 0;             P3REN  = 0xFF;
  P4DIR  = 0x0;  P4OUT  = 0;             P4REN  = ~(BIT4 | BIT3);   // no pull for BOLT IND
  P5DIR  = 0x0;  P5OUT  = 0;             P5REN  = ~BIT5;            // no pull for VIN_MON
  P6DIR  = 0x0;  P6OUT  = 0;             P6REN  = ~(BIT6 | BIT7);   // no pull for I2C pins
  P7DIR  = 0x0;  P7OUT  = 0;             P7REN  = 0xFF;
  P8DIR  = 0x0;  P8OUT  = 0;             P8REN  = ~(BIT0 | BIT1);   // no pull for COM_EN and PERIPH_EN
  P9DIR  = 0x0;  P9OUT  = 0;             P9REN  = 0xFF;
  P10DIR = 0x0;  P10OUT = 0;             P10REN = 0xFF;
  PJDIR  = 0x0;  PJOUT  = 0;             PJREN  = 0xFF;

  // let LED blink twice
  P2OUT |= BIT0;
  __delay_cycles(100000);
  P2OUT &= ~BIT0;
  __delay_cycles(100000);

#endif /* GPIO_OUTPUT */

  // blink LED
  P2OUT |= BIT0;
  __delay_cycles(100000);
  P2OUT &= ~BIT0;
}

#endif /* DPP_REV1 */


void main(void)
{
  WDTCTL = WDTPW | WDTHOLD;

  gpio_init();

  // clear LPM bit to unlock GPIO config
  PCM->CTL1 &= ~(PCM_CTL1_LOCKLPM5 | PCM_CTL1_LOCKBKUP);

  // disable SVS low and high side
  PSS->KEY = PSS_KEY_KEY_VAL;
  PSS->CTL0 |= BIT0 | BIT8;
  PSS->KEY = 0;

  // go to lpm4.5
  SCB->SCR |= (1 << 2);
  PCM->CTL0 = (0x695A0000 | PCM_CTL0_LPMR_12 |
               (PCM->CTL0 & ~(PCM_CTL0_KEY_MASK | PCM_CTL0_LPMR_MASK)));
  __asm("    wfi\n");
  __no_operation();

  while (1);
}
