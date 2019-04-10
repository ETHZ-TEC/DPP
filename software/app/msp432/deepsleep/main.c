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
 *
 * Author:  Reto Da Forno
 */

/*
 * puts the MSP432 on the DPP in a state of minimal power dissipation
 * (less than 100nA)
 */

#include <msp.h>


//#define DPP_REV1


void gpio_init(void)
{
#ifdef DPP_REV1

  // set all GPIOs as outputs (low)
  P1DIR |= 0xFF; P1OUT = 0;
  P2DIR |= 0xFF; P2OUT = 0;
  P3DIR |= 0xFF; P3OUT = 0;
  P4DIR |= 0xFF; P4OUT = 0;
  P5DIR |= 0xFF; P5OUT = 0;
  P6DIR |= 0xFF; P6OUT = 0;
  P7DIR |= 0xFF; P7OUT = 0;
  P8DIR |= 0xFF; P8OUT = 0;
  P9DIR |= 0xFF; P9OUT = 0;
  P10DIR |= 0xFF; P10OUT = 0;
  PJDIR |= (BIT2 | BIT3); PJOUT = 0;
  PJREN |= (BIT0 | BIT1 | BIT4 | BIT5 | BIT6 | BIT7);     // pulldown for the other pins

  P1DIR &= ~BIT1;     // P1.1 and P10.1 must be inputs (BOLT IND lines)
  P10DIR &= ~BIT1;
  P1DIR &= ~BIT2;     // P1.2 must be an input (UART RXD) or output high!

  // let LED blink once
  P2OUT |= BIT4;
  __delay_cycles(100000);
  P2OUT &= ~BIT4;

#else /* DPP_REV1 */

  // set all GPIOs as outputs (low)
  P1DIR |= ~(BIT2 | BIT3); P1OUT = 0;            // UART RXD/TXD
  P2DIR |= 0xFF; P2OUT = 0;
  P3DIR |= 0xFF; P3OUT = 0;
  P4DIR |= ~(BIT4 | BIT3); P4OUT = 0;   // BOLT IND lines
  P5DIR |= ~BIT5; P5OUT = 0;
  P6DIR |= (BIT6 | BIT7); P6OUT = 0;
  P7DIR |= 0xFF; P7OUT = 0;
  P8DIR |= ~(BIT0 | BIT1); P8OUT = 0;
  P9DIR |= 0xFF; P9OUT = 0;
  P10DIR |= 0xFF; P10OUT = 0;
  PJDIR |= ~(BIT4 | BIT5); PJOUT = 0;

  // let LED blink once
  P2OUT |= BIT0;
  __delay_cycles(100000);
  P2OUT &= ~BIT0;

#endif /* DPP_REV1 */
}


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
