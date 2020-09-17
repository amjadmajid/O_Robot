// Clock.c
// Runs on the MSP432 
// Daniel and Jonathan Valvano
// February 23, 2017

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

#include "msp.h"

void error(void);

#if 0
uint32_t ClockFrequency = 3000000; // cycles/second
//static uint32_t SubsystemFrequency = 3000000; // cycles/second


// ------------Clock_InitFastest------------
// Configure the system clock to run at the fastest
// and most accurate settings.  For example, if the
// LaunchPad has a crystal, it should be used here.
// Call BSP_Clock_GetFreq() to get the current system
// clock frequency for the LaunchPad.
// Input: none
// Output: none
uint32_t Prewait = 0;                   // loops between BSP_Clock_InitFastest() called and PCM idle (expect 0)
uint32_t CPMwait = 0;                   // loops between Power Active Mode Request and Current Power Mode matching requested mode (expect small)
uint32_t Postwait = 0;                  // loops between Current Power Mode matching requested mode and PCM module idle (expect about 0)
uint32_t IFlags = 0;                    // non-zero if transition is invalid
uint32_t Crystalstable = 0;             // loops before the crystal stabilizes (expect small)

void clock_init_48MHz(void)
{
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
//  while(PCMCTL1&0x00000100){
    Prewait = Prewait + 1;
    if(Prewait >= 100000){
      return;                           // time out error
    }
  }
  // request power active mode LDO VCORE1 to support the 48 MHz frequency
  PCM->CTL0 = (PCM->CTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
//  PCMCTL0 = (PCMCTL0&~0xFFFF000F) |     // clear PCMKEY bit field and AMR bit field
            0x695A0000 |                // write the proper PCM key to unlock write access
            0x00000001;                 // request power active mode LDO VCORE1
  // check if the transition is invalid (see Figure 7-3 on p344 of datasheet)
  if(PCM->IFG&0x00000004){
    IFlags = PCM->IFG;                    // bit 2 set on active mode transition invalid; bits 1-0 are for LPM-related errors; bit 6 is for DC-DC-related error
    PCM->CLRIFG = 0x00000004;             // clear the transition invalid flag
    // to do: look at CPM bit field in PCMCTL0, figure out what mode you're in, and step through the chart to transition to the mode you want
    // or be lazy and do nothing; this should work out of reset at least, but it WILL NOT work if Clock_Int32kHz() or Clock_InitLowPower() has been called
    return;
  }
  // wait for the CPM (Current Power Mode) bit field to reflect a change to active mode LDO VCORE1
  while((PCM->CTL0&0x00003F00) != 0x00000100){
    CPMwait = CPMwait + 1;
    if(CPMwait >= 500000){
      return;                           // time out error
    }
  }
  // wait for the PCMCTL0 and Clock System to be write-able by waiting for Power Control Manager to be idle
  while(PCM->CTL1&0x00000100){
    Postwait = Postwait + 1;
    if(Postwait >= 100000){
      return;                           // time out error
    }
  }
  // initialize PJ.3 and PJ.2 and make them HFXT (PJ.3 built-in 48 MHz crystal out; PJ.2 built-in 48 MHz crystal in)
  PJ->SEL0 |= 0x0C;
  PJ->SEL1 &= ~0x0C;                    // configure built-in 48 MHz crystal for HFXT operation
//  PJDIR |= 0x08;                      // make PJ.3 HFXTOUT (unnecessary)
//  PJDIR &= ~0x04;                     // make PJ.2 HFXTIN (unnecessary)
  CS->KEY = 0x695A;                     // unlock CS module for register access
  CS->CTL2 = (CS->CTL2&~0x00700000) |   // clear HFXTFREQ bit field
           0x00600000 |                 // configure for 48 MHz external crystal
           0x00010000 |                 // HFXT oscillator drive selection for crystals >4 MHz
           0x01000000;                  // enable HFXT
  CS->CTL2 &= ~0x02000000;              // disable high-frequency crystal bypass
  // wait for the HFXT clock to stabilize
  while(CS->IFG&0x00000002){
    CS->CLRIFG = 0x00000002;              // clear the HFXT oscillator interrupt flag
    Crystalstable = Crystalstable + 1;
    if(Crystalstable > 100000){
      return;                           // time out error
    }
  }
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 0
  FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL&~0x0000F000)|FLCTL_BANK0_RDCTL_WAIT_2;
  // configure for 2 wait states (minimum for 48 MHz operation) for flash Bank 1
  FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL&~0x0000F000)|FLCTL_BANK1_RDCTL_WAIT_2;
  CS->CTL1 = 0x20000000 |               // configure for SMCLK divider /4
           0x00100000 |                 // configure for HSMCLK divider /2
           0x00000200 |                 // configure for ACLK sourced from REFOCLK
           0x00000050 |                 // configure for SMCLK and HSMCLK sourced from HFXTCLK
           0x00000005;                  // configure for MCLK sourced from HFXTCLK
  CS->KEY = 0;                          // lock CS module from unintended access
  ClockFrequency = 48000000;
//  SubsystemFrequency = 12000000;
 }

#else

void clock_init_48MHz(void)
{
    volatile uint32_t i;
    uint32_t currentPowerState;

    P1->DIR |= BIT0;                        // P1.0 set as output

    /* NOTE: This example assumes the default power state is AM0_LDO.
     * Refer to  msp432p401x_pcm_0x code examples for more complete PCM
     * operations to exercise various power state transitions between active
     * modes.
     */

    /* Step 1: Transition to VCORE Level 1: AM0_LDO --> AM1_LDO */

    /* Get current power state, if it's not AM0_LDO, error out */
    currentPowerState = PCM->CTL0 & PCM_CTL0_CPM_MASK;
    if (currentPowerState != PCM_CTL0_CPM_0)
        error();

    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_AMR_1;
    while ((PCM->CTL1 & PCM_CTL1_PMR_BUSY));
    if (PCM->IFG & PCM_IFG_AM_INVALID_TR_IFG)
        error();                            // Error if transition was not successful
    if ((PCM->CTL0 & PCM_CTL0_CPM_MASK) != PCM_CTL0_CPM_1)
        error();                            // Error if device is not in AM1_LDO mode

    /* Step 2: Configure Flash wait-state to 1 for both banks 0 & 1 */
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK0_RDCTL_WAIT_MASK)) |
            FLCTL_BANK0_RDCTL_WAIT_1;
    FLCTL->BANK1_RDCTL  = (FLCTL->BANK0_RDCTL & ~(FLCTL_BANK1_RDCTL_WAIT_MASK)) |
            FLCTL_BANK1_RDCTL_WAIT_1;

    /* Step 3: Configure DCO to 48MHz, ensure MCLK uses DCO as source*/
    CS->KEY = CS_KEY_VAL ;                  // Unlock CS module for register access
    CS->CTL0 = 0;                           // Reset tuning parameters
    CS->CTL0 = CS_CTL0_DCORSEL_5;           // Set DCO to 48MHz
    /* Select MCLK = DCO, no divider */
    CS->CTL1 = (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK|
                             CS_CTL1_SELS_MASK | CS_CTL1_DIVS_MASK
        )) | CS_CTL1_SELM__DCOCLK | CS_CTL1_SELS__DCOCLK | CS_CTL1_DIVS__4  ;
    CS->KEY = 0;                            // Lock CS module from unintended accesses

// uncommented for debugging
//     /* Step 4: Output MCLK to port pin to demonstrate 48MHz operation */
//     P4->DIR |= BIT3;
//     P4->SEL0 |=BIT3;                        // Output MCLK
//     P4->SEL1 &= ~(BIT3);
 }

void error(void)
{
    volatile uint32_t i;

    while (1)
    {
        P1->OUT ^= BIT0;
        for(i = 20000; i > 0; i--);           // Blink LED forever
    }
}

#endif
