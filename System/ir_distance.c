/*
 * Date  : 29 Jul 2020
 * Author: Amjad Yousef Majid
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "msp.h"
#include "lpf.h"

#define MAXDISTANCE 800

uint32_t left_convert(uint32_t nl){        // returns left distance in mm
  int32_t d = 1195172/(nl-880)+70;
  if (d > MAXDISTANCE){d = MAXDISTANCE;}
  if (d < 0){d=0;}
  return (uint32_t) d;
}

uint32_t center_convert(uint32_t nc){   // returns center distance in mm
    int32_t d = 1195172/(nc-880)+70;
    if (d > MAXDISTANCE){d= MAXDISTANCE;}
    if (d < 0){d=0;}
    return (uint32_t) d;
}

uint32_t right_convert(uint32_t nr){      // returns right distance in mm
    int32_t d = 1195172/(nr-880)+70;
    if (d > MAXDISTANCE){d = MAXDISTANCE;}
    if (d < 0){d=0;}
    return (uint32_t) d;
}

void adc_init_channel_17_12_16(void){
     ADC14->CTL0 &= ~ADC14_CTL0_ENC;        // to allow programming
  while(ADC14->CTL0&0x00010000){};          // wait for BUSY to be zero
  ADC14->CTL0 = ADC14_CTL0_ON | ADC14_CTL0_MSC | ADC14_CTL0_SHT0__32 | ADC14_CTL0_SHT1__32 | ADC14_CTL0_CONSEQ_1 |
                ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_DIV__1 | ADC14_CTL0_SHP ;;          // 4) single, SMCLK, on, disabled, /1, 32 SHM
  ADC14->CTL1 = 0x00000030;                 // ADC14MEM0, 14-bit, ref on, regular power
  ADC14->MCTL[0] = ADC14_MCTLN_INCH_12;
  ADC14->MCTL[1] = ADC14_MCTLN_INCH_16;
  ADC14->MCTL[2] = ADC14_MCTLN_INCH_17 | ADC14_MCTLN_EOS; ;         // 0 to 3.3V, channel 17
  ADC14->IER0 = 0; // no interrupts
  ADC14->IER1 = 0; // no interrupts

  P4->SEL1 |= BIT1;
  P4->SEL0 |= BIT1;
  P9->SEL1 |= BIT0;
  P9->SEL0 |= BIT0;
  P9->SEL1 |= BIT1;
  P9->SEL0 |= BIT1;

  ADC14->CTL0 |= ADC14_CTL0_ENC;         // 9) enable
}

volatile uint32_t  debug_reg = 0;
void read_adc_17_12_16(uint32_t *ch17, uint32_t *ch12, uint32_t *ch16){
    while(ADC14->CTL0&0x00010000){};    // wait for BUSY to be zero

      ADC14->CTL0 |=  ADC14_CTL0_SC;
      while(debug_reg != 0x07){
          debug_reg = ADC14->IFGR0;
      };
      *ch12 = ADC14->MEM[0];      // return result 0 to 16383
      *ch16 = ADC14->MEM[1];
      *ch17 = ADC14->MEM[2];
}

uint32_t *raw_left=NULL;
uint32_t * raw_center=NULL;
uint32_t * raw_right=NULL;
void ir_distances(uint32_t *left, uint32_t * center, uint32_t * right){


    read_adc_17_12_16(raw_left,raw_center,raw_right);
    *left = left_convert(LPF_Calc(*raw_left));          // center is channel 12, P4.1
    *center = center_convert(LPF_Calc2(*raw_center));   // right is channel 17 P9.0
    *right = right_convert(LPF_Calc3(*raw_right));      // left is channel 16, P9.1

}

