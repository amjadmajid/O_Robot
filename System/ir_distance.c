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

uint32_t left_convert(uint32_t nl){        // returns left distance in cm
  uint32_t d = 51982/(nl + -1289);+7;
  if (d > MAXDISTANCE){d = MAXDISTANCE;}
  return d;
}

uint32_t center_convert(uint32_t nc){   // returns center distance in mm
  uint32_t d = 51982/(nc + -1289);+7;
  if (d > MAXDISTANCE){d = MAXDISTANCE;}
  return d;
}

uint32_t right_convert(uint32_t nr){      // returns right distance in mm
  uint32_t d = 51982/(nr + -1289);+7;
  if (d > MAXDISTANCE){d = MAXDISTANCE;}
  return d;
}

void adc_init_channel_17_12_16(void){
     ADC14->CTL0 &= ~ADC14_CTL0_ENC;        // to allow programming
  while(ADC14->CTL0&0x00010000){};          // wait for BUSY to be zero
  ADC14->CTL0 = ADC14_CTL0_ON | ADC14_CTL0_MSC | ADC14_CTL0_SHT0__32 | ADC14_CTL0_SHT1__32 | ADC14_CTL0_CONSEQ_1 |
                ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_DIV__1 | ADC14_CTL0_SHP ;          //  single, SMCLK, on, disabled, /1, 32 SHM
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

  ADC14->CTL0 |= ADC14_CTL0_ENC;         // enable
}

volatile uint32_t  debug_reg = 0;
volatile uint32_t adc14_mem_0 =0;
volatile uint32_t adc14_mem_1 =0;
volatile uint32_t adc14_mem_2 =0;
void read_adc_17_12_16(){
    while(ADC14->CTL0&0x00010000){};    // wait for BUSY to be zero

      ADC14->CTL0 |=  ADC14_CTL0_SC;
      while(debug_reg != 0x07){
          debug_reg = ADC14->IFGR0;
      };
      adc14_mem_0 = ADC14->MEM[0];      // return result 0 to 16383
      adc14_mem_1 = ADC14->MEM[1];
      adc14_mem_2 = ADC14->MEM[2];
}


void ir_distances(uint32_t *left, uint32_t * center, uint32_t * right){

    read_adc_17_12_16();
    *left = left_convert(adc14_mem_0);         // left is channel 12, P4.1
    *center = center_convert(adc14_mem_2);     // center is channel 17 P9.0
    *right = right_convert(adc14_mem_1);       // right is channel 16, P9.1

}

