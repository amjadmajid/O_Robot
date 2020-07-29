/**
 *@brief:  low level DC motor power control
 *@Date:   14 May 2020
 *@Author: Amjad Yousef Majid
 */


#include <stdint.h>
#include "msp.h"

#define LEFT_MOTOR_POWER      	TIMER_A0->CCR[3]
#define RIGHT_MOTOR_POWER     	TIMER_A0->CCR[4]
#define PWM_SIGNAL_PERIOD  		TIMER_A0->CCR[0]

/** ------------pwm_init------------
 * Assuming the MCLK=48MHz which causes SMCLK=12MHz
 * Timer_A counts up to the `period` (TA0CCR0) and down to zero. 
 * Output signal is toggled when Timer_A matches the duty cycle (TA0CCR3 or TA0CCR4).
 * The PWM signals are outputted on P2.6 and P2.7
 */
void pwm_init(uint16_t period, uint16_t dutyCycle){

  if (dutyCycle >= period) return; //invalid duty cycles

  // selecting timer functionality 
  P2->SEL0 |=(BIT6+BIT7);
  P2->SEL1 &=~(BIT6+BIT7);
  //makes it output
  P2->DIR |=(BIT6+BIT7);
  
  // configure Timer_A0 coutner
  TIMER_A0->CCTL[0] = 0x0080;  				        //CCI0 toggle (has no effect)
  TIMER_A0->CCR[0]  = period;
  TIMER_A0->EX0     = 0x0000;  				        //divide by 1

  // configure captuer and control submodules 3 and 4  
  TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_2; 		//CCR3 toggle/reset
  LEFT_MOTOR_POWER  = dutyCycle;
  TIMER_A0->CCTL[4] = TIMER_A_CCTLN_OUTMOD_2;  		//CCR4 toggle/reset
  RIGHT_MOTOR_POWER = dutyCycle;
  
  TIMER_A0->CTL     =  TIMER_A_CTL_MC__UPDOWN | TIMER_A_CTL_ID__8 \
   					   | TIMER_A_CTL_SSEL__SMCLK;    
}

/**
 * Set the duty cycle for the left motor
 */
void set_left_duty_cycle(uint16_t dutyCycle){
  if(dutyCycle >= PWM_SIGNAL_PERIOD) return; //invalid duty cycles
  LEFT_MOTOR_POWER = dutyCycle;
}

/**
 * Set the duty cycle for the right motor
 */
void set_right_duty_cycle(uint16_t dutyCycle){
  if(dutyCycle >= PWM_SIGNAL_PERIOD ) return; //invalid duty cycles
  RIGHT_MOTOR_POWER = dutyCycle;
}
