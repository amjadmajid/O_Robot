/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include "msp.h"

void (*timerA1_task)(void);   // a hook for user defined function

/** -------------- TimerA1_Init ---------------
 * Initialize Timer A1 to run user task periodically
 * Inputs:  task is a pointer to a user define function
 *          period specifies the interrupt frequency (every period/SMCLK sec)
 * Outputs: none
 */
void timerA1_init(void (*task)(void), uint16_t period)
{

    // Hook the user task to the callback
    timerA1_task = task;

    TIMER_A1->CTL &= ~0x0030;  // MC=00b, halt timer A1
    TIMER_A1->CTL |= 0x0200;  // TASSL=10b, use SMCLK (max speed 12MHz)
    TIMER_A1->CTL |= 0x0080;  // ID=10b, Clock /4

    TIMER_A1->CCTL[0] = 0x0010;   //bit4=1b, enable capture/compare interrupt request of CCIFG
                                  //bit8=0b, campare mode
                                  //bit15-14=00b, no capture mode
    TIMER_A1->CCR[0] = (period - 1);  // compare match value
    TIMER_A1->EX0 = 0x0005;		  // divide the clock by 6
    // To set the priority
    // // Look at table 5.1 from the book "Real-Time Interfacing to the MSP432 Microcontoller"
    // // First, select the priority register number from the column "NNIC priority"
    // // Then specify the priority bits position from the columnm "Prioity bits"
    // // set the priority. Notice 00000000b highest, 00100000b (0x20) = priority 1, and 0xe0 = priority 7 (lowest)
    NVIC->IP[2] = (NVIC->IP[2] & 0xff00ffff) | 0x0040000;
    NVIC->ISER[0] = 0x00000400;	  // enable interrupt 10 in NVIC
    TIMER_A1->CTL |= 0x0014;   	  // reset and start Timer A1 in up mode
}

/** --------------timerA1_stop---------------
 * Deactivate the interrupt running a user task periodically.
 * Inputs: none
 * Outputs: none
 */
void timerA1_stop(void)
{
    TIMER_A1->CTL &= ~0x0030;       // halt Timer A0
    NVIC->ICER[0] = 0x00000400; 	// disable TimerA1 interrupt 
}

/** ------------TA1_0_IRQHandler------------
 * Is a pre-specified function name used to handle interrupts, after configuring calls timerA1_task for the interupt.
 * Inputs: none
 * Outputs: none
 */
void TA1_0_IRQHandler(void)
{
    TIMER_A1->CCTL[0] &= ~0x0001; // clear the interrupt flag
    (*timerA1_task)();
}

