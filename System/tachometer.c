/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>

#include "msp.h"
#include "interruptHandler.h"
#include "tachometer.h"
#include "UART1.h"

#define CONTROL_PERIOD 5000

void (*taskLeft)(void);
void (*taskRight)(void);

/** ------------tachometer_init------------
 * Setup the connections between the MCU and the tachometers using interrupts.
 * Inputs: userTaskLeft is the task to perform when the left tachometer gives an interrupt.
 userTaksRight is the task to perform when the right tachometer gives an interrupt.
 * Outputs: none
 */
void tachometer_init(void (*userTaskLeft)(void), void (*userTaskRight)(void))
{
    long sr;
    sr = startCritical();

    //setup the hooks for the tachometers ISRs
    taskLeft = userTaskLeft;
    taskRight = userTaskRight;

    // configure pins 10.5 and 10.4 as input and enable interrupts using timer a3
    P10->SEL0 |= ~(TACHORIGHT | TACHOLEFT);
    P10->SEL1 &= ~(TACHORIGHT | TACHOLEFT);
    P10->DIR &= ~(TACHORIGHT | TACHOLEFT);
//    P10->OUT &= ~(TACHORIGHT | TACHOLEFT);
    P10->IES &= ~(TACHORIGHT | TACHOLEFT);
    P10->IFG &= ~(TACHORIGHT | TACHOLEFT);
    P10->IE |= (TACHORIGHT | TACHOLEFT);

    TIMER_A3->CTL &= ~(BIT1 | BIT2);  // MC=00b, halt timer A1
    TIMER_A3->CTL |= BIT9;  // TASSL=10b, use SMCLK (max speed 12MHz)
    TIMER_A3->CTL &= ~BIT8;
    TIMER_A3->CTL |= BIT7;  // ID=10b, Clock /4
    TIMER_A3->CTL &= ~BIT6;


    TIMER_A3->CTL = 0x0200;
    // ^->

    TIMER_A3->CCR[0] = (CONTROL_PERIOD - 1);  // compare match value
    TIMER_A3->CCR[1] = (CONTROL_PERIOD - 1);  // compare match value

    // ^->

    TIMER_A3->CCTL[0] |= BIT4 | BIT8 | BIT(11) | BIT(14) | BIT(15); // arms P10.4
    TIMER_A3->CCTL[0] &= ~BIT1 & ~BIT(12) & ~BIT(13);
    TIMER_A3->CCTL[1] |= BIT4 | BIT8 | BIT(11) | BIT(14) | BIT(15); // arms P10.5
    TIMER_A3->CCTL[1] &= ~BIT1 & ~BIT(12) & ~BIT(13);

    TIMER_A3->EX0 = 0x0005;       // divide the clock by 6


    NVIC->IP[3] &= (NVIC->IP[3] & ~BIT(22) & ~BIT(23) & ~BIT(30) & ~BIT(31)) | BIT(21) | BIT(29);
    NVIC->ISER[0] |= (TA3_0_IRQn | TA3_N_IRQn);

    TIMER_A3->CTL = (TASSEL_2 + ID_3 + MC_1 + TAIE);
//    TIMER_A3->CTL |= 0x0014;      // reset and start Timer A3 in up mode
    endCritical(sr);
}

/** ------------PORT5_IRQHandler------------
 * Is a pre-specified function name used to handle interrupts, after configuring calls taskLeft and taskRight for their corresponding interrupts caused by the tachometers.
 * Inputs: none
 * Outputs: none
 */
void TA3_0_IRQHandler(void)
{
    TIMER_A3->CCTL[0] &= ~BIT1;
    UART1_OutString("RIGHT");
    // execute user hook for right tachometer
    (*taskRight)();
}

void TA3_N_IRQHandler(void)
{
    TIMER_A3->CCTL[1] &= ~BIT1;
    UART1_OutString("LEFT");
    // execute user hook for left tachometer
    (*taskLeft)();
}
