/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>

#include "msp.h"
#include "interruptHandler.h"

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
    //setup the hooks for the tachometers ISRs
    taskLeft = userTaskLeft;
    taskRight = userTaskRight;

    // configure pins 5.0 and 5.2 as input and enable interrupt
    P5->SEL0 &= ~(BIT0 + BIT2 );
    P5->SEL1 &= ~(BIT0 + BIT2 );
    P5->DIR &= ~(BIT0 + BIT2 );

    P5->IFG &= ~(BIT0 + BIT2 );
    P5->IES &= ~(BIT0 + BIT2 );
    P5->IE |= (BIT0 + BIT2 );

    NVIC->IP[9] &= (NVIC->IP[9] & 0x00FFFFFF) | 0x20000000;
    NVIC->ISER[1] = 1 << (PORT5_IRQn & 31);

}

/** ------------PORT5_IRQHandler------------
 * Is a pre-specified function name used to handle interrupts, after configuring calls taskLeft and taskRight for their corresponding interupts caused by the tachometers.
 * Inputs: none
 * Outputs: none
 */
void PORT5_IRQHandler(void)
{
    if (P5->IFG & BIT0)
    {
        P5->IFG &= ~BIT0;
        // execute user hook for right tachometer
        (*taskRight)();
    }
    if (P5->IFG & BIT2)
    {
        P5->IFG &= ~BIT2;
        // execute user hook for left tachometer
        (*taskLeft)();
    }
}
