#include <differentialRobot.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "goToGoal.h"
#include "msp.h"
#include "clock.h"
#include "interruptHandler.h"

#define CONTROL_PERIOD 5000
#define X_GOAL 0
#define Y_GOAL 0


uint8_t read_buttons(){
    // P1->IN works in a negative logic fashion
    return (~(P1->IN) & BIT1 && ~(P1->IN) & BIT4 );
}

void press_buttons_to_go(void){
    P1->SEL0 &= ~(BIT1+BIT4);
    P1->SEL1 &= ~(BIT1+BIT4);
    P1->DIR &= ~(BIT1+BIT4);
    P1->REN |=  (BIT1+BIT4);
    P1->OUT |=  (BIT1+BIT4); // pull-up resistors

    while( read_buttons() == 0x00 ) ; // wait for press
    while( read_buttons() != 0x00 ) ; // wait for release
}


void main(void)
 {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    disableInterrupts();

    differential_robot_t* robot = robot_init();
    clock_init_48MHz();
    press_buttons_to_go();
    enableInterrupts();
    go_to_goal_init(X_GOAL ,Y_GOAL , robot,CONTROL_PERIOD);

    while(1){
      waitForInterrupt();
    }
}
