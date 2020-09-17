#include <differentialRobot.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "UART0.h"
#include "goToGoal.h"
#include "motor.h"
#include "timer_A1.h"
#include "msp.h"
#include "clock.h"
#include "interruptHandler.h"
#include "avoidObstacles.h"

#define CONTROL_PERIOD 5000
//#define PHATH_LEN 6
//float path[PHATH_LEN][2]= { {0.8,0}, {1.6,0}, {1.6, -0.8}, {1.6,-1.6},  {0,-1.6},  {0,0}  };
//uint8_t pauses[] =        {  1,      0,       1,            0,           0,         0     };

#define PHATH_LEN 2
float path[PHATH_LEN][2]= { {2,0}, {2,2} };
uint8_t pauses[] =        {  0,      0   };

uint8_t * goal_flag = NULL;
uint8_t goal_reached = 0;
uint32_t pause_cntr = 0;

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

    // Debug pins
    P4->DIR |=BIT3;
    P4->OUT &= ~BIT3;

    disableInterrupts();

    differential_robot_t* robot = robot_init();
    clock_init_48MHz();
    UART0_Init();
    press_buttons_to_go();
    enableInterrupts();

#if 0
//--------------avoid obstacles behavior-----------------------

avoid_obstacle_init(robot, CONTROL_PERIOD);

while(1) {
    waitForInterrupt();
}

//--------------Go to goal behavior-----------------------
#else
    // used by the control layer to notify the application layer
    goal_flag = & goal_reached; 
    uint32_t location_cntr = 0;

    go_to_goal_init(path[location_cntr][0] ,path[location_cntr][1] , robot,CONTROL_PERIOD, goal_flag);

    while(1){
      if (goal_reached)
      {
        disableInterrupts();
        location_cntr++;  // increase to set the next goal
        goal_reached = 0;
        enableInterrupts();
        
        // check if the robot has reached the final location (or goal)
        if(location_cntr == PHATH_LEN )
        {
            motor_stop();
            timerA1_stop();
        }
        else
        {
            // check if the robot need to pause before going to the next location
            if (pauses[location_cntr-1])
            {
                while (pause_cntr < 0x3fffff)
                {
                    pause_cntr++;
                }
                pause_cntr=0;
            }
            // go to the next location (or goal)
            go_to_goal_init(path[location_cntr][0] ,path[location_cntr][1] , robot, CONTROL_PERIOD, goal_flag);
        }
      }
      waitForInterrupt();
    }
#endif 

}
