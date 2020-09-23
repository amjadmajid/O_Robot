#include <stdint.h>
#include <stdio.h>

#include "msp.h"
#include "initialization.h"
#include "differentialRobot.h"
#include "goToGoal.h"
#include "motor.h"
#include "timer_A1.h"
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




void main(void)
 {
    // initialize the clock, buttons to go, and UART for debugging
    initialize();
    differential_robot_t* robot = robot_init();

#if 1
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
    enableInterrupts();

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
