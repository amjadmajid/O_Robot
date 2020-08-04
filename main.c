#include <differentialRobot.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "goToGoal.h"
#include "msp.h"
#include "clock.h"
#include "pwm.h"
#include "motor.h"
#include "tachometer.h"
#include "timer_A1.h"
#include "interruptHandler.h"

#define MAX_IR 800 // max ir distances in mm
#define DUTY_CYCLE 3750
#define X_GOAL -1
#define Y_GOAL -1
#define CONTROL_PERIOD 2500

// initialize the robot data structure
tachometer_t right_tachometer = {0,0,0};
tachometer_t left_tachometer = {0,0,0};
pose_t pose = {0,0,0};  // initial position of the robot
ir_distance_t ir_distance = {MAX_IR,MAX_IR,MAX_IR};

wheel_t right_wheel;
wheel_t left_wheel;
differential_robot_t robot;

void leftTachometer(void){
  uint32_t tks = left_tachometer.ticks;
  tks++;
  left_tachometer.ticks = tks;
}

void rightTachometer(void){
  uint32_t tks = right_tachometer.ticks;
  tks++;

  right_tachometer.ticks = tks;
}

uint8_t read_buttons(){
    // P1->IN works in a negative logic fashion
    return (~(P1->IN) & (BIT1+BIT4));
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

void robot_init()
{
    right_wheel.radius = RADIUS;
    right_wheel.ticks_per_rev = 0;
    right_wheel.tachometer = &right_tachometer;
    left_wheel.radius = RADIUS;
    left_wheel.ticks_per_rev = 0;
    left_wheel.tachometer = &left_tachometer;
    robot.base_len = L;
    robot.right = &right_wheel;
    robot.left = &left_wheel;
    robot.pose = &pose;
    robot.ir_distance = &ir_distance;
}

void main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;     // stop watchdog timer

    disableInterrupts();

    robot_init();
    clock_init_48MHz();

    press_buttons_to_go();

    motor_init();
    motor_forward(DUTY_CYCLE, DUTY_CYCLE);
    tachometer_init(&leftTachometer, &rightTachometer);
//    go_to_goal_init(X_GOAL - (.1 * X_GOAL),Y_GOAL + (.1 * Y_GOAL), &robot,CONTROL_PERIOD);
    go_to_goal_init(X_GOAL ,Y_GOAL , &robot,CONTROL_PERIOD);


    enableInterrupts();


    pwm_init(15000, DUTY_CYCLE);

    while(1){
      waitForInterrupt();
    }
}
