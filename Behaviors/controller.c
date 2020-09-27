#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "interruptHandler.h"
#include "tachometer.h"
#include "differentialRobot.h"
#include "msp.h"
#include "motor.h"
#include "timer_A1.h"
#include "ir_distance.h"
#include "lpf.h"
#include "UART0.h"
#include "pwm.h"
#include "goToGoal.h"
#include "avoidObstacles.h"

// the linear velocity amplified by 100 for integer math purpose 
#define LINEAR_VELOCITY 35000
#define DUTY_CYCLE 3750

float _x_goal;
float _y_goal;
uint8_t * _goal_flag;

float     theta_goal;
uint32_t  linear_velocity = LINEAR_VELOCITY;
int32_t   left_duty_cycle;
int32_t   right_duty_cycle;
differential_robot_t * _robot;

void controller();

void leftTachometer(void){
  int32_t tks = _robot->left->tachometer->ticks;
  if(left_duty_cycle >= 0){
      tks++;
  }else{
    tks--;
  }
  _robot->left->tachometer->ticks = tks;
}

void rightTachometer(void){
    int32_t tks = _robot->right->tachometer->ticks;
    if(right_duty_cycle >= 0){
        tks++;
    }else{
        tks--;
    }
    _robot->right->tachometer->ticks = tks;
}

void duty_check(int32_t *left_duty_cycle, int32_t *right_duty_cycle ){
    // control the max and the min of the duty cycle
    if ( *right_duty_cycle > 11000)  *right_duty_cycle = 11000;
    if ( *right_duty_cycle < -11000) *right_duty_cycle = -11000;
    if ( *left_duty_cycle > 11000)   *left_duty_cycle = 11000;
    if ( *left_duty_cycle < -11000)  *left_duty_cycle = -11000;
}


void controller_init(float x_g, float y_g, differential_robot_t * robot_pt, uint32_t p, uint8_t * goal_flag)
{
    _x_goal =  x_g ;
    _y_goal = y_g;
    _goal_flag = goal_flag;

    _robot = robot_pt;

    motor_init();
    tachometer_init(&leftTachometer, &rightTachometer);
    pwm_init(15000, 0);
    enableInterrupts();
    timerA1_init(&controller, p);

    adc_init_channel_17_12_16();

    //initialize the ADC for the IR distance sensor
    uint32_t *init_left=NULL;
    uint32_t * init_center=NULL;
    uint32_t * init_right=NULL;

    read_adc_17_12_16(init_left,init_center,init_right);

    //initialize the Low Pass Filters for the ir distance sensors
    LPF_Init(*init_left,32);     // P9.0/channel 17 
    LPF_Init2(*init_center,32);    // P5.2/channel 12
    LPF_Init3(*init_right,32);    // P9.1/channel 16

    // initialize debug arrays
    //   uint16_t i;
    //   for(i = 0; i < LEN_OA_DEBUG; i++){
    //       theta_current[i]=0;
    //       theta_goal[i]=0;
    //   }
}


void controller()
{

  //    updating the IR distance measurements
  ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

  // if ir distance is greater than 50 cm 
      // run avoid abstacles controller 
  // else 
      // run go to goal controller 

  if ( _robot->ir_distance->ir_left > 500 && 
       _robot->ir_distance->ir_center > 500 && 
       _robot->ir_distance->ir_right > 500)
  {
     go_to_goal_controller();
  }else{
    avoid_obstacle_controller();
}
}
