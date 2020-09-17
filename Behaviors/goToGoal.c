/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "tachometer.h"
#include "differentialRobot.h"
#include "msp.h"
#include "timer_A1.h"
#include "motor.h"
#include "ir_distance.h"
#include "lpf.h"
#include "motor.h"
#include "pwm.h"
#include "UART0.h"


void go_to_goal_controller();

// the linear velocity amplified by 100 for integer math purpose 
#define LINEAR_VELOCITY 35000
#define DUTY_CYCLE 3750

uint32_t time;
int32_t left_duty_cycle;
int32_t right_duty_cycle;
float _x_goal;
float _y_goal;
uint8_t * _goal_flag=NULL;
differential_robot_t * _robot;

double E_i=0;
float K_i = 2.2;
float K_p = 630;

uint32_t ir_left[200]={0};
uint32_t ir_right[200]={0};
uint32_t ir_center[200]={0};
float theta_goal;
uint32_t linear_velocity = LINEAR_VELOCITY;

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

void duty_check(){
    // control the max and the min of the duty cycle
    if (right_duty_cycle > 11000) right_duty_cycle = 11000;
    if (right_duty_cycle < -11000) right_duty_cycle = -11000;
    if (left_duty_cycle > 11000) left_duty_cycle = 11000;
    if (left_duty_cycle < -11000) left_duty_cycle = -11000;
}

void go_to_goal_init(float x_g, float y_g, differential_robot_t * robot_pt, uint32_t p, uint8_t * goal_flag){
    _x_goal =  x_g ;// (x_g * 10)/16 ;  // todo make it discrete math
    _y_goal = y_g;
    _robot = robot_pt;
    _goal_flag = goal_flag;

    motor_init();
    tachometer_init(&leftTachometer, &rightTachometer);
    pwm_init(15000, 0);
    timerA1_init(&go_to_goal_controller, p);

    // UART0_Init();
    // adc_init_channel_17_12_16();

    //initialize the ADC for the IR distance sensor
//    uint32_t *init_left=NULL;
//    uint32_t * init_center=NULL;
//    uint32_t * init_right=NULL;
//    read_adc_17_12_16(init_left,init_center,init_right);

    //initialize the Low Pass Filters for the ir distance sensors
//    LPF_Init(*init_left,32);     // P9.0/channel 17
//    LPF_Init2(*init_center,32);    // P4.1/channel 12
//    LPF_Init3(*init_right,32);    // P9.1/channel 16

    time=0;
}

void go_to_goal_controller(){

//    P4->OUT |=BIT3;
//    P4->OUT &=~BIT3;

    float delta_x = _x_goal - _robot->pose->x;
    float delta_y = _y_goal - _robot->pose->y;
    theta_goal = (float) atan2(delta_y, delta_x);

    float heading_error = theta_goal - _robot->pose->theta;
    float err = (float) atan2(sin(heading_error), cos(heading_error));
    // float err = atan2(sin(theta_goal), cos(theta_goal));
    E_i +=err;
    float U_i =  (K_i * E_i);
    float U_p =  (K_p * err);
    float w = U_p + U_i;

//    if (time < 400)
//    {
//        linear_velocity +=50;
//    }

    // left_duty_cycle = (linear_velocity - w * L )/(meter_per_rev);
    // right_duty_cycle = (linear_velocity + w * L )/(meter_per_rev);


    left_duty_cycle = (linear_velocity - w * 14)/7 ;
    right_duty_cycle = (linear_velocity + w * 14)/7 ;

//    UART0_OutUDec((uint32_t) left_duty_cycle );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) right_duty_cycle );
//    UART0_OutChar('\n'); UART0_OutChar('\r');

    duty_check();

    motor_forward(right_duty_cycle, left_duty_cycle);
    robot_position_update(_robot);

    float x_err = (float) fabs((_robot->pose->x - _x_goal));
    float y_err = (float) fabs((_robot->pose->y - _y_goal));

    UART0_OutUDec((uint32_t) (x_err  * 100) );
    UART0_OutChar(' ');
    UART0_OutUDec((uint32_t) (y_err *100) );
    UART0_OutChar('\n'); UART0_OutChar('\r');


    if ( x_err < .05 && y_err <.05 ){
        * _goal_flag = 1; // goal is reached
            motor_stop();
            timerA1_stop();
    }

    if(time == 150000 ){
        motor_stop();
        timerA1_stop();
        __no_operation();
    }
    time++;

//    updating the IR distance measurements
   // ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

//    Debugging
   // if (time < 500 && time >= 300  ){
   //     ir_left[time-300] = _robot->ir_distance->ir_left;
   //     ir_center[time-300] = _robot->ir_distance->ir_center;
   //     ir_right[time-300] = _robot->ir_distance->ir_right;
   // }
   // if (time ==500)
   // {
   //     __no_operation();
   // }

//   UART0_OutUDec(_robot->ir_distance->ir_left); UART0_OutUDec(_robot->ir_distance->ir_center); UART0_OutUDec(_robot->ir_distance->ir_right);
//   UART0_OutChar('\n'); UART0_OutChar('\r');
}



