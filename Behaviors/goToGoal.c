/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "tachometer.h"
#include "msp.h"
#include "timer_A1.h"
#include "motor.h"
#include "lpf.h"
#include "pwm.h"
#include "UART0.h"
#include "differentialRobot.h"

extern void duty_check(int32_t *left_duty_cycle, int32_t *right_duty_cycle);

extern float _x_goal;
extern float _y_goal;
extern uint8_t *_goal_flag;
extern differential_robot_t *_robot;
extern float theta_goal;
extern uint32_t linear_velocity;
extern int32_t left_duty_cycle;
extern int32_t right_duty_cycle;

float static E_i = 0;
float static K_i = 2.2;
float static K_p = 630;

void go_to_goal_controller()
{

    float delta_x = _x_goal - _robot->pose->x;
    float delta_y = _y_goal - _robot->pose->y;
    theta_goal = atan2(delta_y, delta_x);

    float heading_error = theta_goal - _robot->pose->theta;
    float err = atan2(sin(heading_error), cos(heading_error));

    E_i += err;
    float U_i = (K_i * E_i);
    float U_p = (K_p * err);
    float w = U_p + U_i;

//--------------------------------Acceleration--------------------------------------
// # Todo Can we use acceleration ?

//    if (time < 400)
//    {
//        linear_velocity +=50;
//    }

//  left_duty_cycle = (linear_velocity - w * L )/(meter_per_rev);
//  right_duty_cycle = (linear_velocity + w * L )/(meter_per_rev);
//--------------------------------Acceleration--------------------------------------

    left_duty_cycle = (linear_velocity - w * 14) / 7;
    right_duty_cycle = (linear_velocity + w * 14) / 7;

//    UART0_OutUDec((uint32_t) left_duty_cycle );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) right_duty_cycle );
//    UART0_OutChar('\n'); UART0_OutChar('\r');

    duty_check(&left_duty_cycle, &right_duty_cycle);

    motor_forward(right_duty_cycle, left_duty_cycle);
    robot_position_update(_robot);

    float x_err = fabs((_robot->pose->x - _x_goal));
    float y_err = fabs((_robot->pose->y - _y_goal));

//    UART0_OutUDec((uint32_t) (x_err  * 100) );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) (y_err *100) );
//    UART0_OutChar('\n'); UART0_OutChar('\r');

//    if (x_err < .05 && y_err < .05)
    if (x_err < .05 && y_err < .05)
    {
        *_goal_flag = 1; // goal is reached
        motor_stop();
        timerA1_stop();
    }
}

