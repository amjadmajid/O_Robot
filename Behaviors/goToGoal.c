/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <differentialRobot.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "msp.h"
#include "timer_A1.h"
#include "motor.h"



uint32_t time;
int32_t left_duty_cycle;
int32_t right_duty_cycle;
void duty_check(){
    // control the max and the min of the duty cycle
    if (right_duty_cycle > 11000) right_duty_cycle = 11000;
    if (right_duty_cycle < -11000) right_duty_cycle = -11000;
    if (left_duty_cycle > 11000) left_duty_cycle = 11000;
    if (left_duty_cycle < -11000) left_duty_cycle = -11000;

}

//******************************************************
double E_i=0;
float K_i = 0;
float K_p = 566;

float _x_goal;
float _y_goal;
differential_robot_t * _robot;


void go_to_goa_controller(){

//    int32_t v = sqrt((x_goal * x_goal)+(y_goal * y_goal ))  ; // reach the goal in 10 s
    int32_t v = 200;

    float delta_x = _x_goal - _robot->pose->x;
    float delta_y = _y_goal - _robot->pose->y;


    double theta_goal = atan2(delta_y, delta_x);

    double heading_error = theta_goal - _robot->pose->theta;
    double err = atan2(sin(heading_error), cos(heading_error));
    // float err = atan2(sin(theta_goal), cos(theta_goal));
    E_i +=err;
    double U_i =  (K_i * E_i);
    double U_p =  (K_p * err);
    double w = U_p + U_i;

    left_duty_cycle = (2 * v - w * _robot->base_len)/(2 * _robot->left->radius );
    right_duty_cycle = (2 * v + w * _robot->base_len)/(2 * _robot->right->radius );

//    uint32_t static printf_flag=8;
//    printf_flag--;
//    if(printf_flag==0){
//    printf("tg=%.4f e=%.4f w=%d ldc=%d rdc=%d\n",theta_goal, err, w, left_duty_cycle,right_duty_cycle);
//    printf_flag=8;
//    }

    duty_check();

    motor_forward(right_duty_cycle, left_duty_cycle);

    robot_position_update(_robot);

//    uint32_t static printf_flag=10;
//    printf_flag--;
//    if(printf_flag==0){
//    printf("x=%.4f y=%.4f theta=%.4f\n",robot.pose->x,robot.pose->y, robot.pose->theta );
//    printf_flag=10;
//    }


    float x_err = (float) fabs((_robot->pose->x - _x_goal));
    float y_err = (float) fabs((_robot->pose->y - _y_goal));

//        uint32_t static printf_flag=4;
//        printf_flag--;
//        if(printf_flag==0){
//        printf("x=%d y=%d\n", x_err,y_err);
//        printf_flag=4;
//        }

    float static back_flag = 0;
    if ( x_err < .05 && y_err <.05 ){

        _x_goal = 1;
        _y_goal = -1;

//        if(back_flag==1){
//            _x_goal = -.5;
//            _y_goal = -.5;
//        }

        if (back_flag==1){
            motor_stop();
            timerA1_stop();
            printf("Arrived");
            __no_operation();
        }

        back_flag++;
    }

    if(time == 150000 ){
        motor_stop();
        timerA1_stop();
        printf("Not arrived");
        __no_operation();
    }
    time++;
}

void go_to_goal_init(float x_g, float y_g, differential_robot_t * robot_pt, uint16_t p){
    _x_goal = x_g;
    _y_goal = y_g;
    _robot = robot_pt;
    timerA1_init(&go_to_goa_controller, p);
    time=0;
}

