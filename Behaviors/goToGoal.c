/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */


#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "differentialRobot.h"
#include "msp.h"
#include "timer_A1.h"
#include "motor.h"
#include "ir_distance.h"
#include "lpf.h"



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


double E_i=0;
float K_i = 0;
float K_p = 86;

float _x_goal;
float _y_goal;
differential_robot_t * _robot;

uint32_t ir_left[500]={0};

void go_to_goal_controller(){

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

//        _x_goal = 1;
//        _y_goal = -1;

//        if(back_flag==1){
//            _x_goal = -.5;
//            _y_goal = -.5;
//        }

//        if (back_flag==1){
            motor_stop();
            timerA1_stop();
            printf("Arrived");
            __no_operation();
//        }

//        back_flag++;
    }

    if(time == 150000 ){
        motor_stop();
        timerA1_stop();
        printf("Not arrived");
        __no_operation();
    }
    time++;

    // updating the IR distance measurements
    ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

    // Debugging
    if (time < 1500 && time > 1000  ){
        ir_left[time-1000] = _robot->ir_distance->ir_left;
    }

}

void go_to_goal_init(float x_g, float y_g, differential_robot_t * robot_pt, uint16_t p){
    _x_goal = x_g;
    _y_goal = y_g;
    _robot = robot_pt;
    timerA1_init(&go_to_goal_controller, p);

    // initialize the ADC for the IR distance sensor
    uint32_t *init_left=NULL;
    uint32_t * init_center=NULL;
    uint32_t * init_right=NULL;

    adc_init_channel_17_12_16();
    read_adc_17_12_16(init_left,init_center,init_right);

    // initialize the Low Pass Filters for the ir distance sensors
    LPF_Init(*init_left,32);     // P9.0/channel 17
    LPF_Init2(*init_center,32);    // P4.1/channel 12
    LPF_Init3(*init_right,32);    // P9.1/channel 16

    time=0;
}

