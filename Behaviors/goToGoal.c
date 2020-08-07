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

// the linear velocity amplified by 100 to integer math purpose 
#define LINEAR_VELOCITY 30000

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
float K_i = 8.4;
float K_p = 820;

float _x_goal;
float _y_goal;
differential_robot_t * _robot;

uint32_t ir_left[500]={0};

//    int32_t v = sqrt((x_goal * x_goal)+(y_goal * y_goal ))  ; // reach the goal in 10 s
//    int32_t v = 150;
//int32_t linear_velocity = 30;// v * 2
//float meter_per_rev =  0.07; // RADIUS * 2

float theta_goal;
uint32_t linear_velocity = LINEAR_VELOCITY;

void go_to_goal_controller(){

    if ( _robot->left->tachometer->ticks > 150 || _robot->right->tachometer->ticks > 150 )
    {
        time=150000;
    }

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

//    left_duty_cycle = (linear_velocity - w * L )/(meter_per_rev);
//    right_duty_cycle = (linear_velocity + w * L )/(meter_per_rev);

    // increase linear velocity from gradually from 15000 to 35000
    // if the controller is fired every 10 ms then the robot reaches 3000 after 4 second
//    if (time < 400)
//    {
//        linear_velocity +=50;
//    }

    left_duty_cycle = (linear_velocity - w * 14)/7 ;
    right_duty_cycle = (linear_velocity + w * 14)/7 ;


//    uint32_t static printf_flag=10;
//    if(printf_flag==10){
//    printf("tg=%.4f e=%.4f w=%.4f ldc=%d rdc=%d\n",theta_goal, err, w, left_duty_cycle,right_duty_cycle);
//    printf("tg=%.4f tr=%.4f \n",err,_robot->pose->theta);
//      printf("e=%.4f te=%.4f\n", err, heading_error);
//    printf_flag=0;
//    }
//    printf_flag++;

    duty_check();

    motor_forward(right_duty_cycle, left_duty_cycle);
    robot_position_update(_robot);

//    uint32_t static printf_flag=10;
//    printf_flag--;
//    if(printf_flag==0){
//    printf("x=%.4f y=%.4f theta=%.4f\n",_robot.pose->x,_robot.pose->y,_robot.pose->theta );
//    printf_flag=10;
//    }


    float x_err = (float) fabs((_robot->pose->x - _x_goal));
    float y_err = (float) fabs((_robot->pose->y - _y_goal));

//        uint32_t static printf_flag=4;
//        printf_flag--;
//        if(printf_flag==10){
//        printf("x=%.4f y=%.4f\n", x_err,y_err);
//        printf_flag=0;
//        }

//    float static back_flag = 0;
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
//    ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

    // Debugging
//    if (time < 1500 && time > 1000  ){
//        ir_left[time-1000] = _robot->ir_distance->ir_left;
//    }

}

void go_to_goal_init(float x_g, float y_g, differential_robot_t * robot_pt, uint32_t p){
    _x_goal =  x_g ;// (x_g * 10)/16 ;  // todo make it discrete math
    _y_goal = y_g;
    _robot = robot_pt;


    timerA1_init(&go_to_goal_controller, p);

    // initialize the ADC for the IR distance sensor
//    uint32_t *init_left=NULL;
//    uint32_t * init_center=NULL;
//    uint32_t * init_right=NULL;

//    adc_init_channel_17_12_16();
//    read_adc_17_12_16(init_left,init_center,init_right);

    // initialize the Low Pass Filters for the ir distance sensors
//    LPF_Init(*init_left,32);     // P9.0/channel 17
//    LPF_Init2(*init_center,32);    // P4.1/channel 12
//    LPF_Init3(*init_right,32);    // P9.1/channel 16

    time=0;
}

