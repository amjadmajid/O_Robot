#include <stdint.h>
#include <stddef.h>
#include <math.h>
//#include <stdio.h>

#include "interruptHandler.h"
#include "tachometer.h"
#include "differentialRobot.h"
#include "msp.h"
#include "motor.h"
#include "timer_A1.h"
#include "ir_distance.h"
#include "lpf.h"
#include "UART0.h"
//#include "printf.h"
#include "pwm.h"
#include "vectorTranslation.h"

extern void duty_check(int32_t *left_duty_cycle, int32_t *right_duty_cycle );

extern float _x_goal;
extern float _y_goal;

extern differential_robot_t * _robot;
extern uint32_t  linear_velocity;
extern int32_t   left_duty_cycle;
extern int32_t   right_duty_cycle;


#define LEN_OA_DEBUG 300
uint16_t cntr_oa_debug=0;
uint32_t ir_left_oa[LEN_OA_DEBUG]={0};
uint32_t ir_right_oa[LEN_OA_DEBUG]={0};
//int32_t left_duty_cycle_oa[LEN_OA_DEBUG] = {0};
//int32_t right_duty_cycle_oa[LEN_OA_DEBUG]= {0};
//float theta_current[LEN_OA_DEBUG];
//float theta_goal[LEN_OA_DEBUG];
//float w_oa[LEN_OA_DEBUG];


double static E_i=0;
float static K_i = 8;
float static K_p = 830;



void avoid_obstacle_controller()
{

    //    if(cntr_oa_debug < LEN_OA_DEBUG){
    //        ir_left_oa[cntr_oa_debug] = _robot->ir_distance->ir_left;
    //        ir_right_oa[cntr_oa_debug]   = _robot->ir_distance->ir_right;
    //        cntr_oa_debug++;
    //    }


      // pi/4= 0.785
    vector_2d left_sensor_rf = convert2rf(50,70,  0.785, _robot->ir_distance->ir_left);
    vector_2d center_sensor_rf = convert2rf(70, 0, 0, _robot->ir_distance->ir_center);
    vector_2d right_sensor_rf = convert2rf(50,-70,-0.785, _robot->ir_distance->ir_right);

    vector_2d left_sensor_wf =  convert2wf(left_sensor_rf, _robot->pose->x, _robot->pose->y, _robot->pose->theta);
    vector_2d center_sensor_wf =  convert2wf(center_sensor_rf, _robot->pose->x, _robot->pose->y, _robot->pose->theta);
    vector_2d rihgt_sensor_wf =  convert2wf(right_sensor_rf, _robot->pose->x, _robot->pose->y, _robot->pose->theta);

    float x_dir = left_sensor_wf.x + rihgt_sensor_wf.x + center_sensor_wf.x;
    float y_dir = left_sensor_wf.y + rihgt_sensor_wf.y + center_sensor_wf.y;

    float x_g = x_dir - _robot->pose->x;
    float y_g = y_dir - _robot->pose->y;


    float theta_g =   atan2(y_g , x_g);

//    UART0_OutUDec((uint32_t) (y_dir) );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) (x_dir) );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) (_robot->pose->y * 10) );
//    UART0_OutChar(' ');
//    UART0_OutUDec((uint32_t) (_robot->pose->x * 10) );
    UART0_OutChar(' ');
    UART0_OutUDec((uint32_t) (theta_g * 10) );
    UART0_OutChar('\n'); UART0_OutChar('\r');

    float heading_error = theta_g - _robot->pose->theta;
    float err = atan2(sin(heading_error), cos(heading_error));

    E_i +=err;
    float U_i =  (K_i * E_i);
    float U_p =  (K_p * err);
    float w = U_p + U_i;

    //  if(cntr_oa_debug < LEN_OA_DEBUG){
    //      theta_current[cntr_oa_debug] = _robot->pose->theta;
    //      theta_goal[cntr_oa_debug]   = theta_g;
    //      w_oa[cntr_oa_debug] = w;
    //      cntr_oa_debug++;
    //  }

    left_duty_cycle = (linear_velocity - w * 14)/7 ;
    right_duty_cycle = (linear_velocity + w * 14)/7 ;

    //  if(cntr_oa_debug < LEN_OA_DEBUG){
    //      left_duty_cycle_oa[cntr_oa_debug] = left_duty_cycle;
    //      right_duty_cycle_oa[cntr_oa_debug]   = right_duty_cycle;
    //      cntr_oa_debug++;
    //  }

    duty_check(&left_duty_cycle, &right_duty_cycle);

    motor_forward(right_duty_cycle, left_duty_cycle);
    robot_position_update(_robot);


    float x_err = fabs((_robot->pose->x - _x_goal));
    float y_err = fabs((_robot->pose->y - _y_goal));

    if ( x_err < .05 && y_err <.05 ){
            motor_stop();
            timerA1_stop();
    }
    
}






















