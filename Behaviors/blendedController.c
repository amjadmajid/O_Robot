/*
 * blendingController.c
 *
 *  Created on: 28 Sep 2020
 *      Author: Amjad Yousef Majid
 */

#include <math.h>
#include <stdint.h>

#include "vectorTranslation.h"
#include "differentialRobot.h"
#include "motor.h"

extern void duty_check(int32_t *left_duty_cycle, int32_t *right_duty_cycle );

extern float _x_goal;
extern float _y_goal;
extern differential_robot_t * _robot;
extern uint32_t  linear_velocity;
extern int32_t   left_duty_cycle;
extern int32_t   right_duty_cycle;


#define LEN_BC_DEBUG 300
uint16_t cntr_bc_debug=0;
uint32_t ir_left_bc[LEN_BC_DEBUG]={0};
uint32_t ir_right_bc[LEN_BC_DEBUG]={0};
//int32_t left_duty_cycle_bc[LEN_BC_DEBUG] = {0};
//int32_t right_duty_cycle_bc[LEN_BC_DEBUG]= {0};
//float theta_current[LEN_BC_DEBUG];
//float theta_goal[LEN_BC_DEBUG];
//float w_bc[LEN_BC_DEBUG];


double static E_i=0;
float static K_i = 2.6;
float static K_p = 330;



void blended_controller()
{

//    if(cntr_bc_debug < LEN_BC_DEBUG){
//        ir_left_bc[cntr_bc_debug] = _robot->ir_distance->ir_left;
//        ir_right_bc[cntr_bc_debug]   = _robot->ir_distance->ir_right;
//        cntr_bc_debug++;
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

float x_o = x_dir - _robot->pose->x;
float y_o = y_dir - _robot->pose->y; 

float x_g = _x_goal - _robot->pose->x;
float y_g = _y_goal - _robot->pose->y;

// vector normalization 
float denominator = sqrt( (x_o * x_o) + (y_o*y_o) );
float x_o_n = x_o / denominator;
float y_o_n = y_o / denominator;

denominator = sqrt( (x_g * x_g) + (y_g * y_g) );
float x_g_n = x_g / denominator;
float y_g_n = y_g / denominator;

float alpha = 0.3; 

x_dir = alpha * x_g_n + (1-alpha) * x_o_n;
y_dir = alpha * y_g_n + (1-alpha) * y_o_n;

//  P1->OUT &=~BIT5;
//  P1->OUT |=BIT5;
//  printf( "%d %d\n", (uint16_t)x_o, (uint16_t)y_o );
//  P1->OUT |=BIT5;
//  P1->OUT &=~BIT5;

  float theta_g =   atan2(y_dir , x_dir);

//  P1->OUT &=~BIT5;
//  P1->OUT |=BIT5;
//  printf( "%d %d\n", (uint16_t)(theta_g * 100), (uint16_t)( _robot->pose->theta * 100) );
//  P1->OUT |=BIT5;
//  P1->OUT &=~BIT5;


  float heading_error = theta_g - _robot->pose->theta;
  float err = atan2(sin(heading_error), cos(heading_error));

  E_i +=err;
  float U_i =  (K_i * E_i);
  float U_p =  (K_p * err);
  float w = U_p + U_i;

//  if(cntr_bc_debug < LEN_BC_DEBUG){
//      theta_current[cntr_bc_debug] = _robot->pose->theta;
//      theta_goal[cntr_bc_debug]   = theta_g;
//      w_bc[cntr_bc_debug] = w;
//      cntr_bc_debug++;
//  }

  left_duty_cycle = (linear_velocity - w * 14)/7 ;
  right_duty_cycle = (linear_velocity + w * 14)/7 ;

//  if(cntr_bc_debug < LEN_BC_DEBUG){
//      left_duty_cycle_bc[cntr_bc_debug] = left_duty_cycle;
//      right_duty_cycle_bc[cntr_bc_debug]   = right_duty_cycle;
//      cntr_bc_debug++;
//  }

  duty_check(&left_duty_cycle, &right_duty_cycle);

  motor_forward(right_duty_cycle, left_duty_cycle);
  robot_position_update(_robot);
}






