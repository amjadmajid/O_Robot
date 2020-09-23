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

#define LEN_OA_DEBUG 300
uint16_t cntr_oa_debug=0;
uint32_t ir_left_oa[LEN_OA_DEBUG]={0};
uint32_t ir_right_oa[LEN_OA_DEBUG]={0};
//int32_t left_duty_cycle_oa[LEN_OA_DEBUG] = {0};
//int32_t right_duty_cycle_oa[LEN_OA_DEBUG]= {0};
//float theta_current[LEN_OA_DEBUG];
//float theta_goal[LEN_OA_DEBUG];
//float w_oa[LEN_OA_DEBUG];


#define DUTY_CYCLE 3750

// the linear velocity amplified by 100 for integer math purpose
uint32_t static linear_velocity = 35000;
double static E_i=0;
float static K_i = 5.6;
float static K_p = 830;
int32_t left_duty_cycle ;
int32_t right_duty_cycle ;

void avoid_obstacle_controller();

differential_robot_t * _robot;

// ToDo move it to its own utility  file
void static duty_check(){
    // control the max and the min of the duty cycle
    if (right_duty_cycle > 11000) right_duty_cycle = 11000;
    if (right_duty_cycle < -11000) right_duty_cycle = -11000;
    if (left_duty_cycle > 11000) left_duty_cycle = 11000;
    if (left_duty_cycle < -11000) left_duty_cycle = -11000;
}

void static leftTachometer(void){
  int32_t tks = _robot->left->tachometer->ticks;
  if(left_duty_cycle >= 0){
      tks++;
  }else{
    tks--;
  }
  _robot->left->tachometer->ticks = tks;
}

void static rightTachometer(void){
    int32_t tks = _robot->right->tachometer->ticks;
    if(right_duty_cycle >= 0){
        tks++;
    }else{
        tks--;
    }
    _robot->right->tachometer->ticks = tks;
}

void avoid_obstacle_init(differential_robot_t * robot_pt, uint32_t p)
{
     P1->OUT &=~BIT5;
     P1->DIR |=BIT5;

    _robot = robot_pt;
    motor_init();
    tachometer_init(&leftTachometer, &rightTachometer);
    pwm_init(15000, 0);
    enableInterrupts();
    timerA1_init(&avoid_obstacle_controller, p);
    adc_init_channel_17_12_16();

    //initialize the ADC for the IR distance sensor
   uint32_t *init_left=NULL;
   uint32_t * init_center=NULL;
   uint32_t * init_right=NULL;

   read_adc_17_12_16(init_left,init_center,init_right);

    //initialize the Low Pass Filters for the ir distance sensors
   LPF_Init(*init_left,32);     // P9.0/channel 17 
   LPF_Init2(*init_center,32);    // P4.1/channel 12
   LPF_Init3(*init_right,32);    // P9.1/channel 16

   // initialize debug arrays
//   uint16_t i;
//   for(i = 0; i < LEN_OA_DEBUG; i++){
//       theta_current[i]=0;
//       theta_goal[i]=0;
//   }

}

typedef struct 
{
  float x;
  float y;
} vector_2d;

// convert the sensor measurement to the robot frame of reference
vector_2d convert2rf(int32_t x_s, int32_t y_s, float theta, uint32_t ir_distance)
{
  vector_2d sensor_vec;
  float theta_tmp = theta - 0.785;
  if ( theta_tmp < 0.01 && theta_tmp > 0 ) // check if theta is +pi/4
  {
    // cos(+-pi/4) = sin(pi/4) = 0.707;
    float tmp = ((707 * ir_distance)/1000);
    sensor_vec.x = tmp + x_s;
    sensor_vec.y = tmp + y_s;
  }
  else if ( theta_tmp < 0.01  ) // check if theta is -pi/4
  {
     // sin(- pi/4) = -0.707;
    float tmp = ((707 * ir_distance)/1000);
    sensor_vec.x = tmp + x_s;
    sensor_vec.y = y_s - tmp ;
  }else{
    sensor_vec.x = cos(theta) * ir_distance + x_s;
    sensor_vec.y = sin(theta) * ir_distance + y_s;
  }

  return sensor_vec;
}

// convert to the world frame of reference
vector_2d convert2wf(vector_2d robot_sensor, int32_t x_r, int32_t y_r, float theta){

  vector_2d robot_vec;

  robot_vec.x = cos(theta) * robot_sensor.x - sin(theta) * robot_sensor.y + x_r;
  robot_vec.y = sin(theta) * robot_sensor.x + cos(theta) * robot_sensor.y + y_r;

  return robot_vec;
}

void avoid_obstacle_controller()
{
	//    updating the IR distance measurements
  ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

//    if(cntr_oa_debug < LEN_OA_DEBUG){
//        ir_left_oa[cntr_oa_debug] = _robot->ir_distance->ir_left;
//        ir_right_oa[cntr_oa_debug]   = _robot->ir_distance->ir_right;
//        cntr_oa_debug++;
//    }


  // pi/4= 0.785
  vector_2d left_sensor_rf = convert2rf(50,70,  0.785, _robot->ir_distance->ir_left);
  vector_2d right_sensor_rf = convert2rf(50,-70,-0.785, _robot->ir_distance->ir_right);

  vector_2d left_sensor_wf =  convert2wf(left_sensor_rf, _robot->pose->x, _robot->pose->y, _robot->pose->theta);
  vector_2d rihgt_sensor_wf =  convert2wf(right_sensor_rf, _robot->pose->x, _robot->pose->y, _robot->pose->theta);

  float x_dir = left_sensor_wf.x + rihgt_sensor_wf.x;
  float y_dir = left_sensor_wf.y + rihgt_sensor_wf.y;

  float x_g = x_dir - _robot->pose->x;
  float y_g = y_dir - _robot->pose->y; 

//  P1->OUT &=~BIT5;
//  P1->OUT |=BIT5;
//  printf( "%d %d\n", (uint16_t)x_g, (uint16_t)y_g );
//  P1->OUT |=BIT5;
//  P1->OUT &=~BIT5;

  float theta_g =   atan2f(y_g , x_g);

//  P1->OUT &=~BIT5;
//  P1->OUT |=BIT5;
//  printf( "%d %d\n", (uint16_t)(theta_g * 100), (uint16_t)( _robot->pose->theta * 100) );
//  P1->OUT |=BIT5;
//  P1->OUT &=~BIT5;


  float heading_error = theta_g - _robot->pose->theta;
  float err = atan2f(sin(heading_error), cos(heading_error));

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

  duty_check();

  motor_forward(right_duty_cycle, left_duty_cycle);
  robot_position_update(_robot);
}






















