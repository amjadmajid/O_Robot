/*
 * vectorTranslation.c
 *
 *  Created on: 28 Sep 2020
 *      Author: Amjad Yousef Majid
 */

#include <stdint.h>
#include <math.h>

#include "vectorTranslation.h"


// convert the sensor measurement to the robot frame of reference
vector_2d convert2rf(float x_s, float y_s, float theta, uint32_t ir_distance)
{
  vector_2d sensor_vec;
//  float theta_tmp = theta - 0.785;
//
//  if (theta == 0){  // check if theta is 0
//      sensor_vec.x = x_s;
//      sensor_vec.y = ir_distance + y_s
//
//  } else if ( theta_tmp < 0.01 && theta_tmp > 0 ) // check if theta is +pi/4
//  {
//    // cos(+-pi/4) = sin(pi/4) = 0.707;
//    float tmp = ((707 * ir_distance)/1000);
//    sensor_vec.x = tmp + x_s;
//    sensor_vec.y = tmp + y_s;
//  }
//  else if ( theta_tmp < 0.01  ) // check if theta is -pi/4
//  {
//     // sin(- pi/4) = -0.707;
//    float tmp = ((707 * ir_distance)/1000);
//    sensor_vec.x = tmp + x_s;
//    sensor_vec.y = y_s - tmp ;
//  }else{
    sensor_vec.x = cos(theta) * ir_distance + x_s;
    sensor_vec.y = sin(theta) * ir_distance + y_s;
//  }

  return sensor_vec;
}

// convert to the world frame of reference
vector_2d convert2wf(vector_2d robot_sensor, float x_r, float y_r, float theta){

  vector_2d robot_vec;

  robot_vec.x = cos(theta) * robot_sensor.x - sin(theta) * robot_sensor.y + x_r;
  robot_vec.y = sin(theta) * robot_sensor.x + cos(theta) * robot_sensor.y + y_r;

  return robot_vec;
}
