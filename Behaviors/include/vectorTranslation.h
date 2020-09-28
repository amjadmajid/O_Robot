/*
 * vectorTranslation.h
 *
 *  Created on: 28 Sep 2020
 *      Author: Amjad Yousef Majid
 */

#include <stdint.h>


#ifndef BEHAVIORS_INCLUDE_VECTORTRANSLATION_H_
#define BEHAVIORS_INCLUDE_VECTORTRANSLATION_H_

typedef struct 
{
  float x;
  float y;
} vector_2d;

vector_2d convert2rf(int32_t x_s, int32_t y_s, float theta, uint32_t ir_distance);
vector_2d convert2wf(vector_2d robot_sensor, int32_t x_r, int32_t y_r, float theta);



#endif /* BEHAVIORS_INCLUDE_VECTORTRANSLATION_H_ */
