/*
 * Date  : 26 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <differentialRobot.h>
#include <stdint.h>

#ifndef BEHAVIORS_INCLUDE_GO_TO__GOAL_H_
#define BEHAVIORS_INCLUDE_GO_TO__GOAL_H_

void go_to_goal_init(float x_g, float y_g, differential_robot_t * robot_pt, uint16_t p);

#endif /* BEHAVIORS_INCLUDE_GO_TO__GOAL_H_ */
