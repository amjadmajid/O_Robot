/*
 * Date: 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email: amjad.y.majid@gmail.com
 */

#include "differentialRobot.h"
#include <stdio.h>
#include <math.h>

float _robot_distance_update_mm(float d_r, float d_l){
	return (d_r + d_l)/2;
}


void _wheel_distance_update_mm( tachometer_t * tachometer){
	// calculate the distance travelled since the last update in millimeter 
    int32_t ticks = tachometer->ticks;
	float delta_ticks = ticks - tachometer->prev_ticks;
	// update the previous tachometer ticks
	tachometer->prev_ticks = ticks;

//	uint16_t static printf_flag=10;
//    printf_flag--;
//    if(printf_flag==0){
//	printf("%d\n",delta_ticks);
//    printf_flag=10;
//    }

	tachometer->delta_dis = (TICK_DIS_NUMERATOR * delta_ticks) / TICK_DIS_DENOMINATOR;  //distance in meter

	  // uint16_t static printf_flag=10;
	  //   printf_flag--;
	  //   if(printf_flag==0){
	  //       printf("%.d\n",tachometer->delta_dis);
	  //       printf_flag=10;
	  //   }
}

void robot_position_update(differential_robot_t * robot){
    _wheel_distance_update_mm( robot->right->tachometer );
    _wheel_distance_update_mm( robot->left->tachometer );
    float d_r = robot->right->tachometer->delta_dis;
    float d_l = robot->left->tachometer->delta_dis;
	float d_c = _robot_distance_update_mm(d_r, d_l);

    uint16_t static printf_flag=0;
    if(printf_flag==4){
    printf("l=%d r=%d \n",robot->left->tachometer->ticks, robot->right->tachometer->ticks);
    printf_flag=0;
    }
    printf_flag++;

	float x = robot->pose->x;
	float y = robot->pose->y;
	float theta = robot->pose->theta;

	x += d_c * cos(theta);
	y += d_c * sin(theta);
	float delta_theta = (float)(d_r - d_l)/robot->base_len;
	theta += delta_theta;

	robot->pose->x = x;
	robot->pose->y = y;
	robot->pose->theta = atan2( sin(theta), cos(theta));

//	uint16_t static printf_flag=0;
//	if(printf_flag==8){
//	printf("x=%.4f y=%.4f theta=%.4f\n",x,y, theta);
//	printf_flag=0;
//	}
//	printf_flag++;
}










