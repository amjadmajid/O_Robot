#include <stdint.h>
#include <stdio.h>

#include "differentialRobot.h"
#include "msp.h"
#include "timer_A1.h"
#include "ir_distance.h"
#include "lpf.h"
#include "UART0.h"

void avoid_obstacle_controller();

differential_robot_t * _robot;

void avoid_obstacle_init(differential_robot_t * robot_pt, uint32_t p)
{

    _robot = robot_pt;

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

}

void avoid_obstacle_controller()
{
	//    updating the IR distance measurements
  ir_distances(&(_robot->ir_distance->ir_left),&(_robot->ir_distance->ir_center),&(_robot->ir_distance->ir_right) );

   UART0_OutUDec(_robot->ir_distance->ir_left); UART0_OutChar(' ');
   UART0_OutUDec(_robot->ir_distance->ir_center); UART0_OutChar(' ');
   UART0_OutUDec(_robot->ir_distance->ir_right);
   UART0_OutChar('\n'); UART0_OutChar('\r');
}
