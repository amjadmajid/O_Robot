#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "interruptHandler.h"
#include "tachometer.h"
#include "differentialRobot.h"
#include "msp.h"
#include "motor.h"
#include "timer_A1.h"
#include "ir_distance.h"
#include "us_distance.h"
#include "lpf.h"
#include "UART1.h"
#include "pwm.h"
#include "goToGoal.h"
#include "avoidObstacles.h"
#include "blendedController.h"

// the linear velocity amplified by 100 for integer math purpose 
#define LINEAR_VELOCITY 35000
#define DUTY_CYCLE 3750

float _x_goal;
float _y_goal;
uint8_t *_goal_flag;

float theta_goal;
uint32_t linear_velocity = LINEAR_VELOCITY;
int32_t left_duty_cycle;
int32_t right_duty_cycle;
differential_robot_t *_robot;

//select whether infrared or ultrasound sensors are used for the robot in differentialRobot.h

void controller();

void leftTachometer(void)
{
    int32_t tks = _robot->left->tachometer->ticks;
    if (left_duty_cycle >= 0)
    {
        tks++;
    }
    else
    {
        tks--;
    }
    _robot->left->tachometer->ticks = tks;
}

void rightTachometer(void)
{
    int32_t tks = _robot->right->tachometer->ticks;
    if (right_duty_cycle >= 0)
    {
        tks++;
    }
    else
    {
        tks--;
    }
    _robot->right->tachometer->ticks = tks;
}

void duty_check(int32_t *left_duty_cycle, int32_t *right_duty_cycle)
{
    // control the max and the min of the duty cycle
    if (*right_duty_cycle > 11000)
        *right_duty_cycle = 11000;
    if (*right_duty_cycle < -11000)
        *right_duty_cycle = -11000;
    if (*left_duty_cycle > 11000)
        *left_duty_cycle = 11000;
    if (*left_duty_cycle < -11000)
        *left_duty_cycle = -11000;
}

void controller_init(float x_g, float y_g, differential_robot_t *robot_pt, uint32_t p, uint8_t *goal_flag)
{
    _x_goal = x_g;
    _y_goal = y_g;
    _goal_flag = goal_flag;

    _robot = robot_pt;

    motor_init();
    enableInterrupts();
    tachometer_init(&leftTachometer, &rightTachometer);
    pwm_init(15000, 0);
    timerA1_init(&controller, p);

#if ULTRASOUND == 0
    adc_init_channel_14_16_17();
    //initialize the ADC for the ir distance sensor
    uint32_t *init_left = NULL;
    uint32_t *init_center = NULL;
    uint32_t *init_right = NULL;
    ir_distances(init_left, init_center, init_right);
    //initialize the Low Pass Filters for the ir distance sensors
    LPF_Init(*init_left, 32);       // P9.0/channel 17
    LPF_Init2(*init_center, 32);    // P6.1/channel 14
    LPF_Init3(*init_right, 32);     // P9.1/channel 16
#elif ULTRASOUND == 1
    ultrasound_init();
#endif
}

uint16_t controller_switch = 500;
uint8_t control_code = '0';

void controller()
{
    P1->OUT |= BIT5;
    P1->OUT &= ~BIT5;
    uint16_t res = RxFifo_Get(&control_code);
    //    if (res == 1)
    //    {
    //        UART1_OutChar(control_code);
    //    }

    if (control_code == '1')
    {
        // updating the sensor distance measurements
        // if sensor distance is greater than 50 cm
        // run avoid obstacles controller
        // else
        // run go to goal controller
#if ULTRASOUND == 0
        ir_distances(&(_robot->sensor_distances->sensor_left), &(_robot->sensor_distances->sensor_center), &(_robot->sensor_distances->sensor_right));
#elif ULTRASOUND == 1
        us_distances(&(_robot->sensor_distances->sensor_left), &(_robot->sensor_distances->sensor_center), &(_robot->sensor_distances->sensor_right));
#endif

//        if (_robot->sensor_distances->sensor_left > controller_switch && _robot->sensor_distances->sensor_center > controller_switch && _robot->sensor_distances->sensor_right > controller_switch)
//        {
//            controller_switch = 500;
//            go_to_goal_controller();
//        }
//        //     else
//        //      ( _robot->sensor_distance->sensor_left > 300 &&
//        //            _robot->sensor_distance->sensor_center > 300 &&
//        //            _robot->sensor_distance->sensor_right > 300)
//        //  {
//        //    controller_switch = 500;
//        //    blended_controller();
//        //  }
//        else
//        {
//            controller_switch = 600;
            avoid_obstacle_controller();
//        }
    }
    else
    {
        motor_forward(0, 0);
    }

    UART1_OutUDec((uint32_t) _robot->sensor_distances->sensor_left);
    UART1_OutChar(' ');
    UART1_OutUDec((uint32_t) _robot->sensor_distances->sensor_center);
    UART1_OutChar(' ');
    UART1_OutUDec((uint32_t) _robot->sensor_distances->sensor_right);
    UART1_OutChar('\r');
    UART1_OutChar('\n');

    //    UART1_OutUDec((uint32_t) _robot->pose->x);
    //    UART1_OutChar(' ');
    //    UART1_OutUDec((uint32_t) _robot->pose->y);
    //    UART1_OutChar(' ');
    //    UART1_OutUDec((uint32_t) _robot->pose->theta);
    //    UART1_OutChar('\r');
    //    UART1_OutChar('\n');
}
