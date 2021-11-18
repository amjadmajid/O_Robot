/**
 *@brief:  low level motion control
 *@Date:   14 May 2020
 *@Author: Amjad Yousef Majid
 */

#include <stdint.h>
#include <stdio.h>
#include "msp.h"
#include "pwm.h"
#include "motor.h"

/**
 * Make the DC motors' direction, power, sleep pins output.
 * Make sure the DC motors are not powered 
 */
void motor_init(void)
{
    P5->DIR |= BIT5 | BIT4;   // Control direction
    P2->DIR |= BIT6 | BIT7;   // Enable power
    P3->DIR |= BIT6 | BIT7;   // Control the sleep mode

    POWER &= ~(BIT6 | BIT7 );
}

/**
 * Cut output power on the DC motors. Kill the direction signal and
 * Put the motor in sleep mode to safe energy. Set the duty cycles to zero
 */
void motor_stop(void)
{
    DIRECTION &= ~(BIT5 | BIT4);   // kill the direction signal
    SLEEP &= ~(BIT6 | BIT7 );       // sleep mode
    POWER &= ~(BIT6 | BIT7 );       // kill the power signal 
    set_left_duty_cycle(0);
    set_right_duty_cycle(0);
}

void motor_right(uint32_t leftDutyCycle, uint32_t rightDutyCycle)
{
    set_left_duty_cycle(leftDutyCycle);
    set_right_duty_cycle(rightDutyCycle);
    DIRECTION &= ~BIT4;
    DIRECTION |= BIT5;
    SLEEP |= (BIT6 | BIT7 );
    POWER |= (BIT6 | BIT7 );
}

void motor_left(uint32_t leftDutyCycle, uint32_t rightDutyCycle)
{
    set_left_duty_cycle(leftDutyCycle);
    set_right_duty_cycle(rightDutyCycle);
    DIRECTION |= BIT4;
    DIRECTION &= ~BIT5;
    SLEEP |= (BIT6 | BIT7 );
    POWER |= (BIT6 | BIT7 );
}

void motor_backward(uint32_t leftDutyCycle, uint32_t rightDutyCycle)
{
    set_left_duty_cycle(leftDutyCycle);
    set_right_duty_cycle(rightDutyCycle);
    DIRECTION |= (BIT5 | BIT4);
    SLEEP |= (BIT6 | BIT7 );
    POWER |= (BIT6 | BIT7 );
}

void motor_forward(int32_t leftDuty, int32_t rightDuty)
{

    if (leftDuty < 0 && rightDuty < 0)
    {
        motor_backward((uint32_t) (leftDuty * -1), (uint32_t) (rightDuty * -1));
    }
    else if (leftDuty < 0)
    {
        motor_left((uint32_t) (leftDuty * -1), (uint32_t) rightDuty);
    }
    else if (rightDuty < 0)
    {
        motor_right((uint32_t) leftDuty, (uint32_t) (rightDuty * -1));
    }
    else
    {
        // Go forward
        set_left_duty_cycle(leftDuty);
        set_right_duty_cycle(rightDuty);

        DIRECTION &= ~(BIT5 | BIT4);

        SLEEP |= (BIT6 | BIT7 );
        POWER |= (BIT6 | BIT7 );
    }
}

