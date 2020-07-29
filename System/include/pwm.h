/**
 * Date	 : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>

#ifndef SYSTEM_INCLUDE_PWM_H_
#define SYSTEM_INCLUDE_PWM_H_

/** ------------pwm_init------------
 *@brief:  power the DC motors
 *@params: period specifies the PWM signal frequency.
 *         dutyCycle sets the PWM signal duty cycle.
 *@return: none
*/
void pwm_init(uint16_t period, uint16_t dutyCycle);

/** ------------set_left_duty_cycle------------
 *@brief:  set the PWM duty cycle for the left motor
 *@params: dutyCycle
 *@return: none
 */
void set_left_duty_cycle(uint16_t leftDutyCycle);

/** ------------set_right_duty_cycle------------
 *@brief:  set the PWM duty cycle for the right motor
 *@params: dutyCycle
 *@return: none
 */
void set_right_duty_cycle(uint16_t rightDutyCycle);

#endif /* SYSTEM_INCLUDE_PWM_H_ */
