/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#ifndef SYSTEM_INCLUDE_TIMER_A1_H_
#define SYSTEM_INCLUDE_TIMER_A1_H_

void timerA1_init(void(*task)(void), uint32_t period);
void timerA1_stop(void);

#endif /* SYSTEM_INCLUDE_TIMER_A1_H_ */
