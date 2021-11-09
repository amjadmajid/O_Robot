/*
 * Date  : 25 Jul 2020
 * Author: Amjad Yousef Majid
 * Email : amjad.y.majid@gmail.com
 */

#include <stdint.h>
#include <interruptHandler.h>
#include "msp.h"
// Tachometer.c
// Runs on MSP432
// Provide mid-level functions that initialize ports, take
// angle and distance measurements, and report total travel
// statistics.
// Daniel Valvano
// December 20, 2018

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Left Encoder A connected to P10.5 (J5)
// Left Encoder B connected to P5.2 (J2.12)
// Right Encoder A connected to P10.4 (J5)
// Right Encoder B connected to P5.0 (J2.13)

#include <stdint.h>
#include "clock.h"
#include "TA3InputCapture.h"
#include "msp.h"
#include "tachometer.h"

void ta3dummy(uint16_t t){};       // dummy function
void(*taskLeft)(uint16_t time) = ta3dummy;
void(*taskRight)(uint16_t time) = ta3dummy;

// ------------Tachometer_Init------------
// Initialize GPIO pins for input, which will be
// used to determine the direction of rotation.
// Initialize the input capture interface, which
// will be used to measure the speed of rotation.
// Input: none
// Output: none
void tachometer_init(void(*userTaskLeft)(uint16_t time), void(*userTaskRight)(uint16_t time) ){
  // initialize P5.0 and P5.2 and make them GPIO inputs
  P5->SEL0 &= ~0x05;
  P5->SEL1 &= ~0x05;               // configure P5.0 and P5.2 as GPIO
  P5->DIR &= ~0x05;                // make P5.0 and P5.2 in

  // write this for Lab 16
      taskLeft = userTaskLeft;              // user function0
      taskRight = userTaskRight;              // user function1
      // initialize P10.4 and make it input (P10.4 TA3CCP0)
      P10->SEL0 |= 0x10;
      P10->SEL1 &= ~0x10;                 // configure P10.4 as TA3CCP0
      P10->DIR &= ~0x10;                  // make P10.4 in
      // initialize P10.5 and make it input (P10.5 TA3CCP1)
      P10->SEL0 |= 0x20;
      P10->SEL1 &= ~0x20;                 // configure P10.5 as TA3CCP1
      P10->DIR &= ~0x20;                  // make P10.5 in
      TIMER_A3->CTL &= ~0x0030;          // halt Timer A3
      // bits15-10=XXXXXX, reserved
      // bits9-8=10,       clock source to SMCLK
      // bits7-6=00,       input clock divider /1
      // bits5-4=00,       stop mode
      // bit3=X,           reserved
      // bit2=0,           set this bit to clear
      // bit1=0,           interrupt disable
      // bit0=0,           clear interrupt pending
      TIMER_A3->CTL = 0x0200;
      // bits15-14=01,     capture on rising edge
      // bits13-12=00,     capture/compare input on CCI3A
      // bit11=1,          synchronous capture source
      // bit10=X,          synchronized capture/compare input
      // bit9=X,           reserved
      // bit8=1,           capture mode
      // bits7-5=XXX,      output mode
      // bit4=1,           enable capture/compare interrupt
      // bit3=X,           read capture/compare input from here
      // bit2=X,           output this value in output mode 0
      // bit1=X,           capture overflow status
      // bit0=0,           clear capture/compare interrupt pending
      TIMER_A3->CCTL[0] = 0x4910;
      TIMER_A3->CCTL[1] = 0x4910;
      TIMER_A3->EX0 &= ~0x0007;       // configure for input clock divider /1
      NVIC->IP[0] = (NVIC->IP[0]&0xFFFFFF00)|0x00000040; // priority 1
    // interrupts enabled in the main program after all devices initialized
      NVIC->ISER[0] = 0x0000C000;      // enable interrupt 14 and 15 in NVIC
        // bits15-10=XXXXXX, reserved
        // bits9-8=10,       clock source to SMCLK
        // bits7-6=00,       input clock divider /1
        // bits5-4=10,       continuous count up mode
        // bit3=X,           reserved
        // bit2=1,           set this bit to clear
        // bit1=0,           interrupt disable (no interrupt on rollover)
        // bit0=0,           clear interrupt pending
      TIMER_A3->CTL |= 0x0024;        // reset and start Timer A3 in continuous up mode

}

void TA3_0_IRQHandler(void){
    TIMER_A3->CCTL[0] &= ~0x0001;             // acknowledge capture/compare interrupt 0
    (*taskLeft)(TIMER_A3->CCR[0]);         // execute user task
}

void TA3_N_IRQHandler(void){
    TIMER_A3->CCTL[1] &= ~0x0001;             // acknowledge capture/compare interrupt 1
    (*taskRight)(TIMER_A3->CCR[1]);         // execute user task
}

// ------------Tachometer_Get------------
// Get the most recent tachometer measurements.
// Input: leftTach   is pointer to store last measured tachometer period of left wheel (units of 0.083 usec)
//        leftDir    is pointer to store enumerated direction of last movement of left wheel
//        leftSteps  is pointer to store total number of forward steps measured for left wheel (360 steps per ~220 mm circumference)
//        rightTach  is pointer to store last measured tachometer period of right wheel (units of 0.083 usec)
//        rightDir   is pointer to store enumerated direction of last movement of right wheel
//        rightSteps is pointer to store total number of forward steps measured for right wheel (360 steps per ~220 mm circumference)
// Output: none
// Assumes: Tachometer_Init() has been called
// Assumes: Clock_Init48MHz() has been called
//void Tachometer_Get(uint16_t *leftTach, enum TachDirection *leftDir, int32_t *leftSteps,
//                    uint16_t *rightTach, enum TachDirection *rightDir, int32_t *rightSteps){
//  *leftTach = (Tachometer_SecondLeftTime - Tachometer_FirstLeftTime);
//  *leftDir = Tachometer_LeftDir;
//  *leftSteps = Tachometer_LeftSteps;
//  *rightTach = (Tachometer_SecondRightTime - Tachometer_FirstRightTime);
//  *rightDir = Tachometer_RightDir;
//  *rightSteps = Tachometer_RightSteps;
//}
