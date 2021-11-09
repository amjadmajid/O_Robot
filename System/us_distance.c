/*
 * us_distance.c
 *
 *  Created on: 29 Oct 2021
 *      Author: Kilian van Berlo
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>

#include "msp.h"
#include "clock.h"

#define MAXDISTANCE 800

uint32_t SetDistanceMid(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found within the time limit

    uint32_t time;
    uint8_t echoValue;

    //Trigger off
    P6->OUT &= ~0x10;
    Clock_Delay1us(2);

    //Trigger on
    P6->OUT |= 0x10;
    Clock_Delay1us(10);

    //Trigger off
    P6->OUT &= ~0x10;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the echo sensor is connected to
    echoValue = (P6->IN & 0x20);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & 0x20);
    }

    time = 0;
    echoValue = (P6->IN & 0x20);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P6->IN & 0x20);
    }

    //Rate of sound in air is approximately 0.343 milimeters per microsecond
    //Distance = Rate * time
    return (0.343 * time);
}

uint32_t SetDistanceLeft(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found

    //Trigger Left connect to P6.0
    //Echo Left connect to P2.5

    uint32_t time;
    uint8_t echoValue;

    //Trigger off
    P6->OUT &= ~0x01;
    Clock_Delay1us(2);

    //Trigger on
    P6->OUT |= 0x01;
    Clock_Delay1us(10);

    //Trigger off
    P6->OUT &= ~0x01;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the echo sensor is connected to
    echoValue = (P2->IN & 0x20);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P2->IN & 0x20);
    }

    time = 0;
    echoValue = (P2->IN & 0x20);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P2->IN & 0x20);
    }

    //Rate of sound in air is approximately 0.343 milimeters per microsecond
    //Distance = Rate * time
    return (0.343 * time);
}

uint32_t SetDistanceRight(int timeOut){
    //timeOut is a value in microseconds
    //it is used to exit the while loop if the signal is not found

    //Trigger Right P4.1
    //Echo Right P3.0

    uint32_t time;
    uint8_t echoValue;

    //Trigger off
    P4->OUT &= ~0x02;
    Clock_Delay1us(2);

    //Trigger on
    P4->OUT |= 0x02;
    Clock_Delay1us(10);

    //Trigger off
    P4->OUT &= ~0x02;

    time = 0;
    //Store the value of the echo sensor ANDing the pin vector with the bit that the eacho sensor is connected to
    echoValue = (P3->IN & 0x01);

    //wait for triggers signal to be received by the echo sensor
    while( echoValue == 0x00 && time < timeOut){
        //Delay for one microsecond so we can keep track of how much time it took to receive signal
        Clock_Delay1us(1);
        time++;
        echoValue = (P3->IN & 0x01);
    }

    time = 0;
    echoValue = (P3->IN & 0x01);

    //wait for echo signal to return to LOW
    while( echoValue != 0x00 && time < timeOut  ){
        //Delay for 1 microsecond and increment time variable so the time variable will store how much time it took for the echo signal to return to 0
        Clock_Delay1us(1);
        time++;
        echoValue = (P3->IN & 0x01);
    }

    //Rate of sound in air is approximately 0.343 milimeters per microsecond
    //Distance = Rate * time
    return (0.343 * time);
}


void ultrasound_init(void)
{
    //TriggerC as GPIO, P6.4
    P6->SEL0 &= ~0x10;
    P6->SEL1 &= ~0x10;
    //TriggerC as Output
    P6->DIR |= 0x10;

    //EchoC as GPIO P6.5
    P6->SEL0 &= ~0x20;
    P6->SEL1 &= ~0x20;
    //EchoC as input
    P6->DIR &= ~0x20;

    //TriggerL as GPIO P6.0
    P6->SEL0 &= ~0x01;
    P6->SEL1 &= ~0x01;
    //Trigger as Output
    P6->DIR |= 0x01;

    //EchoBitL as GPIO P2.5
    P2->SEL0 &= ~0x20;
    P2->SEL1 &= ~0x20;
    //Echo as input
    P2->DIR &= ~0x20;

    //TriggerR as GPIO P4.1
    P4->SEL0 &= ~0x02;
    P4->SEL1 &= ~0x02;
    //Trigger as Output
    P4->DIR |= 0x02;

    //EchoBitR as GPIO P3.0
    P3->SEL0 &= ~0x01;
    P3->SEL1 &= ~0x01;
    //Echo as input
    P3->DIR &= ~0x01;
}

void us_distances(uint32_t *left, uint32_t *center, uint32_t *right)
{

    *center = SetDistanceMid(2333);
    *left = SetDistanceLeft(2333);
    *right = SetDistanceRight(2333);

}
