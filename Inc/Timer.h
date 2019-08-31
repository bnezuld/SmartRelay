/*
 * Timer.h
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */
#include <math.h> //sqrt


#define INT_16BIT_MAX 65536

int debouncer;
unsigned char timerCount;
unsigned char timer[2][4][4];
unsigned int timerCounter[4];
unsigned int timerPin[4];
unsigned char timerPinGroup[4];

unsigned int GetDesiredPeriodandPrescaler(float DesiredDelay, int clockspeed);

char UpdateTimer(int timerNumber, int pinState);
