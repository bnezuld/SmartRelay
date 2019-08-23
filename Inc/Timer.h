/*
 * Timer.h
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */
#include <math.h> //sqrt

#define INT_16BIT_MAX 65536

int debouncer;

unsigned int GetDesiredPeriodandPrescaler(float DesiredDelay);

