/*
 * Timer.c
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */

#include "Timer.h"


unsigned int GetDesiredPeriodandPrescaler(float DesiredDelay)//seconds(desired second for period)
{
	unsigned int clockspeed = 62500;//clock speed(timer clock? or system clock? or ABP1/2) not sure how to get will change
	unsigned int periodPrescaler = 0;
	periodPrescaler = sqrt((double)clockspeed * (double)DesiredDelay)+1;

	if(periodPrescaler >= INT_16BIT_MAX)
		periodPrescaler = INT_16BIT_MAX - 1;

	return periodPrescaler;
}
