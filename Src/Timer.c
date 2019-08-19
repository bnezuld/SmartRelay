/*
 * Timer.c
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */

#include "Timer.h"

int GetDesiredPeriod(float DesiredDelay,int currentPrescaler)//seconds(desired second for period)
{
	int clockspeed = 62500;//clock speed not sure how to get will change
	int period = 0;
	period = ((DesiredDelay * clockspeed) / (currentPrescaler + 1));

	if(period >= INT_16BIT_MAX)
		period = INT_16BIT_MAX - 1;
	if(period <= 0)
		period = 1;

	return period;
}
