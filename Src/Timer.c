/*
 * Timer.c
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */

#include "Timer.h"


unsigned int GetDesiredPeriodandPrescaler(float DesiredDelay, int clockspeed)//seconds(desired second for period)
{
	//clock speed(timer clock? or system clock? or ABP1/2) not sure how to get will change
	unsigned int periodPrescaler = 0;
	periodPrescaler = sqrt((double)clockspeed * (double)DesiredDelay)+1;

	if(periodPrescaler >= INT_16BIT_MAX)
		periodPrescaler = INT_16BIT_MAX - 1;

	return periodPrescaler;
}

char UpdateTimer(int timerNumber,int pinState)
{
	char togglePin = 0;
	timerCounter[timerNumber]++;
	  unsigned int triggerTime = 0;
	  if(pinState == 1)
	  {
		  triggerTime = ((float)timer[0][timerNumber][3] * 60 * 60 * 100)
							  +((float)timer[0][timerNumber][2] * 60 * 100)
							  +((float)timer[0][timerNumber][1] * 100)
							  +((float)timer[0][timerNumber][0]);
	  }else
	  {
		  triggerTime = ((float)timer[1][timerNumber][3] * 60 * 60 * 100)
							  +((float)timer[1][timerNumber][2] * 60 * 100)
							  +((float)timer[1][timerNumber][1] * 100)
							  +((float)timer[1][timerNumber][0]);
	  }
	  if(timerCounter[timerNumber] >= triggerTime)
	  {
		  timerCounter[timerNumber] = 0;
		  togglePin = 1;
	}
	  return togglePin;
}
