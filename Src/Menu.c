/*
 * Menu.c
 *
 *  Created on: Aug 18, 2019
 *      Author: bnezu
 */

#include "Menu.h"

char* GetTimeType(int value)
{
	if(value == centi)
	{
		return "centi ";
	}else if(value == seconds)
	{
		return "sec   ";
	}else if(value == minutes)
	{
		return "min   ";
	}else if(value == hours)
	{
		return "hr    ";
	}
	return "NONE  ";
}
