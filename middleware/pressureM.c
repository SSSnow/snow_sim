/*
 * pressureM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "pressureM.h"

#define BARO_FILTER_LEN 24

static float baroPressureSum = 0;
int temp_baro = 0;

unsigned char BaroErr = 0;
unsigned char BaroErrFlag = 0;

int B_buffer[2] = {0};

void pressure_update(void){
	static unsigned char state = 0;

	static int baro_HistTab[BARO_FILTER_LEN] = {0};
	int baro_SortTab[BARO_FILTER_LEN] = {0};

	static unsigned char HistTab_index = 0;

	if(BaroErrFlag==1)
	{
		if(ioctrl(ms56xx_getID(), MS56XX_IOCTRL_RESET, NULL))
		{
			state=0;
			BaroErrFlag=0;
			BaroErr ++;
		 }
		 else
		 {
			 BaroErr ++;
		 }
		 return;
	}

	 if(state)
	 {
		if(!ioctrl(ms56xx_getID(), MS56XX_IOCTRL_PRESSURE_READ, NULL)) { BaroErrFlag=1;return;}
		if(!ioctrl(ms56xx_getID(), MS56XX_IOCTRL_START_TEMPERATURE, NULL))  { BaroErrFlag=1;return;}
		if(!ioctrl(ms56xx_getID(), MS56XX_IOCTRL_PRESSURE_CALCULATE, B_buffer))  { BaroErrFlag=1;return;}

		temp_baro = B_buffer[1];
		baro_HistTab[HistTab_index] = B_buffer[0];
		HistTab_index ++;
		if(HistTab_index >= BARO_FILTER_LEN)
		{
			HistTab_index = 0;
		}

		memcpy(baro_SortTab, baro_HistTab, BARO_FILTER_LEN * 4);

		sort(baro_SortTab, BARO_FILTER_LEN, 0);

		int sum = 0;
		for(unsigned char k = BARO_FILTER_LEN / 4; k < (BARO_FILTER_LEN / 4) * 3; k ++)
		{
			sum += baro_SortTab[k];
		}
		baroPressureSum = (float)sum / (BARO_FILTER_LEN / 2);

		state = 0;//toggle.
	 }
	 else
	 {
		if(!ioctrl(ms56xx_getID(),MS56XX_IOCTRL_TEMPERATURE_READ, NULL))  { BaroErrFlag=1;return;}
		if(!ioctrl(ms56xx_getID(),MS56XX_IOCTRL_START_PRESSURE, NULL))  { BaroErrFlag=1;return;}
		state = 1;
	 }
}

float get_pressure(void)
{
	return baroPressureSum;
}
