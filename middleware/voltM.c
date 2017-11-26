/*
 * voltM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "voltM.h"

float get_raw_volt(void){
	short voltValueRaw = 0;
	float temp = 0;
	read(volt_getID(), &voltValueRaw, 2);
	temp = (float)(voltValueRaw) * ADC_V_COEFF * BATTERY_V_DIV + ADC_V_OFFSET;
	return temp;
}

float get_lpf_volt(void){
	
	static float voltage = 0.f;
	voltage = voltage * 0.95f + get_raw_volt() * 0.05f;
	return voltage;
}

bool low_battery_status(void){
		if(get_lpf_volt() < 3.2f)
			return true;
		else
			return false;
}
