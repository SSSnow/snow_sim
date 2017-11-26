/*
 * voltM.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _VOLTM_H_
#define _VOLTM_H_

#include "middleware.h"
#include "stdbool.h"

#define BATTERY_V_DIV (1.5466667f) //modify according the bleeder circuit
#define ADC_V_COEFF (0.00073242188f)  //3.0v/4096
#define ADC_V_OFFSET 0     //set adc offset method of calculation (actual_v - adc_measure_v)

float get_raw_volt(void);
float get_lpf_volt(void);
bool low_battery_status(void);

#endif /* SRC_MIDDLEWARE_VOLTM_H_ */
