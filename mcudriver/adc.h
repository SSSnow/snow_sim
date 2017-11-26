/*
 * adc.h
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#ifndef _ADC_H_
#define _ADC_H_

#include "sys.h"
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_adc.h>
#include <stm32f10x_dma.h>

void adc_Init(void);
u16 adc_Get(void);

#endif /* SRC_MCUDRIVER_ADC_H_ */
