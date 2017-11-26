/*
 * pwm.h
 *
 *  Created on: Oct 21, 2017
 *      Author: snow
 */

#ifndef _PWM_H_
#define _PWM_H_

#include "sys.h"
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>

void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);

#endif /* SRC_MCUDRIVER_PWM_H_ */
