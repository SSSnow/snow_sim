/*
 * timer.h
 *
 *  Created on: Oct 21, 2017
 *      Author: snow
 */

#ifndef _TIMER_H_
#define _TIMER_H_

#include "sys.h"
#include "../modules/attitude_control.h"
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_tim.h>
void timer2_int_init(void);
void TIM2_IRQHandler(void);

#endif /* SRC_MCUDRIVER_TIMER_H_ */
