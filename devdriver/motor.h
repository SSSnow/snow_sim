/*
 * motor.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "../mcudriver/pwm.h"
#include "bsp.h"
unsigned int moto_getID(void);
unsigned int moto_register(void);
static int moto_init(void);

#endif /* SRC_DEVDRIVER_MOTOR_H_ */
