/*
 * motoM.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _MOTOM_H_
#define _MOTOM_H_

#include "middleware.h"
#include "stdbool.h"
#include "algorithm.h"
void moto_pwm_output(float motor0,float motor1,float motor2,float motor3, bool reveser_enable);

#endif /* SRC_MIDDLEWARE_MOTOM_H_ */
