/*
 * delay.h
 *
 *  Created on: Oct 13, 2017
 *      Author: snow
 */

#ifndef _DELAY_H_
#define _DELAY_H_

#include "sys.h"

void delay_init(void);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif /* SRC_MCUDRIVER_DELAY_H_ */
