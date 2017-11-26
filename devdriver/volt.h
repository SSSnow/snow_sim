/*
 * volt.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _VOLT_H_
#define _VOLT_H_

#include "device.h"
#include "driver.h"
#include "../mcudriver/adc.h"

unsigned int volt_getID(void);
unsigned int volt_register(void);
static int volt_init(void);
static int volt_read(void* buffer,unsigned int len);

#endif /* SRC_DEVDRIVER_VOLT_H_ */
