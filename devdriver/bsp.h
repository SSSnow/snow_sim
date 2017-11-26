/*
 * bsp.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _BSP_H_
#define _BSP_H_

#include <devdriver/cc2530.h>   //ning jing zhi yuan
#include "icm20602.h"
#include "motor.h"
#include "ms56xx.h"
#include "volt.h"
#include "led.h"
#include "device.h"
#include "driver.h"
#include "monitor.h"

/* device error flag */
#define DEV_ERR_ICM20602 0x01
#define DEV_ERR_MS56xx 0x02

#define DEV_ERR_MASK    0x03

uint8_t bsp_init(void);

#endif /* SRC_DEVDRIVER_BSP_H_ */
