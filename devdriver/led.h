/*
 * led.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _LED_H_
#define _LED_H_

#include "device.h"
#include "driver.h"
#include "mcudriver/delay.h"
#include <stm32f10x_rcc.h>
#include <stm32f10x.h>
#include <stm32f10x_gpio.h>

#define R_MASK 0x01
#define G_MASK 0x02
#define B_MASK 0x04
unsigned int led_getID(void);
unsigned int led_register(void);
static int led_init(void);
static int led_ioctrl(unsigned char cmd, void* arg);
static void led_gpio_config(void);

typedef enum
{
	BLACK = 0,
	RED   = 1,
	GREEN = 2,
	BLUE  = 4,
	WHITE = RED|GREEN|BLUE
}LED_COLOR;

typedef enum{
	LED_BOARD,
	LED_INDEP
}LED_CMD;

#endif /* SRC_DEVDRIVER_LED_H_ */
