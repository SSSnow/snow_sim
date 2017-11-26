/*
 * led.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "led.h"

static unsigned int LedID = 0;

static DEV Led = {
	.name = "LED",
	.devDrv = {
	.init = led_init,
	.ioctrl = led_ioctrl
	}
};

unsigned int led_getID(void)
{
	return LedID;
}

unsigned int led_register(void)
{
	LedID = register_driver(&Led.devDrv);
	return LedID;
}

static int led_init(void)
{
	led_gpio_config();
	GPIO_ResetBits(GPIOB, GPIO_Pin_12); //blue -> RED
	GPIO_ResetBits(GPIOB, GPIO_Pin_13); //green -> BLUE
	GPIO_ResetBits(GPIOB, GPIO_Pin_14); //red -> GREEN

	return 1;
}

static int led_ioctrl(unsigned char cmd, void* arg)
{
	uint8_t Color = *(uint8_t *)arg;
	switch(cmd)
	{
	case LED_BOARD:

			break;
	case LED_INDEP:
		if((Color & R_MASK) == R_MASK)
			GPIO_ResetBits(GPIOB, GPIO_Pin_12);
		else
			GPIO_SetBits(GPIOB, GPIO_Pin_12);

		if((Color & G_MASK) == G_MASK)
			GPIO_ResetBits(GPIOB, GPIO_Pin_14);
		else
			GPIO_SetBits(GPIOB, GPIO_Pin_14);

		if((Color & B_MASK) == B_MASK)
			GPIO_ResetBits(GPIOB, GPIO_Pin_13);
		else
			GPIO_SetBits(GPIOB, GPIO_Pin_13);
		break;

		default :
			break;
	}
	return 1;
}

static void led_gpio_config(void){

	GPIO_InitTypeDef	GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
