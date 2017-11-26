/*
 * motor.c
 *
 *  Created on: Oct 21, 2017
 *      Author: snow
 */

#include "motor.h"
static unsigned int motoID;
static DEV moto={
	.name = "MOTO",
	.devDrv = {
		.init =  moto_init
	}
};

unsigned int moto_getID(void)
{
	return motoID;
}

unsigned int moto_register(void)
{
	motoID = register_driver(&moto.devDrv);
	return  motoID;
}

static int moto_init(void)
{
	
	TIM3_PWM_Init(0x03E7,0x05);//6kHz
	TIM4_PWM_Init(0x03E7,0x05);
	delay_ms(5);
	return 1;
}

