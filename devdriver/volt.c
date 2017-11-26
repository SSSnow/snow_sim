/*
 * volt.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */
#include "volt.h"

static unsigned int  voltID = 0;

static DEV volt={
	.name = "VOLT",
	.devDrv={
		.init =  volt_init,
		.read =  volt_read,
	}
};

unsigned int volt_getID(void)
{
	return voltID;
}
unsigned int volt_register(void)
{
	voltID = register_driver(&volt.devDrv);
	return  voltID;
}

static int volt_init(void){
	adc_Init();
	return 1;
}

static int volt_read(void* buffer,unsigned int len){
	short* pBuffer = buffer;
	*pBuffer = adc_Get();

	return 1;
}
