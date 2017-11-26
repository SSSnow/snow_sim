/*
 * bsp.c
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */
#include "bsp.h"

uint8_t bsp_init(void){

	uint8_t COLOR = 0;
	uint8_t deviceInitSta = 0;

	init(led_register());
	COLOR = WHITE;
	ioctrl(led_getID(), LED_INDEP, &COLOR);
	delay_ms(1);

	init(moto_register());
	delay_ms(1);

	init(volt_register());
	delay_ms(1);
	
	if(!init(icm20602_register())) deviceInitSta |= DEV_ERR_ICM20602;
	delay_ms(1);
//	
//	if(!init(ms56xx_register())) deviceInitSta |= DEV_ERR_MS56xx;
//	delay_ms(1);

	init(cc2530_register());
	delay_ms(1);
	
	//init(monitor_register());
	//delay_ms(1);

	return deviceInitSta;
}
