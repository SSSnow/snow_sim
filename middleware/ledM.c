/*
 * ledM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */
#include "ledM.h"

void sensors_init_failed(uint8_t color, uint8_t num){
	uint8_t color_blink = BLACK;
	while(num--){
		ioctrl(led_getID(), LED_INDEP, &color);
			delay_ms(500);
		ioctrl(led_getID(), LED_INDEP, &color_blink);
			delay_ms(500);
	}
}

void into_start_status(uint8_t num){
	uint8_t color = RED;
	while(num--){
			ioctrl(led_getID(), LED_INDEP, &color);
			delay_ms(500);
			color = GREEN;
			ioctrl(led_getID(), LED_INDEP, &color);
			delay_ms(500);
			color = BLUE;
			ioctrl(led_getID(), LED_INDEP, &color);
			delay_ms(500);
			color = RED;
		}
}

void led_on(uint8_t color){
	ioctrl(led_getID(), LED_INDEP, &color);
}

