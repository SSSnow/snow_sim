/*
 * status_Instructions.c
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#include "status_instructions.h"

/*
 * led status: DISARMED:WHITE fast blink  ARMED->according mode status  RATE_MODE: RED  ANGLE_MODE: BLUE BARO_MDOE: GREEN
 * LOW_BATTERY:RED fast blink this do in control
 */
void led_status_update(void){//10ms task
	static uint32_t led_count = 0;
	if(low_battery_status()){
		if(led_count % 100 > 50)
			led_on(RED);
		else
			led_on(BLACK);
	}else{
		if(rfIsLost()){
			if(led_count % 100 > 50)
				led_on(RED|GREEN);
			else
				led_on(BLACK);
		}else if(get_mode() == RATE_MODE)//orange led, err mode
			led_on(RED);
		else if(get_mode() == ANGLE_MODE)
			led_on(BLUE);
		else if(get_mode() == BARO_MODE)
			led_on(GREEN);
		else
		{
			if(led_count % 1000 > 500)
				led_on(WHITE);
			else
				led_on(BLACK);
		}
	}
	if(led_count < 4000000000)
		led_count ++;
	else
		led_count = 0;
}
