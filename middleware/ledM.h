/*
 * ledM.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _LEDM_H_
#define _LEDM_H_

#include "middleware.h"
void sensors_init_failed(uint8_t color, uint8_t num);
void into_start_status(uint8_t num);
void led_on(uint8_t color);

#endif /* SRC_MIDDLEWARE_LEDM_H_ */
