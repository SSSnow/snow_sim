/*
 * middleware.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */
#include "middleware.h"

uint8_t middleware_init(void)
{
	return bsp_init();
}


