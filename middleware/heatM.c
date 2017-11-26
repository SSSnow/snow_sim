/*
 heatM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "heatM.h"

void OpenHeater(void)
{
	open(Heat_getID());
}

void CloseHeater(void)
{
	close(Heat_getID());
}
