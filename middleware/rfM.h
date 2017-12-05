/*
 * rfM.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _RFM_H_
#define _RFM_H_
#include "middleware.h"
#include "global_type.h"

typedef struct{
	uint8_t h1;  //0x4E
	uint8_t h2;  //0x59
	uint8_t len;
	uint8_t type;
	uint8_t buffer[16];
}NJZY_SEND_STR;

typedef union
{
	struct
	{
		unsigned short aThrottle, aRoll, aPitch, aYaw, aRFMode;
	}CtrlVar;
	unsigned short RemoteD[5];
}NJZY_CONTROL_DATA;

void rc_data_update(void);
void rf_relink(void);
uint8_t rfIsLost(void);
NJZY_CONTROL_DATA* get_rf_data(void);
unsigned char get_rf_relink_flag(void);

#endif /* SRC_MIDDLEWARE_RFM_H_ */
