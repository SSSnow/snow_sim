#/*
 * ano.h
 *
 *  Created on: Dec 3, 2017
 *      Author: snow
 */

#ifndef _ANO_H_
#define _ANO_H_
#include "middleware.h"

typedef struct 
{
		u8 send_version;
		u8 send_status;
		u8 send_senser;
		u8 send_pid1;
		u8 send_pid2;
		u8 send_pid3;
		u8 send_pid4;
		u8 send_pid5;
		u8 send_pid6;
		u8 send_rcdata;
		u8 send_offset;
		u8 send_motopwm;
		u8 send_power;

}dt_flag_t;


void ANO_DT_Send_Sensor(void);
void ANO_DT_Send_Check(void);

#endif
