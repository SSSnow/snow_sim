/*
 * commander.h
 *
 *  Created on: Oct 17, 2017
 *      Author: snow
 */

#ifndef _COMMANDER_H_
#define _COMMANDER_H_

#include "stdbool.h"
#include "global_type.h"
#include "../middleware/middleware.h"
#include "attitude_control.h"

typedef enum{
	RATE_MODE = 1,
	ANGLE_MODE,
	BARO_MODE,

}NJZY_MODE;


typedef struct{
	unsigned char bindbuffer[8]; //data refer to bindbuffer array;
}NJZY_BIND_PACKET;

typedef enum{
	DISARMED = 1,
	ARMED
}ARM_STA;

typedef struct{
	bool control_rate_enable;
	bool control_attitude_enable;
	bool control_altitude_enable;
}CONTROL_MODE;

void handle_rc_data(void);
bool get_fly_status(void);
bool get_flipped_status(void);
NJZY_MODE get_mode(void);
CONTROL_MODE *get_control_mode(void);
CONTROL *get_control_val(void);
void set_control_mode(NJZY_MODE mode);

#endif /* SRC_MODULES_COMMANDER_H_ */
