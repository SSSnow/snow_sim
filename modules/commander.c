/*
 * commander .c
 *
 *  Created on: Oct 17, 2017
 *      Author: snow
 */

#include "commander.h"
#define RC_CHANNAL_MAX 2000
#define RC_CHANNAL_MIN 1000

NJZY_MODE mode = ANGLE_MODE;
bool fly_enable = false;
CONTROL_MODE control_mode = {0};
NJZY_CONTROL_DATA ctrl_data = {0};
CONTROL manual_control = {0};
bool flipped_enable = false;

bool get_fly_status(void){
	return fly_enable;
}

bool get_flipped_status(void){
	return flipped_enable;
}


NJZY_MODE get_mode(void){
	return mode;
}

CONTROL_MODE *get_control_mode(){
	return &control_mode;
}

CONTROL *get_control_val(){
	return &manual_control;
}

void set_control_mode(NJZY_MODE mode){
	switch(mode){
	case RATE_MODE:
		control_mode.control_rate_enable = true;
		control_mode.control_attitude_enable = false;
		control_mode.control_altitude_enable = false;
	break;
	case ANGLE_MODE:
		control_mode.control_rate_enable = true;
		control_mode.control_attitude_enable = true;
		control_mode.control_altitude_enable = false;
		break;
	case BARO_MODE:
		control_mode.control_rate_enable = true;
		control_mode.control_attitude_enable = true;
		control_mode.control_altitude_enable = false;
		break;
	}
}

void handle_rc_data(void){
	static int armed_timeout = 0;
	static uint32_t relink_count = 0;
	static bool fly_enable_last = false;
	rc_data_update();
	if(rfIsLost()){
		relink_count++;
		//if(relink_count % 1000 == 0)
			//rf_relink();
			
	}else{
		relink_count = 0;
		ctrl_data = *get_rf_data();
		if((ctrl_data.CtrlVar.aPitch < 2001 && ctrl_data.CtrlVar.aPitch > 999)||(ctrl_data.CtrlVar.aRoll < 2001 && ctrl_data.CtrlVar.aRoll > 999) \
			|| (ctrl_data.CtrlVar.aYaw < 2001 && ctrl_data.CtrlVar.aYaw > 999)){
			manual_control.x = linermap(RC_CHANNAL_MAX, RC_CHANNAL_MIN, 1, -1, ctrl_data.CtrlVar.aPitch);//pitch
			manual_control.y = linermap(RC_CHANNAL_MAX, RC_CHANNAL_MIN, 1, -1, ctrl_data.CtrlVar.aRoll);//roll
			manual_control.r = linermap(RC_CHANNAL_MAX, RC_CHANNAL_MIN, 1, -1, ctrl_data.CtrlVar.aYaw);//yaw
		}
		manual_control.z = linermap(RC_CHANNAL_MAX, RC_CHANNAL_MIN, 1, 0, ctrl_data.CtrlVar.aThrottle);//throttle

		if(ctrl_data.CtrlVar.aRFMode > 1000 && ctrl_data.CtrlVar.aRFMode < 1300){ //1200 rate_mode
			mode = RATE_MODE;
		}else if(ctrl_data.CtrlVar.aRFMode > 1400 && ctrl_data.CtrlVar.aRFMode < 1600){ //1500 angle_mode
			mode = ANGLE_MODE;
		}else{//1800 BARO_MODE
			mode = BARO_MODE;
		}

		set_control_mode(mode);

		if(IsFlipped()){//1s Z axis rotate > 90 deg flipped
			if(fly_enable_last)
				fly_enable = false; //disable motor

			if(manual_control.z > 0.8){
				flipped_enable = true;
			}

		}else{
			flipped_enable = false;
			if(manual_control.z < 0.1f && manual_control.r > 0.9f && manual_control.x < -0.8 \
				   && manual_control.y < -0.7 && !fly_enable_last && (armed_timeout ++ > 1000)){//"\/" armed 1s timeout
						fly_enable = true;
						armed_timeout = 0;
			}
			if(manual_control.z < 0.05f && manual_control.r < -0.9f && \
				    fly_enable_last && (armed_timeout ++ > 1000)){ // "/\" disarmed 1s timeout
					fly_enable = false;
					armed_timeout = 0;
			}
		}

		fly_enable_last = fly_enable;
	
	}
}
