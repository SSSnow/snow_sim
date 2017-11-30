/*
 * altitude_control.h
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#ifndef _ATTITUDE_CONTROL_H_
#define _ATTITUDE_CONTROL_H_
#include "../middleware/middleware.h"
#include "commander.h"
#include "global_type.h"
#include "algorithm.h"
#include "status_instructions.h"
#include "monitor_debug.h"
#define max(a,b) (a > b)?a:b
#define min(a,b) (a < b)?a:b

void attitude_quat_cal(void);
FLOAT_RPY *get_curEur(void);
Quat *get_att_quat(void);
FLOAT_ACC* get_acc_lowPass(void);
void attitude_angle_loop(void);
void attitude_rate_loop(void);
bool IsFlipped(void);
FLOAT_RPY *get_curEur(void);
void pid_init(void);
void task_1ms_int(void);

#endif /* SRC_MODULES_ATTITUDE_CONTROL_H_ */
