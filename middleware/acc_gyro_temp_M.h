/*
 * acc_gyro_temp_M.h
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#ifndef _ACC_GYRO_TEMP_M_H_
#define _ACC_GYRO_TEMP_M_H_

#include "middleware.h"
#include "../modules/global_type.h"
#include "math.h"

typedef struct{
	float scales;
	float units;
}RANGE_S;

/** accel scaling factors; Vout = Vscale * (Vin + Voffset) */
typedef struct {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
}ACC_CAL_S;

/** gyro scaling factors; Vout = (Vin * Vscale) + Voffset */
typedef struct gyro_calibration_s {
	float	x_offset;
	float	x_scale;
	float	y_offset;
	float	y_scale;
	float	z_offset;
	float	z_scale;
}GYRO_CAL_S;

static uint8_t get_gyro_raw(void);
static uint8_t get_acc_raw(void);
static uint8_t get_temp_raw(void);
uint8_t get_imu_range_scale(void);
bool gyro_acc_Peace(unsigned int imu_wait);
float GetGravity(void);
uint8_t get_IMUtemperature_unit(float *Temp);
FLOAT_ACC* get_acc_unit(void);
FLOAT_GYRO* get_gyro_unit(void);

#endif /* SRC_MIDDLEWARE_ACC_GYRO_TEMP_M_H_ */
