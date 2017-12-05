/*
 * algorithm.h
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#ifndef _ALGORITHM_H_
#define _ALGORITHM_H_

#include "global_type.h"
#include "attitude_control.h"
#include "math.h"
#define DEG_TO_RAD 0.01745329251994
#define RAD_TO_DEG 57.2957795130823
#define abs(a) (a>0)?a:-a

void sensfusion6UpdateQ(FLOAT_GYRO *gyro, FLOAT_ACC *accel, float dt, volatile Quat *Q);
void EulerAngleToQuaternion1(FLOAT_RPY *eur, volatile Quat *Q);
void Quat2Euler(volatile  Quat *Q, FLOAT_RPY *eur);
void QuaternionMultiplicationCross(Quat *Q_Start, Quat *Q_Rotate, volatile Quat *Q_Terminal);
void QuaternionDiviCross(Quat *Q_Start, Quat *Q_Terminal, volatile Quat *Q_Rotate);
void QuatToRotate(volatile Quat *rotateQ, volatile FLOAT_XYZ *rotateAngle);
void quaternion_rotateVector(Quat *Q, FLOAT_XYZ *from, volatile FLOAT_XYZ *to, unsigned char Dir);
float constrain_float(float amt, float low, float high);
float apply_deadband(float value, float deadband);
float apply_limit(float value, float limit);
void step_change(float *in, float target, float step, float deadBand );
void pid_loop(PID *pid, float eurDesir, float measure);
unsigned char  crcRfCal(unsigned char *buffer, unsigned int len);
float linermap(const float L_max,   const float L_min,  \
              const float tar_max, const float tar_min, \
              float input_t);
void Dcm_from_quat(Quat q, float dcm[3][3]);
void sort(int *buffer, unsigned int len, unsigned char dir);
float invSqrt(float x);
void sensfusion6UpdateEuler(Quat *Q,float *roll, float *pitch);
			  
float fmax_motor(float motor1,float motor2, float motor3, float motor4);
float fmin_motor(float motor1,float motor2, float motor3, float motor4);			  
#endif /* SRC_MODULES_ALGORITHM_H_ */
