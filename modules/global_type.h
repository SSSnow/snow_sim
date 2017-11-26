/*
 * global_type.h
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#ifndef _GLOBAL_TYPE_H_
#define _GLOBAL_TYPE_H_

typedef struct{
    float qw;
    float qx;
    float qy;
    float qz;
} Quat;

typedef struct
{
    float kp,ki,kd,PreErr,Pout,Iout,Dout,Output,I_max,I_sum,Dt,DLPF, EC,kp_temp;

} PID; //PID

typedef struct
{
	float X;
	float Y;
	float Z;
} FLOAT_XYZ;

typedef struct
{
    float Roll;
    float Pitch;
    float Yaw;
} FLOAT_RPY;

typedef struct
{
    float Roll_rate;
    float Pitch_rate;
    float Yaw_rate;
} FLOAT_RPY_RATE;

typedef struct{
    float accX;
    float accY;
    float accZ;
} FLOAT_ACC;

typedef struct
{
    float gyroX;
    float gyroY;
    float gyroZ;

} FLOAT_GYRO;

typedef struct
{
    short gyroX;
    short gyroY;
    short gyroZ;
} GYRO_RAW_SHORT;

typedef struct
{
    short accX;
    short accY;
    short accZ;
} ACC_RAW_SHORT;

typedef struct
{
    short value;
} TEMP_RAW_SHORT;

typedef struct
{
    int temperature;
    int pressure;
    unsigned char check_flag;

} PRESSURE_RAW;

typedef struct
{
    GYRO_RAW_SHORT* gyro_raw_short;
    ACC_RAW_SHORT* acc_raw_short;
} _6AXIE;


//typedef enum {
//	false = 0,
//	true = 1
//} bool;

typedef struct{
	float x;
	float y;
	float z;
	float r;
}CONTROL;

#endif /* SRC_MODULES_GLOBAL_TYPE_H_ */
