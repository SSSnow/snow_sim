/*
 * altitude_control.c
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */

#include "attitude_control.h"

extern float IMU_P;
extern float IMU_I;

#define Angle_Pitch_Max 35.f
#define Angle_Roll_Max 35.f

#define Rate_Pitch_Max 90.f
#define Rate_Roll_Max 90.f
#define Rate_Yaw_Max 20.f

#define Vel_Vert_Up_Max 3.f
#define Vel_Vert_Down_Max 2.0f

#define Hover_thr 0.5f
#define Up_Scale 1.f/(1.f - Hover_thr)
#define Down_thr 1.f/Hover_thr
#define PWM_MAX 1000
#define PWM_MIN 100
#define PWM_RANGE (PWM_MAX-PWM_MIN)
#define THR_FACTOR 0.9f

Quat imuQ_6Axie = {1.f, 0.f, 0.f, 0.f};
FLOAT_RPY curEur = {0.f};
FLOAT_RPY expEur = {0.f};
FLOAT_RPY_RATE expRate = {0.f};
FLOAT_RPY rotateEur = {0.f};
FLOAT_RPY errEur = {0.f};
Quat expQ = {0.f};
Quat rotateQ = {0.f};
FLOAT_ACC curAccUnit = {0.f};
FLOAT_ACC acc_lowPass = {0.f};
FLOAT_GYRO curGyroUnit = {0.f};
FLOAT_GYRO gyro_lowPass = {0.f};
float vel_z_sp = 0.f;
float pos_z_err_sp = 0.f;
float throttle = 0.f;

float  moto1PRYOutput;
float  moto2PRYOutput;
float  moto3PRYOutput;
float  moto4PRYOutput;
float  Motor1 = 0;
float  Motor2 = 0;
float  Motor3 = 0;
float  Motor4 = 0;

bool imuStableFlag = false;
bool baroStableFlag = false;

PID pidPitch,pidRoll,pidPitchRate,pidRollRate,pidYawRate;

void attitude_quat_cal(void){//1ms
	get_gyro_acc_temp_raw();
	curAccUnit = *get_acc_unit();
	curGyroUnit = *get_gyro_unit();
	
//	static unsigned int imuStableCount = 0;
//	//wait for imu attitude calculation stable
//	if(imuStableCount < 5000){
//		imuStableCount ++;
//		imuStableFlag = false;
//	}else{
//		imuStableFlag = true;
//	}

	if(!get_fly_status()){ //disarmed state
		IMU_P = 10.0f;
		IMU_I = 0.0f;
	}else{//armed
		IMU_P = 0.6f;
	}

	sensfusion6UpdateQ(&curGyroUnit, &curAccUnit, 0.001, &imuQ_6Axie);
	//sensfusion6UpdateEuler(&imuQ_6Axie, &curEur.Roll, &curEur.Pitch);
	Quat2Euler(&imuQ_6Axie, &curEur);

}

FLOAT_RPY *get_curEur(void){
	return &curEur;
}

Quat *get_att_quat(void){
	return &imuQ_6Axie;
}

FLOAT_ACC* get_acc_lowPass(void){
	acc_lowPass.accX = acc_lowPass.accX * 0.9f + get_acc_unit()->accX * 0.1f;
	acc_lowPass.accY = acc_lowPass.accY * 0.9f + get_acc_unit()->accY * 0.1f;
	acc_lowPass.accZ = acc_lowPass.accZ * 0.9f + get_acc_unit()->accZ * 0.1f;
	return &acc_lowPass;
}


void attitude_angle_loop(void){//5ms
	//expect Euler to expect Quaternion
	EulerAngleToQuaternion1(&expEur, &expQ);
	//quaternion protect
	if(fabs(expQ.qw - imuQ_6Axie.qw) > 1 || fabs(expQ.qz - imuQ_6Axie.qz) > 1){
			expQ.qw = -expQ.qw;
			expQ.qx = -expQ.qx;
			expQ.qy = -expQ.qy;
			expQ.qz = -expQ.qz;
	}

	//get Quaternion error
	QuaternionDiviCross(&imuQ_6Axie, &expQ, &rotateQ);

	//according to  err qauternion rotateQ ，get err angle
	Quat2Euler(&rotateQ, &errEur);

	errEur.Roll = constrain_float(errEur.Roll, -35, 35);
	errEur.Pitch = constrain_float(errEur.Pitch, -35, 35);

	pid_loop(&pidPitch, errEur.Pitch, 0);
	pid_loop(&pidRoll, errEur.Roll, 0);
}

void attitude_rate_loop(void){ //1ms

	pid_loop(&pidPitchRate, expRate.Pitch_rate, curGyroUnit.gyroY);
	pid_loop(&pidRollRate, expRate.Roll_rate,  curGyroUnit.gyroX);
	pid_loop(&pidYawRate, expRate.Yaw_rate , curGyroUnit.gyroZ);
}

void attitude_angle_convert(void){
	
}

FLOAT_XYZ rotateAngle = {0};
Quat curQ = {0};
bool IsFlipped(void){
	static bool Flipped = false;
	static int Flipped_count = 0;
	float z_axis = imuQ_6Axie.qw * imuQ_6Axie.qw - imuQ_6Axie.qx * imuQ_6Axie.qx \
			- imuQ_6Axie.qy * imuQ_6Axie.qy + imuQ_6Axie.qz * imuQ_6Axie.qz;
	if(z_axis < 0.f){//Continuous 1s flipped
		Flipped_count ++;
		if(Flipped_count < 2000){ //wait 2s to flipped
			Flipped_count ++;
		}else{
			Flipped = true;
		}
	}else{
		Flipped = false;
		Flipped_count = 0;
	}
		
	return Flipped;
}

void pid_init(void)
{

	//Pitch params
	pidPitch.kp=6.0f;
	pidPitch.kd=0.12f;
	pidPitch.ki=0.0f;
	pidPitch.I_max=20.f;
	pidPitch.Dt =0.005f;
  //PitchRate params
	pidPitchRate.kp=0.65f;
	pidPitchRate.kd=0.05f;
	pidPitchRate.ki=0.f;
	pidPitchRate.I_max=20.f;
	pidPitchRate.Dt=0.001f;
  //Roll params
	pidRoll.kp=pidPitch.kp;
	pidRoll.kd=pidPitch.kd;
	pidRoll.ki=pidPitch.ki;
	pidRoll.I_max=pidPitch.I_max;
	pidRoll.Dt = pidPitch.Dt;
  //RollRate params
	pidRollRate.kp=pidPitchRate.kp;
	pidRollRate.kd=pidPitchRate.kd;
	pidRollRate.ki=pidPitchRate.ki;
	pidRollRate.I_max=pidPitchRate.I_max;
	pidRollRate.Dt = pidPitchRate.Dt;
  //YawRate
	pidYawRate.kp=1.0f;
	pidYawRate.kd=0.001f;
	pidYawRate.ki=0.00f;
	pidYawRate.I_max=20.0f;
	pidYawRate.Dt = 0.001f;

	//TODO:add altitude params
}

float   P_MotoOutput = 0;
float   R_MotoOutput = 0;
float   Y_MotoOutput = 0;
//TODO: all sp need to adjust line
void task_1ms_int(void){
	
	static uint32_t count_ms = 0;  //32bit = 2^32 * e-3 >> flight time
	static uint32_t control_ms = 0;
	//GPIO_ResetBits(GPIOA,GPIO_Pin_2);
	count_ms ++;
	if(count_ms > 4000000000) count_ms = 0;	

	handle_rc_data();
	
	//calculate attitude angle 
	attitude_quat_cal();
	if(count_ms % 10 == 0){
		led_status_update();
	}

	//6 axis mode and 3 axis mode
	if(get_fly_status()){
		control_ms ++;
		if(control_ms == 4000000000) control_ms = 0;
		if(get_control_mode()->control_attitude_enable){
			if(control_ms%5 == 0){
				expEur.Pitch = -get_control_val()->x * Angle_Pitch_Max;
				expEur.Roll = get_control_val()->y * Angle_Roll_Max;
				
				//attitude_angle_loop();
				pid_loop(&pidPitch, expEur.Pitch, curEur.Pitch);
				pid_loop(&pidRoll, expEur.Roll, curEur.Roll);

				expRate.Pitch_rate = pidPitch.Output;
				expRate.Roll_rate = pidRoll.Output;
			}
		}else{
			expRate.Pitch_rate = get_control_val()->x * Rate_Pitch_Max;
			expRate.Roll_rate = get_control_val()->y * Rate_Roll_Max;
		}

		expRate.Yaw_rate = get_control_val()->r * Rate_Yaw_Max;
		attitude_rate_loop();

		if(get_control_mode()->control_altitude_enable){
			if(control_ms%10 == 0){

				//TODO run alt loop get throttle output
				throttle = 100;
			}
		}else{
			throttle = (get_control_val()->z * PWM_RANGE + PWM_MIN) * THR_FACTOR;
		}

		//TODO:throttle TPA
	if(get_control_val()->z < 0.1f){
		P_MotoOutput = R_MotoOutput = Y_MotoOutput = 0;
		
	}else{
		pidPitchRate.Output = constrain_float(pidPitchRate.Output,-1000.f,1000.f);
		pidRollRate.Output = constrain_float(pidRollRate.Output,-1000.f,1000.f);
		pidYawRate.Output = constrain_float(pidYawRate.Output,-1000.f,1000.f);

		P_MotoOutput = (pidPitchRate.Output/1000.0f)*PWM_RANGE;
		R_MotoOutput = (pidRollRate.Output/1000.0f) *PWM_RANGE;
		Y_MotoOutput = (pidYawRate.Output/1000.0f)  *PWM_RANGE;
	}
		
		

		moto1PRYOutput = +P_MotoOutput  +R_MotoOutput +Y_MotoOutput;
		moto2PRYOutput = -P_MotoOutput  +R_MotoOutput -Y_MotoOutput;
		moto3PRYOutput = -P_MotoOutput  -R_MotoOutput +Y_MotoOutput;
		moto4PRYOutput = +P_MotoOutput  -R_MotoOutput -Y_MotoOutput;

		Motor1 = throttle + moto1PRYOutput;
		Motor2 = throttle + moto2PRYOutput;
		Motor3 = throttle + moto3PRYOutput;
		Motor4 = throttle + moto4PRYOutput;
		float thr_attenuation = 0;
		//do throttle output protect
		if(Motor1>PWM_MAX||Motor2>PWM_MAX||Motor3>PWM_MAX||Motor4>PWM_MAX){
			//find max motor output
			float output_max = max(Motor1,max(Motor2,max(Motor3,Motor4)));
			//lost height protect attitude
			thr_attenuation=throttle-(output_max-PWM_MAX);
			
		}else if(Motor1 < PWM_MIN || Motor2>PWM_MIN || Motor3>PWM_MIN || Motor4>PWM_MIN){
			//find min motor output
			float output_min = min(Motor1,min(Motor2,min(Motor3,Motor4)));
			//lost height protect attitude
			thr_attenuation=throttle-(output_min-PWM_MIN);
		}
		//update attenuation output
		Motor1 = thr_attenuation + moto1PRYOutput;
		Motor2 = thr_attenuation + moto2PRYOutput;
		Motor3 = thr_attenuation + moto3PRYOutput;
		Motor4 = thr_attenuation + moto4PRYOutput;
		moto_pwm_output(Motor1,Motor2,Motor3,Motor4,get_flipped_status());
		
			
	}else{ // fly disable
		control_ms = 0;
		//clear integral
		pidPitch.ki = 0;
		pidPitchRate.ki = 0;
		pidRoll.ki = 0;
		pidRollRate.ki = 0;
		pidYawRate.ki = 0;

		if(IsFlipped())
			moto_pwm_output(0.f,0.f,0.f,0.f,true);
		else
			moto_pwm_output(0.f,0.f,0.f,0.f,false);
	}
	//GPIO_SetBits(GPIOA,GPIO_Pin_2);
}

