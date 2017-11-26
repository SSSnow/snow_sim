/*
 * acc_gyro_temp_M.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "acc_gyro_temp_M.h"

#define HALF_SQRT_2 0.707106781f

FLOAT_GYRO gyroUnit = {0};
GYRO_RAW_SHORT gyroRawShort = {0};
FLOAT_ACC accUnit = {0};
ACC_RAW_SHORT accRawShort = {0};
int16_t temperature_RAW = 0;
static RANGE_S acc_range = {0};
static RANGE_S gyro_range = {0};

static ACC_CAL_S acc_scale = {0,1.f,0,1.f,0,1.f};
static GYRO_CAL_S gyro_scale = {0,1.f,0,1.f,0,1.f};

float Gravity = 9.8f;//Init Val


static uint8_t get_gyro_raw(void){
	unsigned char Gyro_ReadBuffer[6] = {0};
	if(ioctrl(icm20602_getID(), ICM20602_IOCTRL_GYRO_READ, (void *)Gyro_ReadBuffer)){
		((char*)&gyroRawShort.gyroX)[0] = Gyro_ReadBuffer[1];
		((char*)&gyroRawShort.gyroX)[1] = Gyro_ReadBuffer[0];

		((char*)&gyroRawShort.gyroY)[0] = Gyro_ReadBuffer[3];
		((char*)&gyroRawShort.gyroY)[1] = Gyro_ReadBuffer[2];

		((char*)&gyroRawShort.gyroZ)[0] = Gyro_ReadBuffer[5];
		((char*)&gyroRawShort.gyroZ)[1] = Gyro_ReadBuffer[4];
		return 1;
	}else
		return 0;
}

static uint8_t get_acc_raw(void){
	unsigned char Acc_ReadBuffer[6] = {0};
		if(ioctrl(icm20602_getID(), ICM20602_IOCTRL_ACCEL_READ, (void *)Acc_ReadBuffer)){
			((char*)&accRawShort.accX)[0] = Acc_ReadBuffer[1];
			((char*)&accRawShort.accX)[1] = Acc_ReadBuffer[0];

			((char*)&accRawShort.accY)[0] = Acc_ReadBuffer[3];
			((char*)&accRawShort.accY)[1] = Acc_ReadBuffer[2];

			((char*)&accRawShort.accZ)[0] = Acc_ReadBuffer[5];
			((char*)&accRawShort.accZ)[1] = Acc_ReadBuffer[4];
			return 1;
		}else
			return 0;

}

static uint8_t get_temp_raw(void){
	unsigned char temp_ReadBuffer[2] = {0};
	if(ioctrl(icm20602_getID(), ICM20602_IOCTRL_TEMP_READ, (void *)temp_ReadBuffer)){
		temperature_RAW = (uint16_t)(temp_ReadBuffer[0] << 8)| temp_ReadBuffer[1];
		return 1;
	}else
		return 0;
}

uint8_t get_imu_range_scale(void){
	uint8_t ret = 1;
	ret &= ioctrl(icm20602_getID(), ICM20602_IOCTRL_ACC_SCALE_READ, &acc_range.scales);
	ret &= ioctrl(icm20602_getID(), ICM20602_IOCTRL_ACCEL_M_S2_READ, &acc_range.units);
	ret &= ioctrl(icm20602_getID(), ICM20602_IOCTRL_GYRO_SCALE_READ, &gyro_range.scales);
	ret &= ioctrl(icm20602_getID(), ICM20602_IOCTRL_GYRO_RAD_S_READ, &gyro_range.units);
	return ret;
}

FLOAT_ACC* get_acc_unit(void){
	get_acc_raw();
	short accX_temp = 0;
	short accY_temp = 0;
	short accZ_temp = 0;
	
	accX_temp = HALF_SQRT_2 * (accRawShort.accX - accRawShort.accY);
	accY_temp = HALF_SQRT_2 * (accRawShort.accX + accRawShort.accY);
	accZ_temp = accRawShort.accZ;
	
	accUnit.accX = (((float)accX_temp * acc_range.scales) - acc_scale.x_offset) * acc_scale.x_scale;
	accUnit.accY = (((float)accY_temp * acc_range.scales) - acc_scale.y_offset) * acc_scale.y_scale;
	accUnit.accZ = (((float)accZ_temp * acc_range.scales) - acc_scale.z_offset) * acc_scale.z_scale;

	return &accUnit;
}

FLOAT_GYRO* get_gyro_unit(void){
	get_gyro_raw();
	short gyroX_temp = 0;
	short gyroY_temp = 0;
	short gyroZ_temp = 0;
	
	gyroX_temp = HALF_SQRT_2 * (gyroRawShort.gyroX - gyroRawShort.gyroY);
	gyroY_temp = HALF_SQRT_2 *  (gyroRawShort.gyroX + gyroRawShort.gyroY);
	gyroZ_temp = gyroRawShort.gyroZ;
	
	gyroUnit.gyroX = (((float)gyroX_temp * gyro_range.scales) - gyro_scale.x_offset) * gyro_scale.x_scale;
	gyroUnit.gyroY = (((float)gyroY_temp * gyro_range.scales) - gyro_scale.y_offset) * gyro_scale.y_scale;
	gyroUnit.gyroZ = (((float)gyroZ_temp * gyro_range.scales) - gyro_scale.z_offset) * gyro_scale.z_scale;

	return &gyroUnit;
}

uint8_t PeaceDataCnt = 0, PeaceDataIndex = 0;

#define BUF_SIZE 100

static GYRO_RAW_SHORT GryPeaceBuf[BUF_SIZE] = {0};
static ACC_RAW_SHORT  AccPeaceBuf[BUF_SIZE] = {0};
float AccModelBuf[BUF_SIZE] = {0};

bool gyro_acc_Peace(unsigned int imu_wait){
	static bool get_imu_scale = false;
	static GYRO_RAW_SHORT gyro_last = {0};
	static uint32_t PeaceTimeCnt = 0;
	static uint8_t CalibrationFlag = 0;
	unsigned int turbulen = 0;
	float AccModRaw = 0.f;
	float AccModRaw_last = 0.f;
	float AccMod = 0.f;
	
	if(!get_imu_scale){
		if(get_imu_range_scale()){
			get_imu_scale = true;
		}else
			return false;
	}
	if(!get_gyro_raw())
		return false;
	if(!get_acc_raw())
		return false;

	turbulen = fabsf(gyroRawShort.gyroX - gyro_last.gyroX) + fabsf(gyroRawShort.gyroY - gyro_last.gyroY) + fabsf(gyroRawShort.gyroZ - gyro_last.gyroZ);
	AccModRaw = sqrtf(accRawShort.accX * accRawShort.accX + accRawShort.accY * accRawShort.accY + accRawShort.accZ * accRawShort.accZ);

	AccMod = sqrtf(get_acc_unit()->accX * get_acc_unit()->accX + get_acc_unit()->accY * get_acc_unit()->accY + get_acc_unit()->accZ * get_acc_unit()->accZ);

	gyro_last = gyroRawShort;

	if(turbulen < 20 && fabsf( AccModRaw_last - AccModRaw) < 60.f){

		if(PeaceTimeCnt < imu_wait){ // continue stable
			PeaceTimeCnt ++;
			return false;

		}else{
			if(CalibrationFlag == 0){
				CalibrationFlag = 1;
				gyro_scale.x_offset = gyro_last.gyroX;
				gyro_scale.y_offset = gyro_last.gyroY;
				gyro_scale.z_offset = gyro_last.gyroZ;
				Gravity = AccMod;
			}else{

				//TODO:this can use ring buffer optimization
				if(PeaceDataCnt < BUF_SIZE){
					GryPeaceBuf[PeaceDataCnt].gyroX = gyro_last.gyroX;
					GryPeaceBuf[PeaceDataCnt].gyroY = gyro_last.gyroY;
					GryPeaceBuf[PeaceDataCnt].gyroZ = gyro_last.gyroZ;

					AccPeaceBuf[PeaceDataCnt].accX = accRawShort.accX;
					AccPeaceBuf[PeaceDataCnt].accY = accRawShort.accY;
					AccPeaceBuf[PeaceDataCnt].accZ = accRawShort.accZ;

					AccModelBuf[PeaceDataCnt] = AccMod;
					PeaceDataCnt ++;
				}else{
					GryPeaceBuf[PeaceDataIndex].gyroX = gyro_last.gyroX;
					GryPeaceBuf[PeaceDataIndex].gyroY = gyro_last.gyroY;
					GryPeaceBuf[PeaceDataIndex].gyroZ = gyro_last.gyroZ;
					AccModelBuf[PeaceDataIndex] = AccMod;
					PeaceDataIndex ++;

					if(PeaceDataIndex == BUF_SIZE)
						PeaceDataIndex = 0;

					gyro_scale.x_offset = gyro_scale.x_offset * 0.9f + GryPeaceBuf[PeaceDataIndex].gyroX * 0.1f;
					gyro_scale.y_offset = gyro_scale.y_offset * 0.9f + GryPeaceBuf[PeaceDataIndex].gyroY * 0.1f;
					gyro_scale.z_offset = gyro_scale.z_offset * 0.9f + GryPeaceBuf[PeaceDataIndex].gyroZ * 0.1f;

					acc_scale.x_offset = acc_scale.x_offset * 0.9f + AccPeaceBuf[PeaceDataIndex].accX * 0.1f;
					acc_scale.y_offset = acc_scale.y_offset * 0.9f + AccPeaceBuf[PeaceDataIndex].accY * 0.1f;
					acc_scale.z_offset = acc_scale.z_offset * 0.9f + AccPeaceBuf[PeaceDataIndex].accZ * 0.1f;

					Gravity = Gravity * 0.99f + AccModelBuf[PeaceDataIndex] * 0.01f;
				}
			}
			return true;
		}
	}else{
			PeaceDataCnt = 0;
			PeaceTimeCnt = 0;
			PeaceDataIndex = 0;
			return false;
	}
}

float GetGravity(void){
	return Gravity;
}

//----------------------------------------Temperature------------------------------------------------
uint8_t get_IMUtemperature_unit(float *Temp){
	if(!get_temp_raw()) return 0;
	else{
		*Temp = (float)temperature_RAW / 326.8f + 25.0f;
		return 1;
	}
}
