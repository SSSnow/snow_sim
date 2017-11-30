/*
 * icm20602.c
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#include "icm20602.h"

//static float gyro_range_scale = 0.f;
//static float gyro_range_rad_s = 0.f;
//static float accel_range_scale = 0.f;
//static float accel_range_m_s2 = 0.f;

static unsigned int icm20602ID = 0;
static uint8_t MPU_ID = 0;
static DEV icm20602 = {
		.name = "ICM20602",
		.devDrv = {
			.init = icm20602_init,
			.ioctrl = icm20602_ioctrl
		}
};

unsigned int icm20602_getID(void){
	return icm20602ID;
}

unsigned int icm20602_register(void){
	icm20602ID = register_driver(&icm20602.devDrv);
	return icm20602ID;
}

static void icm20602_spi_init(void){

	spi1_Init();
	SPI1_SetSpeed(SPI_BaudRatePrescaler_64);
}

static uint8_t icm20602_write(uint8_t reg, uint8_t data){
	u8 status;
    ICM20602_CS=ENABLE_ICM20602;
	status=SPI1_ReadWriteByte(reg&DIR_WRITE);
	SPI1_ReadWriteByte(data);
	ICM20602_CS=DISABLE_ICM20602;
    return status;

}
/*
 * Note: reg must be consecutive register head address
 */
int icm20602_read(uint8_t reg, uint8_t readLen, uint8_t* readBuffer ){

	u8 reg_val;
    ICM20602_CS=ENABLE_ICM20602;//choose icm20602
	SPI1_ReadWriteByte(reg|0x80); //read
	reg_val=SPI1_ReadWriteLen(readBuffer, NULL, readLen);
	ICM20602_CS=DISABLE_ICM20602;//cancle choose
	return reg_val;
}

static int icm20602_ioctrl(uint8_t cmd, void* arg){
	switch(cmd){
		case ICM20602_IOCTRL_ACCEL_READ:
		{
			uint8_t* pBuffer = arg;	
			if(!icm20602_read(ICM20602_ACC_X_H, ICM20602_ACC_OUT_LEN, pBuffer))
				return 0;
		}
		break;
		case ICM20602_IOCTRL_GYRO_READ:
		{
			uint8_t* pBuffer = arg;
			if(!icm20602_read(ICM20602_GYRO_X_H, ICM20602_GYRO_OUT_LEN, pBuffer))
				return 0;
		}
		break;
		case ICM20602_IOCTRL_TEMP_READ:
		{
			uint8_t* pBuffer = arg;
			if(!icm20602_read(ICM20602_TEMP_H, ICM20602_TEMP_OUT_LEN, pBuffer))
				return 0;
		}
		break;
//		case ICM20602_IOCTRL_ACCEL_M_S2_READ:
//		{
//			uint8_t* pBuffer = arg;
//			*pBuffer = accel_range_m_s2;
//		}
//		break;
//		case ICM20602_IOCTRL_GYRO_RAD_S_READ:
//		{
//			uint8_t* pBuffer = arg;
//			*pBuffer = gyro_range_rad_s;
//		}
//		break;
//		case ICM20602_IOCTRL_ACC_SCALE_READ:
//		{
//			float* pBuffer = arg;
//			*pBuffer = accel_range_scale;
//		}
//		break;
//		case ICM20602_IOCTRL_GYRO_SCALE_READ:
//		{
//			float* pBuffer = arg;
//			*pBuffer = gyro_range_scale;
//		}
//		break;
		case ICM20602_IOCTRL_IMU_READ:
		{
			unsigned char* pBuffer = arg;
			if(!icm20602_read(ICM20602_ACC_X_H, 14, pBuffer))
				return 0;
		}
		break;
	}
	return 1;
}

static int probe(void)
{
	int err = 0;
	return (icm20602_read(ICMREG_WHOAMI, 1, &MPU_ID) > 0 && (MPU_ID == ICM20602_WHO_AM_I)) ? 1 : err;
}

static int icm20602_init(void){
	icm20602_spi_init();//init spi1
	delay_ms(100);
	
	icm20602_write(ICMREG_PWR_MGMT_1, BIT_H_RESET);
	// Note that the ICM20602 starts up in sleep mode, and it can take some time for it to come out of sleep
	delay_ms(20);

	// Wake up device and auto selects the best available clock source.
	icm20602_write(ICMREG_PWR_MGMT_1, BITS_BESTCLOCK_PLL3);
	delay_ms(10);

	// icm20602 select communication interface automatically.

	icm20602_write(ICMREG_SMPLRT_DIV, 0x00);//set sample rate
	delay_ms(1);
	// FS & DLPF   FS=2000 deg/s, DLPF = 20Hz (low pass filter)
	// was 90 Hz, but this runs quality and does not improve the
	// system response
	icm20602_write(ICMREG_ACCEL_CONFIG2,ICM20602_ACC_DLPF_CFG_44HZ);
	delay_ms(1);
	icm20602_write(ICMREG_CONFIG,ICM20602_ACC_DLPF_CFG_44HZ); //44Hz
	delay_ms(1);
	icm20602_write(ICMREG_GYRO_CONFIG,BITS_FS_2000DPS);//+-2000deg/s
	delay_ms(1);

	// correct gyro scale factors
	// scale to rad/s in SI units
	// 2000 deg/s = (2000/180)*PI = 34.906585 rad/s
	// scaling factor:
	// 1/(2^15)*(2000/180)*PI

	//gyro_range_scale = 0.061;//1.0f / (32768.0f * (2000.0f / 180.0f) * M_PI_F);
	//gyro_range_rad_s = 2000.0f ;/// 180.0f) * M_PI_F;

	icm20602_write(ICMREG_ACCEL_CONFIG,ICM20602_ACCEL_SCALE8G); //+-8G
	delay_ms(1);

	//accel_range_scale = 0.002392578125f;//(ICM20602_ONE_G / 4096.0f);
	//accel_range_m_s2 = 16.0f * ICM20602_ONE_G;

	// INT CFG => Interrupt on Data Ready
	icm20602_write(ICMREG_INT_ENABLE, BIT_DATA_RDY_INT_EN);   // INT: Data ready interrupt enable
	delay_ms(1);
	icm20602_write(ICMREG_INT_PIN_CFG, BIT_INT_ANYRD_2CLEAR); // INT: Clear on any read
	delay_ms(2);

	icm20602_write(MPUREG_ICM_UNDOC1, MPUREG_ICM_UNDOC1_VALUE); //refer to define describe
	delay_ms(2);
	
	SPI1_SetSpeed(SPI_BaudRatePrescaler_16);
	return probe();
}

/*
u8 ICM20602_Gyro_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=250)data=7;
	else if(lpf>=176)data=1;
	else if(lpf>=92)data=2;
	else if(lpf>=41)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6;  
	return icm20602_write(ICMREG_CONFIG,data);  
}

u8 ICM20602_Acc_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=218)data=7;
	else if(lpf>=99)data=2;
	else if(lpf>=44)data=3;
	else if(lpf>=21)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return icm20602_write(ICMREG_ACCEL_CONFIG2,data); 
}

u8 ICM20602_Set_LPF(u16 lpf)
{
	ICM20602_Gyro_Set_LPF(lpf/2);
	ICM20602_Acc_Set_LPF(lpf/2);
	return 0;
}

u8 ICM20602_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=icm20602_write(ICMREG_SMPLRT_DIV,data);
 	return ICM20602_Set_LPF(rate);
}

u8 ICM20602_Get_Gyroscope(short *gx,short *gy,short *gz)
{
    u8 buf[6],res;  
	res=icm20602_read(ICM20602_GYRO_X_H,ICM20602_GYRO_OUT_LEN,buf);
	if(res==1)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	
    return res;
}

u8 ICM_Get_Accelerometer(short *ax,short *ay,short *az)
{
	u8 buf[6],res;
	res=icm20602_read(ICM20602_ACC_X_H,ICM20602_ACC_OUT_LEN,buf);
	if(res==1)
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
	return res;
}

short ICM20602_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	icm20602_read(ICM20602_TEMP_H,ICM20602_TEMP_OUT_LEN,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=36.53+((double)raw)/340;  
    return temp*100; //x100
}*/


