/*
 * ms56xx.c
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#include "ms56xx.h"
#include "delay.h"
#include "device.h"
#include "driver.h"

#include "iic.h"
#include <stm32f10x.h>

static void ms5611_i2c_init(void);
static u8 ms5611_i2c_write_byte(unsigned char deviceAddr, unsigned char regAddr);
static u8 ms5611_i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer);
static u8 ms5611_reset(void);
static unsigned char ms5611Detect(void);
static unsigned short ms5611_prom(char coef_num);
static signed char ms5611_crc(unsigned short *prom);
static u8 ms5611_read_adc(unsigned int *ADC_Val);
static u8 ms5611_start_ut(void);
static u8 ms5611_get_ut(void);
static u8 ms5611_start_up(void);
static u8 ms5611_get_up(void);
static void ms5611_calculate(int *pressure, int *temperature);
static int ms56xx_init(void);
static int ms56xx_ioctrl(unsigned char cmd, void* arg);

static unsigned int ms5611_ut = 0;  // static result of temperature measurement
static unsigned int ms5611_up = 0;  // static result of pressure measurement
static unsigned short ms5611_c[PROM_NB];  // on-chip ROM 出厂校准字节
static unsigned char ms5611_osr = CMD_ADC_4096;

static unsigned int  ms56xxID = 0;

static DEV ms56xx = {
	.name = "MS56XX",
	.devDrv = {
		.init = ms56xx_init,
		.ioctrl = ms56xx_ioctrl
	}
};

unsigned int ms56xx_getID(void)
{
	return ms56xxID;
}

unsigned int ms56xx_register(void)
{
	ms56xxID = register_driver(&ms56xx.devDrv);
	return  ms56xxID;
}

static void  ms5611_i2c_init(void)
{
	IIC_Init();
}

static u8 ms5611_i2c_write_byte(unsigned char deviceAddr, unsigned char regAddr)
{
	return i2c_write(deviceAddr, regAddr);
}

static u8 ms5611_i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer)
{
	return i2c_read(slaveAddr, readNumber, readBuffer);
}

static uint8_t ms5611_reset(void)
{
	IIC_Start();
	IIC_Send_Byte(MS5611_ADDR<<1);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Send_Byte(CMD_RESET);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Stop();
	return 1;
}

//-------------------------------------------------------------
//                         初始化MS5611
//--------------------------------------------------------------
static unsigned char ms5611Detect(void)
{
	ms5611_reset();	//复位器件
	delay_ms(20);
	//读出16Byte PROM 用于校准
	for (int i = 0; i < PROM_NB; i ++)
		ms5611_c[i] = ms5611_prom(i);

	if (ms5611_crc(ms5611_c) != 0) return 0;

	return 1;
}

//--------------------------------------------------------------
//                         读取2byte Prom
//--------------------------------------------------------------
static unsigned short ms5611_prom(char coef_num)
{
	u8 Sta = 1;
    unsigned char rxbuf[2] = {0, 0};
	Sta &= ms5611_i2c_write_byte(MS5611_ADDR, CMD_PROM_RD + coef_num * 2);
    delay_us(5);
	Sta &= ms5611_i2c_read(MS5611_ADDR, 2, rxbuf); //2个字节PROM
    return rxbuf[0] << 8 | rxbuf[1];
}

//--------------------------------------------------------------
//                      进行PROM 的CRC校验
//--------------------------------------------------------------

static signed char ms5611_crc(unsigned short *prom)
{
    int i, j;
    unsigned int res = 0;
    unsigned char zero = 1;
    unsigned char crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;
    for (i = 0; i < 8; i++) {
        if (prom[i] != 0)
            zero = 0;
    }
    if (zero)
        return -1;

    for (i = 0; i < 16; i++) {
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}
//--------------------------------------------------------------
//                 读出 3Byte/24bit ADC结果
//--------------------------------------------------------------
static u8 ms5611_read_adc(unsigned int *ADC_Val)
{
	unsigned char rxbuf[3];
	if(ms5611_i2c_write_byte(MS5611_ADDR, CMD_ADC_READ))
	{
		if(ms5611_i2c_read(MS5611_ADDR, 3, rxbuf))
		{
			*ADC_Val = (rxbuf[0] << 16) | (rxbuf[1] << 8) | rxbuf[2];
			return 1;
		}
	}
	return 0;
}
//--------------------------------------------------------------
//                 开始温度转换
//--------------------------------------------------------------
static u8 ms5611_start_ut(void)
{
	return ms5611_i2c_write_byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D2 + ms5611_osr);// D2 (temperature) conversion start!
}

static u8 ms5611_get_ut(void)
{
    return ms5611_read_adc(&ms5611_ut);
}
//--------------------------------------------------------------
//                开始气压转换
//--------------------------------------------------------------
static u8 ms5611_start_up(void)
{
	return ms5611_i2c_write_byte(MS5611_ADDR, CMD_ADC_CONV + CMD_ADC_D1 + ms5611_osr);// D1 (pressure) conversion start!
}

static u8 ms5611_get_up(void)
{
    return ms5611_read_adc(&ms5611_up);
}
//--------------------------------------------------------------
//                 根据Prom校准数据
//--------------------------------------------------------------
static void ms5611_calculate(int *pressure, int *temperature)
{
    unsigned int press;
    signed long long temp;
    signed long long delt;
    int dT = (signed long long)ms5611_ut - ((unsigned long long)ms5611_c[5] * 256);
    signed long long off = ((signed long long)ms5611_c[2] << 16) + (((signed long long)ms5611_c[4] * dT) >> 7);
    signed long long sens = ((signed long long)ms5611_c[1] << 15) + (((signed long long)ms5611_c[3] * dT) >> 8);
    temp = 2000 + ((dT * (signed long long)ms5611_c[6]) >> 23);

    if (temp < 2000) { // temperature lower than 20degC
        delt = temp - 2000;
        delt = 5 * delt * delt;
        off -= delt >> 1;
        sens -= delt >> 2;
        if (temp < -1500) { // temperature lower than -15degC
            delt = temp + 1500;
            delt = delt * delt;
            off -= 7 * delt;
            sens -= (11 * delt) >> 1;
        }
    }
    press = ((((signed long long)ms5611_up * sens ) >> 21) - off) >> 15;

    if (pressure)
        *pressure = press;
    if (temperature)
        *temperature = temp;
}

static int ms56xx_init(void)
{
	ms5611_i2c_init();
	return ms5611Detect();
}

static int ms56xx_ioctrl(unsigned char cmd, void* arg)
{
	switch(cmd)
	{
		case MS56XX_IOCTRL_START_PRESSURE:
			return ms5611_start_up();  //pressure
		case MS56XX_IOCTRL_PRESSURE_READ:
			return ms5611_get_up();
		case MS56XX_IOCTRL_START_TEMPERATURE:
			return ms5611_start_ut();
		case MS56XX_IOCTRL_TEMPERATURE_READ:
			return ms5611_get_ut();   //temperature
		case MS56XX_IOCTRL_PRESSURE_CALCULATE:
		{
			int* pbuffer = arg;
			ms5611_calculate(pbuffer, pbuffer + 1);
		}
		return 1;

		case  MS56XX_IOCTRL_RESET:
		return ms5611_reset();

		default: return 0;
	}
}

/*=======
#include "mcudriver/iic.h"
#include "mcudriver/delay.h"

u8 exchange_Pres_num[8];
u8 exchange_Temp_num[8];

u16 Cal_C[7];
u32 D1_Pres,D2_Temp;
float Pressure;
float dT,Temperature,Temperature2;
double OFF,SENS;

static DEV ms5611 = {
		.name = "MS5611",
		.devDrv = {
			.init = MS5611_Init,
			.ioctrl = MS5611_ioctrl
		}
};

void MS561101BA_RESET(void)
{
	IIC_Start();
	IIC_Send_Byte(MS5611_SlaveAddress);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Send_Byte(MS5611_RST);
	while(IIC_Wait_Ack());
	delay_us(100);
	IIC_Stop();
}

void MS5611_Init(void)
{
	MS5611_RESET();
	delay_ms(100);
	MS5611_PROM_READ();
	delay_ms(100);
}

void MS561101BA_PROM_READ(void)
{
	u16 d1,d2;
	u8 i;
	for(i=0;i<=6;i++)
	{
		IIC_Start();
		IIC_Send_Byte(MS5611_SlaveAddress);
		while(IIC_Wait_Ack());
		IIC_Send_Byte((MS5611_PROM_RD+i*2));
		while(IIC_Wait_Ack());

		IIC_Start();
		IIC_Send_Byte(MS5611_SlaveAddress+0x01);
		while(IIC_Wait_Ack());
		d1 = IIC_Read_Byte(1);
		d2 = IIC_Read_Byte(0);
		IIC_Stop();

		delay_ms(10);

		Cal_C[i] = (d1<<8) | d2;
	}
}

u32 MS561101BA_DO_CONVERSION(u8 command)
{
	u32 conversion = 0;
	u32 conv1,conv2,conv3;
	IIC_Start();
	IIC_Send_Byte(MS5611_SlaveAddress);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(command);
	while(IIC_Wait_Ack());
	IIC_Stop();

	delay_ms(20);

	IIC_Start();
	IIC_Send_Byte(MS5611_SlaveAddress);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(0x00);
	while(IIC_Wait_Ack());
	IIC_Stop();

	IIC_Start();
	IIC_Send_Byte(MS5611_SlaveAddress+1);
	while(IIC_Wait_Ack());
	conv1 = IIC_Read_Byte(1);
	conv2 = IIC_Read_Byte(1);
	conv3 = IIC_Read_Byte(0);
	IIC_Stop();

	conversion = (conv1<<16) + (conv2<<8) + conv3;

	return conversion;
}

void MS561101BA_GetTemperature(u8 OSR_Temp)
{

	D2_Temp = MS561101BA_DO_CONVERSION(OSR_Temp);
	delay_ms(10);

	dT = D2_Temp - (((u32)Cal_C[5])<<8);
	Temperature = 2000 + dT * ((u32)Cal_C[6]) / 8388608.0;

}

void MS561101BA_GetPressure(u8 OSR_Pres)
{
	float Aux,OFF2,SENS2;

	D1_Pres = MS561101BA_DO_CONVERSION(OSR_Pres);
	delay_ms(10);

	OFF = (u32)(Cal_C[2]<<16) + ((u32)Cal_C[4]*dT) / 128.0;
	SENS = (u32)(Cal_C[1]<<15) + ((u32)Cal_C[3]*dT) / 256.0;

	if(Temperature < 2000)
	{
		Temperature2 = (dT*dT) / 0x80000000;
		Aux = (Temperature - 2000) * (Temperature - 2000);
		OFF2 = 2.5 * Aux;
		SENS2 = 1.25 * Aux;
		if(Temperature < -1500)
		{
			Aux = (Temperature + 1500) * (Temperature + 1500);
			OFF2 = OFF2 + 7 * Aux;
			SENS2 = SENS + 5.5 * Aux;
		}
	} else  //(Temperature > 2000)
	{
		Temperature2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}

	Temperature = Temperature - Temperature2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	Pressure = (D1_Pres * SENS / 2097152.0 - OFF) / 32768.0;
}

void MS561101BA_Init(void)
{
	MS561101BA_RESET();
	delay_ms(100);
	MS561101BA_PROM_READ();
	delay_ms(100);
}

int MS5611_ioctrl(uint8_t cmd, void* arg)
{
	switch(cmd)
	{
	case MS5611_IOCTL_PROM_READ:
	case MS5611_IOCTL_TEMP_READ:
	case MS5611_IOCTL_PRESSURE_READ:
	}

	return 1;
}*/
