/*
 * ms56xx.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _MS56XX_H_
#define _MS56XX_H_

#include "device.h"
#include "stm32f10x.h"

//#define MS5611_SlaveAddress     0xee
#define MS5611_ADDR             0x76
#define CMD_RESET               0x1E // ADC reset command
#define CMD_ADC_READ            0x00 // ADC read command
#define CMD_ADC_CONV            0x40 // ADC conversion command
#define CMD_ADC_D1              0x00 // ADC D1 conversion
#define CMD_ADC_D2              0x10 // ADC D2 conversion
#define CMD_ADC_256             0x00 // ADC OSR=256
#define CMD_ADC_512             0x02 // ADC OSR=512
#define CMD_ADC_1024            0x04 // ADC OSR=1024
#define CMD_ADC_2048            0x06 // ADC OSR=2048
#define CMD_ADC_4096            0x08 // ADC OSR=4096
#define CMD_PROM_RD             0xA0 // Prom read command
#define PROM_NB                 8


typedef enum
{
	MS56XX_IOCTRL_START_PRESSURE = 0,
	MS56XX_IOCTRL_PRESSURE_READ = 1,
	MS56XX_IOCTRL_START_TEMPERATURE = 2,
	MS56XX_IOCTRL_TEMPERATURE_READ = 3,
	MS56XX_IOCTRL_PRESSURE_CALCULATE = 4,
	MS56XX_IOCTRL_RESET =5
}MS56XX_IOCTRL;

unsigned int ms56xx_getID(void);
unsigned int ms56xx_register(void);

#endif
