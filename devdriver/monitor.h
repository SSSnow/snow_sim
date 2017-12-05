#ifndef _MONITOR_H_
#define _MONITOR_H_

#include <stdio.h>
#include "delay.h"
#include "device.h"
#include "driver.h"
#include "mcudriver/sys.h"
#include <stm32f10x_rcc.h>

__packed typedef struct
{
	unsigned char head;
	unsigned char cmd;
	unsigned char length;
	__packed union{
			unsigned char c_data[4];
			float f_data;	
	}data[9];
	unsigned char checksum;
}COMM_DATA;

typedef struct
{
	unsigned char head;
	unsigned char cmd;
	unsigned char length;
	unsigned char buffer[120];
	unsigned char checksum;
}COMM_STR;

__packed typedef union
{
	unsigned char cData[4];
	float fData;
}FloatDataUnion;

__packed typedef struct
{
	unsigned char DataID;
	FloatDataUnion uData;
}GetDataTypeDef;

__packed typedef struct
{
	unsigned char CommType;
	unsigned char CommID;
}CommTypeDef;

__packed typedef union
{
	GetDataTypeDef GetData;
	unsigned char rData[5];
}RecPackTypeDef;

__packed typedef union
{
	CommTypeDef CommData;
	unsigned char rData[2];
}RecCommTypeDef;

typedef enum
{
 MONITOR_IOCTRL_COMM_DATA_WRITE = 0,		 
 MONITOR_IOCTRL_COMM_STR_WRITE = 1,
 MONITOR_IOCTRL_ANO_DATA_WRITE
}MONITOR_IOCTRL;

typedef enum 
{
  ParPack=1,
  CommPack=2,
}RecFlag;

unsigned int monitor_getID(void);
unsigned int monitor_register(void);
static int monitor_init(void);
static int  monitor_ioctrl(unsigned char cmd,void* arg);
static void monitor_usart2_init(void);
static void monitor_send_nbyte(unsigned char* pBuffer,unsigned int len);
static int monitor_write(void* buffer,unsigned int len);
static int monitor_read(void* buffer, unsigned int type);
void monitor_usart2_rx_irq(unsigned char data);

#endif 
