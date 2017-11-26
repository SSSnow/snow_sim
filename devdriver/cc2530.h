/*
 * rf_njzy.h
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */

#ifndef _CC2530_H_
#define _CC2530_H_

#include "device.h"
#include "driver.h"
#include "stdio.h"
#include "mcudriver/sys.h"
#include <stm32f10x_rcc.h>
#include "../modules/algorithm.h"


typedef struct{

	uint8_t h1;  //'N' 0x4E
	uint8_t h2;  //'Y' 0x59
	uint8_t len;
	uint8_t type;
	uint8_t buffer[32];
}NJZY_REC_STR;

typedef struct{
	unsigned char seq;
	unsigned char status;
	unsigned char channel[10];
}NJZY_RF_STR;

typedef enum
{
	CC2530_IOCTRL_BIND_PACKAGE_SEND = 0,
	CC2530_IOCTRL_GET_CONTROL_DATA = 1
}CC2530_IOCTRL;

typedef enum{
	NJZY_PACKET_TYPE_CONTROL = 0x01,
	NJZY_PACKET_TYPE_BIND = 0x02
}NJZY_PACKET_TYPE;

typedef enum{
	HEAD1 = 0,
	HEAD2,
	LEN,
	TYPE,
	DATA
}NJZY_PARSE_PACKET;

uint8_t cc2530_getID(void);
uint8_t cc2530_register(void);
static int cc2530_init(void);
static int cc2530_write(void* buffer, unsigned int len);
static int cc2530_read(void* buffer, unsigned int len);
static void cc2530_usart1_init(void);
//void cc2530_usart1_rx_irqhandle(uint8_t data);
void cc2530_usart1_send_nbyte(unsigned char *pBuffer, unsigned int len);
uint8_t cc2530_parse_packet(uint8_t *buf);
#endif /* SRC_DEVDRIVER_CC2530_H_ */
