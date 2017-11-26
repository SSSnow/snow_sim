/*
 * spi.h
 *
 *  Created on: Oct 12, 2017
 *      Author: snow
 */

#ifndef _SPI_H_
#define _SPI_H_

#include <stm32f10x.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>
#include "delay.h"

void spi1_Init(void);
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler);
u8 SPI1_ReadWriteByte(u8 TxData);
u8 SPI1_ReadWriteLen(u8 *out, const u8 *in, int len);

#endif /* SRC_MCUDRIVER_SPI_H_ */
