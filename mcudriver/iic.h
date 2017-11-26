/*
 * iic.h
 *
 *  Created on: Oct 17, 2017
 *      Author: nuaa
 */

#ifndef _IIC_H_
#define _IIC_H_

#include "stm32f10x.h"
#include "delay.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_i2c.h"

/*PB10 SCL  PB11 SDA*/
#define SDA_IN()  {GPIOB->CRH &= 0xFFFF0FFF; GPIOB->CRH |= 8<<12;}
#define SDA_OUT() {GPIOB->CRH &= 0xFFFF0FFF; GPIOB->CRH |= 3<<12;}

#define IIC_SCL     PCout(10)
#define IIC_SDA     PCout(11)
#define READ_SDA    PCin(11)
//#define I2C_SPEED_400K 400000

void IIC_Init(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Send_Byte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
u8 i2c_write(unsigned char deviceAddr, unsigned char regAddr);
u8 i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer);

#endif /* SRC_DEVDRIVER_IIC_H_ */
