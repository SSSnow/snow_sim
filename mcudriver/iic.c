/*
 * iic.c
 *
 *  Created on: Oct 17, 2017
 *      Author: nuaa
 */

#include "iic.h"

//void IIC_Init(void){
//	GPIO_InitTypeDef GPIO_InitStructure;
//	I2C_InitTypeDef I2C_InitStructure;
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,ENABLE);
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
//	GPIO_Init(GPIOB,&GPIO_InitStructure);
//	
//	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
//	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
//	I2C_InitStructure.I2C_OwnAddress1 = 0x0A;  //STM32 I2C addr, set it by user, don't care if it is Master 
//	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
//	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
//	I2C_InitStructure.I2C_ClockSpeed = I2C_SPEED_400K;
//	I2C_Cmd(I2C2,ENABLE);
//	I2C_Init(I2C2,&I2C_InitStructure);	
//}
//FIME: hardware I2C while can't interrupt, therefore we can't use it ,then we use the sofeware i2c
//void I2C_write_byte(u8 SlaveAddr, u8 regAddr, u8 data){
//	I2C_GenerateSTART(I2C2,ENABLE);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(I2C2,SlaveAddr,I2C_Direction_Transmitter);
//	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	
//	I2C_SendData(I2C2,regAddr);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	
//	I2C_SendData(I2C2,data);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	
//	I2C_GenerateSTOP(I2C2,ENABLE);
//}

//void I2C_read_byte(u8 SlaveAddr, u8 regAddr, u8 *recData){
//	while(I2C_GetFlagStatus(I2C2,I2C_FLAG_BUSY));
//	
//	I2C_GenerateSTART(I2C2,ENABLE);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(I2C2,SlaveAddr,I2C_Direction_Transmitter);
//	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
//	
//	I2C_SendData(I2C2,regAddr);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_TRANSMITTED));
//	
//	I2C_GenerateSTART(I2C2,ENABLE);
//	while(!I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_MODE_SELECT));
//	
//	I2C_Send7bitAddress(I2C2,SlaveAddr,I2C_Direction_Receiver);
//	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
//	
//	I2C_AcknowledgeConfig(I2C2,DISABLE);
//	I2C_GenerateSTOP(I2C2,ENABLE); 
//	while(!(I2C_CheckEvent(I2C2,I2C_EVENT_MASTER_BYTE_RECEIVED)));
//	
//	*recData = I2C_ReceiveData(I2C2);
//}

void IIC_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
	   
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;  
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	IIC_SCL = 1;
	IIC_SDA = 1;
// 	RCC->APB2ENR |= 1<<3;
//	GPIOB->CRH &= 0XFFFF00FF;
//	GPIOB->CRH |= 0X00003300;
//	GPIOB->ODR |= 3<<10;
}


void IIC_Start(void)
{
	SDA_OUT();
	IIC_SDA=1;
	IIC_SCL=1;
	delay_us(4);
 	IIC_SDA=0;
	delay_us(4);
	IIC_SCL=0;
}

void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4);
	IIC_SCL=1;
	IIC_SDA=1;
	delay_us(4);
}


u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();
	IIC_SDA=1;delay_us(1);
	IIC_SCL=1;delay_us(1);
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;
	return 0;
}

void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}

void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}

void IIC_Send_Byte(u8 txd)
{
    u8 t;
	SDA_OUT();
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1;
		delay_us(2);
		IIC_SCL=1;
		delay_us(2);
		IIC_SCL=0;
		delay_us(2);
    }
}

u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i = 0; i < 8; i++ )
	{
        IIC_SCL = 0;
        delay_us(2);
		IIC_SCL = 1;
        receive <<= 1;
        if(READ_SDA) receive ++;
		delay_us(1);
    }
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}

u8 i2c_write(unsigned char deviceAddr, unsigned char regAddr)
{
	IIC_Start();
	IIC_Send_Byte(deviceAddr<<1);
	while(IIC_Wait_Ack());
	IIC_Send_Byte(regAddr);
	while(IIC_Wait_Ack());
	IIC_Stop();
	return 1;
}

u8 i2c_read(unsigned char slaveAddr, unsigned int readNumber, unsigned char* readBuffer)
{
	IIC_Start();
	IIC_Send_Byte(slaveAddr<<1|0x01);//read
	while(IIC_Wait_Ack());
	for (unsigned i = readNumber; i > 0; i--)
	{
		readBuffer[readNumber - i] = IIC_Read_Byte(i);
	}
	IIC_Stop();
	return 1;
}


