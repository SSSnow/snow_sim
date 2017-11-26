/*
 * spi.c
 *
 *  Created on: Oct 12, 2017
 *      Author: snow
 */
#include "spi.h"

void spi1_Init(void){ //spi1 for icm20603

 	GPIO_InitTypeDef GPIO_InitStructure;
  	SPI_InitTypeDef  SPI_InitStructure;

  	 //icm20602  support SPI mode 0 and 3 rate up to 10M
  	//Enable clock
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE); //72M / 8 = 9M

  	//configure SPI1 pins: SCK, MISO and MOSI
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	//configure NSS for icm20602
	GPIO_InitStructure.GPIO_Pin	= GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA, GPIO_Pin_4);

  	SPI_Cmd(SPI1, DISABLE); //must disable SPI,then can change mode
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High; //idle high
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge; //odd edge
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8; //defualt setting
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;  //
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_Cmd(SPI1, ENABLE);
	delay_ms(100);

}
/*SPI1 speed configure
 *SPI1=fAPB2/Prescale
 *@ref SPI_BaudRate_Prescaler:SPI_BaudRatePrescaler_2~SPI_BaudRatePrescaler_256  
 *fAPB2->72Mhz:
 */
void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	assert_param(IS_SPI_BAUDRATE_PRESCALER(SPI_BaudRatePrescaler));//?????
	SPI1->CR1&=0XFFC7;//bit 3-5,clear the baudrate
	SPI1->CR1|=SPI_BaudRatePrescaler;	//SET spi1 speed 
	SPI_Cmd(SPI1,ENABLE); 
} 

u8 SPI1_ReadWriteByte(u8 TxData)
{		 			
	u8 retry;
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		retry++;
		if(retry>200)
			return 0;
	} 
	SPI_I2S_SendData(SPI1, TxData);
		
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET) 
	{
		retry++;
		if(retry>200)
			return 0;	
	}
	return ((uint8_t)SPI_I2S_ReceiveData(SPI1));		    
}


u8 SPI1_ReadWriteLen(u8 *out, const u8 *in, int len)
{
    u8 b;
    SPI1->DR;
    while (len--) 
	{
		b = in ? *(in++) : 0xFF;
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}  
		SPI_I2S_SendData(SPI1, b);
		while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
		b = SPI_I2S_ReceiveData(SPI1);
		if (out)
			*(out++) = b;
    }
    return 1;
}

