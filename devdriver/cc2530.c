/*
 * rf_njzy.c
 *
 *  Created on: Oct 11, 2017
 *      Author: snow
 */
#include "cc2530.h"
#define RX_BUF_LEN 64
static unsigned int cc2530ID = 0;
NJZY_REC_STR cc2530_rec ={0};
NJZY_RF_STR cc2530_remote_rec_packet = {0};
static uint8_t cc2530_rec_data_flag = 0;
uint8_t *USART1_DMA_TX_Buf;
uint8_t USART1_DMA_RX_Buf0[RX_BUF_LEN];
uint8_t USART1_DMA_RX_Buf1[RX_BUF_LEN];
uint8_t using_buf0=0;
uint8_t using_buf1=0; 

static DEV cc2530 = {
		.name = "CC2530",
		.devDrv = {
			.init = cc2530_init,
			.read = cc2530_read,
			.write = cc2530_write
		}
};

uint8_t cc2530_getID(void){
	return cc2530ID;
}

uint8_t cc2530_register(void){
	cc2530ID = register_driver(&cc2530.devDrv);
	return cc2530ID;
}

static int cc2530_init(void){
	cc2530_usart1_init();
	return 1;
}

static int cc2530_read(void* buffer, unsigned int len){
//	if(using_buf0)
//		while(cc2530_parse_packet(USART1_DMA_RX_Buf1));
//	else
//		while(cc2530_parse_packet(USART1_DMA_RX_Buf0));

	if(cc2530_rec_data_flag == 1){
		NJZY_RF_STR* pBuffer = buffer;
			*pBuffer = cc2530_remote_rec_packet;
			cc2530_rec_data_flag = 0;
			return 1;
	}
	return 0;
}

static int cc2530_write(void* buffer, unsigned int len){
	USART1_DMA_TX_Buf = buffer;
	cc2530_usart1_send_nbyte(USART1_DMA_TX_Buf, len);
	return 1;
}

static void cc2530_usart1_init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
		//DMA_InitTypeDef DMA_InitStructure;

		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
		//RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    USART_InitStructure.USART_BaudRate = 115200;
	    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	    USART_InitStructure.USART_Parity = USART_Parity_No;
	    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	    USART_Init(USART1, &USART_InitStructure);
	    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	    USART_Cmd(USART1, ENABLE);
		
//		DMA_DeInit(DMA1_Channel5);
//		DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)(&USART1->DR);
//		DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_DMA_RX_Buf0;
//		DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
//		DMA_InitStructure.DMA_BufferSize = RX_BUF_LEN;
//		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
//	    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
//		DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	    DMA_Init(DMA1_Channel5, &DMA_InitStructure);
//		DMA_ITConfig(DMA1_Channel5,DMA_IT_TC,ENABLE);
//		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//		DMA_Cmd(DMA1_Channel5,ENABLE);
		
//		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
//		NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel5_IRQn;
//		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);

//	    /* Enable USART1 DMA TX request */
//	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	    DMA_DeInit(DMA1_Channel4);
//	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
//	    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_DMA_TX_Buf;
//	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//	    DMA_InitStructure.DMA_BufferSize = 0;
//	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	    DMA_Init(DMA1_Channel4, &DMA_InitStructure);
//	    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

//	    /* Enable USART1 DMA TX Finish Interrupt */
//	    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
//	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
//	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	    NVIC_Init(&NVIC_InitStructure);

	    /* Enable USART1 Interrupt */
		
		//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);

}

void cc2530_usart1_rx_irqhandle(uint8_t data){
	static uint8_t cc2530_step = 0;
	static uint8_t cc2530_count = 0;
	switch(cc2530_step){
		case 0:
			if(data == 0x4E){ //'N'
				cc2530_rec.h1 = data;
				cc2530_step = 1;
			}else{
				cc2530_step = 0;
			}
			break;
		case 1:
			if(data == 0x59){//'Y'
				cc2530_rec.h2 = data;
				cc2530_step = 2;
			}else{
				cc2530_step = 0;
			}
			break;
		case 2:
			if(data == 0x0E){
				cc2530_rec.len = data;
				cc2530_step = 3;
			}else{
				cc2530_step = 0;
			}
			break;
		case 3:
			if( data == NJZY_PACKET_TYPE_CONTROL){
				cc2530_rec.type = data;
				cc2530_count = 0;
				cc2530_step = 4;
			}else{
				cc2530_step = 0;
			}
			break;
		case 4:
			cc2530_rec.buffer[cc2530_count] = data;
			cc2530_count++;
			if(cc2530_count > 32){
				cc2530_step = 0;
				cc2530_count = 0;
			}else if(cc2530_count == (cc2530_rec.len-1)){
				//uint8_t crc = crcRfCal((unsigned char *)&cc2530_rec.len, cc2530_rec.len);
				if(crcRfCal((unsigned char *)&cc2530_rec.len, cc2530_rec.len) == cc2530_rec.buffer[cc2530_rec.len - 2]){				
					cc2530_remote_rec_packet = *(NJZY_RF_STR*)(cc2530_rec.buffer);
					cc2530_rec_data_flag = 1;
				}	
				cc2530_count = 0;
				cc2530_step = 0;		
			}
			break;
		default:
			cc2530_count = 0;
			cc2530_step = 0;
			break;
		}
}

void USART1_IRQHandler(void){
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET){
		//GPIO_ResetBits(GPIOA,GPIO_Pin_2);		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE); 
		cc2530_usart1_rx_irqhandle(USART_ReceiveData(USART1));
		//GPIO_SetBits(GPIOA,GPIO_Pin_2);
	}
}


void cc2530_usart1_send_nbyte(unsigned char *pBuffer, unsigned int len){
	while(len--){
		USART_SendData(USART1, *pBuffer);  
		pBuffer ++;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
	}
}

//void DMA1_Channel5_IRQHandler()
//{
//    if(DMA_GetITStatus(DMA1_IT_TC5)){
//      if(using_buf0 ==0){
//		DMA1_Channel5->CMAR = (u32)USART1_DMA_RX_Buf0;
//        using_buf0 = 1;
//    }else{
//       DMA1_Channel5->CMAR = (u32)USART1_DMA_RX_Buf1;
//       using_buf0 = 0;
//    }
//    DMA_ClearITPendingBit(DMA1_IT_TC5);
//  }    
//}

//void cc2530_usart1_send_nbyte(unsigned char *pBuffer, unsigned int len){
//	while(cc2530UartDmaTxFlag == 0);
//	cc2530UartDmaTxFlag = 0;
//	DMA1_Channel7->CMAR  = (uint32_t)pBuffer;
//	DMA1_Channel7->CNDTR = len;// len
//	DMA_Cmd(DMA1_Channel7, ENABLE);
//}

//void DMA1_Channel4_IRQHandler(void){
//  if(DMA_GetITStatus(DMA1_IT_TC7)) {
//    /* USART1 DMA transmit completed */
//    DMA_ClearITPendingBit(DMA1_IT_TC7);
//    DMA_Cmd(DMA1_Channel7, DISABLE);
//    cc2530UartDmaTxFlag = 1;
//  }
//}

//NJZY_PARSE_PACKET parse_step = HEAD1;
//uint8_t cc2530_parse_packet(uint8_t *buf){
//	static uint8_t cc2530_count = 0;
//	static uint8_t buffer_count = 0;
//	switch(parse_step){
//		case HEAD1:
//			if(0x4E == buf[buffer_count]){ //'N'
//				cc2530_rec.h1 = 0x4E;
//				buffer_count++;
//				if(buffer_count > (RX_BUF_LEN-1)) return 0;
//				parse_step = HEAD2;
//			}else{
//				buffer_count++;
//				if(buffer_count > (RX_BUF_LEN-1)) return 0;
//			}
//			break;
//		case HEAD2:
//			if(0x59 == buf[buffer_count]){//'Y'
//				cc2530_rec.h2 = 0x59;
//				buffer_count++;
//				if(buffer_count > (RX_BUF_LEN-1)) {
//					parse_step = HEAD1;
//					return 0;
//				}
//				parse_step = LEN;
//			}else{
//				buffer_count++;
//				parse_step = HEAD1;
//				if(buffer_count > (RX_BUF_LEN-1)) return 0;
//			}
//			break;
//		case LEN:
//			if((buf[buffer_count] > 0) && (buf[buffer_count] < 32)){
//				cc2530_rec.len = buf[buffer_count];
//				buffer_count++;
//				if(buffer_count > (RX_BUF_LEN-1)) {
//					parse_step = HEAD1;
//					return 0;
//				}
//				parse_step = TYPE;
//			}else{
//				buffer_count++;
//				parse_step = HEAD1;
//				if(buffer_count > (RX_BUF_LEN-1))return 0; 
//			}
//			break;
//		case TYPE:
//			if(buf[buffer_count] == NJZY_PACKET_TYPE_CONTROL){
//				cc2530_rec.type = NJZY_PACKET_TYPE_CONTROL;
//				cc2530_count = 0;
//				buffer_count++;
//				if(buffer_count > (RX_BUF_LEN-1)) {
//					parse_step = HEAD1;
//					return 0;
//				}
//				parse_step = DATA;
//			}else{
//				buffer_count++;
//				parse_step = HEAD1;
//				if(buffer_count > (RX_BUF_LEN-1))return 0;
//			}
//			break;
//		case DATA:
//			cc2530_rec.buffer[cc2530_count] = buf[buffer_count];
//			cc2530_count++;
//			buffer_count++;
//			if(cc2530_count > 31){
//				parse_step = HEAD1;
//				cc2530_count = 0;
//				buffer_count = 0;
//				return 0;
//			}else if(cc2530_count == (cc2530_rec.len-1)){
//				//uint8_t crc = crcRfCal((unsigned char *)&cc2530_rec.len, cc2530_rec.len);
//				if(crcRfCal((unsigned char *)&cc2530_rec.len, cc2530_rec.len)  == cc2530_rec.buffer[cc2530_rec.len - 2]){{
//					cc2530_remote_rec_packet = *(NJZY_RF_STR*)(cc2530_rec.buffer);
//					cc2530_rec_data_flag = 1;
//				}
//				cc2530_count = 0;
//				buffer_count = 0;
//				parse_step = HEAD1;
//				return 0;
//				}
//			}
//			break;
//		default:
//			cc2530_count = 0;
//			buffer_count = 0;
//			parse_step = HEAD1;
//			return 0;
//		}
//		return 1;
//}
