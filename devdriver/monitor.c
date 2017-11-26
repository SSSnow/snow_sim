#include "monitor.h"

static unsigned int  monitorID = 0;

static RecPackTypeDef RecPack_Par = {0};
static RecCommTypeDef RecPack_Comm = {0};

uint8_t DataReceivedFlag = 0;

static DEV monitor = {
	.name = "MONITOR",
	.devDrv = {
		.init =  monitor_init,
		.write = monitor_write,
		.read = monitor_read,
		.ioctrl = monitor_ioctrl
	}
};

unsigned int monitor_getID(void)
{
	return monitorID;
}

unsigned int monitor_register(void)
{
	monitorID = register_driver(&monitor.devDrv);
	return monitorID;
}

static int monitor_init(void)
{
	monitor_usart2_init();
	return 1;
}

static int monitor_write(void* buffer, unsigned int len)
{
	unsigned char* pBuffer = buffer;
	monitor_send_nbyte(pBuffer, len);
	return 1;
}

static int monitor_read(void* buffer, unsigned int type)
{
   switch(type)
	 {	 
		 case ParPack:			 
	        if( DataReceivedFlag & ParPack ) {*((RecPackTypeDef *)buffer) = RecPack_Par; DataReceivedFlag &=~(ParPack);  return 1;}
     break;
	   
		 case CommPack:			 
	        if( DataReceivedFlag & CommPack ) {*((RecCommTypeDef *)buffer) = RecPack_Comm; DataReceivedFlag &=~(CommPack);  return 1;}
     break;
	
	 }
		return 0;
}

static int monitor_ioctrl(unsigned char cmd, void* arg)
{
	switch(cmd)
	{
		case MONITOR_IOCTRL_COMM_DATA_WRITE:
		{
			COMM_DATA* pBuffer = arg;
			monitor_send_nbyte((unsigned char*)pBuffer, sizeof(COMM_DATA));
		}
		break;
		case MONITOR_IOCTRL_COMM_STR_WRITE:
		{
			COMM_STR* pBuffer = arg;
			monitor_send_nbyte((unsigned char*)pBuffer, sizeof(COMM_STR));
		}
		break;
		default: break;
	}
	return 1;
}

static void monitor_usart2_init(void){
		GPIO_InitTypeDef GPIO_InitStructure;
		USART_InitTypeDef USART_InitStructure;
		NVIC_InitTypeDef NVIC_InitStructure;
//		DMA_InitTypeDef DMA_InitStructure;

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	    GPIO_Init(GPIOA, &GPIO_InitStructure);

	    USART_InitStructure.USART_BaudRate = 115200;
	    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	    USART_InitStructure.USART_StopBits = USART_StopBits_1;
	    USART_InitStructure.USART_Parity = USART_Parity_No;
	    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	    USART_Init(USART2, &USART_InitStructure);
	    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
	    USART_Cmd(USART2, ENABLE);

//	    USART_DMACmd(USART1, USART_DMAReq_Tx, ENABLE);
//	    /* Enable USART1 DMA TX request */
//	    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//	    DMA_DeInit(DMA1_Channel7);
//	    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
//	    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)USART1_DMA_TX_Buf;
//	    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
//	    DMA_InitStructure.DMA_BufferSize = 40;
//	    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//	    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
//	    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//	    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//	    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
//	    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
//	    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
//	    DMA_Init(DMA1_Channel7, &DMA_InitStructure);
//	    DMA_ITConfig(DMA1_Channel7, DMA_IT_TC, ENABLE);

//	    /* Enable USART1 DMA TX Finish Interrupt */
//	    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;
//	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
//	    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	    NVIC_Init(&NVIC_InitStructure);

	    /* Enable USART2 Interrupt */
		
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
	    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
}

static void	monitor_send_nbyte(unsigned char* pBuffer,unsigned int len){
	while(len--){
		USART_SendData(USART2, *pBuffer);  
		pBuffer ++;
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET){}  
	}
}

void monitor_usart2_rx_irq(unsigned char data)
{
	static uint8_t RecDataStep = 0;
	static uint8_t DataIndexCnt = 0;
	static uint8_t DataCheck = 0;
	static uint8_t DataLen = 0;
	static uint8_t* DataBuf=NULL;

	switch(RecDataStep)
	{
		case 0:
			if(data == 0x55) RecDataStep ++;
			else RecDataStep = 0;
		break;
		case 1:
			if(data == 0xAA) RecDataStep ++; //param setting packet
			else if(data == 0xAB) RecDataStep ++; //command packet
			else RecDataStep = 0;
		break;
		case 2:
			if(data == 5){RecDataStep ++; DataIndexCnt = 0; DataCheck = 0;DataLen=5;DataBuf=NULL;}
			else if(data == 2){RecDataStep ++; DataIndexCnt = 0; DataCheck = 0;DataLen=2;DataBuf=NULL;}
			else RecDataStep = 0;
		break;
		case 3:
			if(DataLen==5) DataBuf= RecPack_Par.rData;
			if(DataLen==2) DataBuf= RecPack_Comm.rData;
			if(DataIndexCnt < DataLen) DataBuf[DataIndexCnt ++] = data;
			if(DataIndexCnt == DataLen) RecDataStep ++;
		break;
		case 4: //sum check
			for(uint8_t i = 0; i < DataLen; i ++)  DataCheck += DataBuf[i];
			if(data == DataCheck) //check succeed
			{
				if(DataLen==5)DataReceivedFlag |= ParPack;
				if(DataLen==2)DataReceivedFlag |= CommPack;
			}
			RecDataStep = 0;
		break;
		default :
			RecDataStep = 0;
		break;
	}
}

void USART2_IRQHandler(void){
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET){
		 monitor_usart2_rx_irq(USART_ReceiveData(USART2));
	}
}
