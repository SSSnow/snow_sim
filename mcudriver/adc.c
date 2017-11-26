/*
 * adc.c
 *
 *  Created on: Oct 15, 2017
 *      Author: snow
 */
#include "adc.h"

volatile int16_t ADCConvertedValue[2] = {0};

void  adc_config(void)
{
	ADC_InitTypeDef ADC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |RCC_APB2Periph_ADC1, ENABLE );

	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //72M/6=12

	/* configure PA.1(ADC CHhannel) as alalog input */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	ADC_DeInit(ADC1);

	/* ADC1 configuration */
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	ADC_DMACmd(ADC1,ENABLE);//enable adc dma
	/* ADC1 regular channel1 configure */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_71Cycles5);

	/*Enable ADC */
	ADC_Cmd(ADC1, ENABLE);

	/* Check the end of ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);

	/* Start ADC1 calibration */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/*Start ADC1 calibration */
	ADC_StartCalibration(ADC1);

	/*Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}

void ADC_DMA_Config(void)
{
  DMA_InitTypeDef DMA_InitStructure; //adc 12 bits
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);//enable dma
  DMA_DeInit(DMA1_Channel1);//open dma channel1
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR; // #define ADC1_DR_Address  0x4001244C  ((uint32_t)ADC1_BASE+0x4C)
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADCConvertedValue;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode =DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  DMA_Cmd(DMA1_Channel1,ENABLE);
 // DMA_ITConfig(DMA1_Channel1,DMA_IT_TC, ENABLE);//使能传输完成中断
}

void adc_Init(void){
	ADC_DMA_Config();
	adc_config();
}

u16 adc_Get(void){
	/* Test if the ADC1 EOC flag is set or not */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	/* Returns the ADC1 Master data value of the last converted channel */
	return ADC_GetConversionValue(ADC1);
}

