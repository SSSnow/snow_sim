/*
 * sys.c
 *
 *  Created on: Oct 13, 2017
 *      Author: snow
 */

#include "sys.h"

void RCC_Config(void)
{
	RCC_HSEConfig(RCC_HSE_ON);

	while((RCC->CR & RCC_CR_HSIRDY) == 0) {}

    // Enable Prefetch Buffer and set Flash Latency
    FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

    // HCLK = SYSCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    // PCLK = HCLK
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV1;

    // PLL configuration
    RCC_PLLConfig(RCC_PLLSource_HSI_Div2, RCC_PLLMul_12);

    // Enable PLL
    RCC->CR |= RCC_CR_PLLON;//RCC_PLLCmd(ENABLE);

    // Wait till PLL is ready
    while((RCC->CR & RCC_CR_PLLRDY) == 0);

    // Select PLL as system clock source
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    // Wait till PLL is used as system clock source
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
    {
    }
}

//RCC_HSE_Configuration function set HSE to PLL clock source, or callback SystemInit() function

/****
  * function name: RCC_HSE_Configuration
  * Description: RCC set 16M external crystal oscillator
  *	note: none
  **/
void HSE_RCC_Configuration(void){
	RCC_DeInit(); 
	RCC_HSEConfig(RCC_HSE_ON);//set external clock source 
	if(SUCCESS == RCC_WaitForHSEStartUp()){//wait for HSE work
		RCC_HCLKConfig(RCC_SYSCLK_Div1); //set AHB clock (HCLK) RCC_SYSCLK_Div1   AHB clock = system clock
		RCC_PCLK2Config(RCC_HCLK_Div1);//set AHB clock (PCLK2) RCC_HCLK_Div1     APB2 clock = HCLK clock
		RCC_PCLK1Config(RCC_HCLK_Div2);//set AHB clock (PCLK1) RCC_HCLK_Div2     APB2 clock = HCLK/2
		
		FLASH_SetLatency(FLASH_Latency_2);//set FLASH delay clock cycle FLASH_Latency_2 2
		FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);//choose FLASH cache 
		
		RCC_PLLConfig(RCC_PLLSource_HSE_Div2,RCC_PLLMul_9);//16M /2 * 9 = 72M
		RCC_PLLCmd(ENABLE);
		while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){};//wait for PLL colck work
		
		RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);//set system clock (SYSCLK)
		while(RCC_GetSYSCLKSource()!= 0x08); //wait for system work
	}	
	
}
