#include "stm32f10x.h"
#include <stdio.h>
#include "middleware/middleware.h"
#include "../mcudriver/timer.h"
#include "attitude_control.h"

void test_gpioa2_init(void){
	
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_SetBits(GPIOA,GPIO_Pin_2);
}

int main(){
	//RCC_Config();
	SystemInit();
	NVIC_SetVectorTable(NVIC_VectTab_FLASH,0x0);
	uint8_t DevInitSta = 0x00;
	uint8_t color = 0x00;
	uint8_t sensor_start_failed = 0;
	delay_init();
	DevInitSta = middleware_init();
	if((DevInitSta & DEV_ERR_ICM20602) != 0){
		color = RED|BLUE;
		sensors_init_failed(color,5);
		sensor_start_failed = 1;
	}
	if((DevInitSta & DEV_ERR_MS56xx) != 0){
		color = RED|GREEN;
		sensors_init_failed(color,5);
		sensor_start_failed = 1;
	}

	if(sensor_start_failed){
		while(1){
			color = RED;
			sensors_init_failed(color,1);
		}
	}else{
		into_start_status(1);
		pid_init();
		timer2_int_init();
		while(1){
		}
	}
}

