#include "stm32f10x.h"
#include <stdio.h>
#include "middleware/middleware.h"
#include "../mcudriver/timer.h"
#include "attitude_control.h"

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
		get_imu_range_scale();
		timer2_int_init();
		while(1){
		}
	}
}

