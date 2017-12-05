/*
 * motoM.c
 *
 *  Created on: Oct 14, 2017
 *      Author: snow
 */

#include "motoM.h"

void moto_pwm_output(float motor0,float motor1,float motor2,float motor3, bool reveser_enable){

	motor0 = constrain_float(motor0, 0.f, 1000.f);
	motor1 = constrain_float(motor1, 0.f, 1000.f);
	motor2 = constrain_float(motor2, 0.f, 1000.f);
	motor3 = constrain_float(motor3, 0.f, 1000.f);

	if(reveser_enable){//if need flipped, direct motor2,motor3 output max pwm value

		TIM_SetCompare3(TIM3,0);//motoA+
		TIM_SetCompare4(TIM3,1000);////motoA-

		TIM_SetCompare1(TIM4,0);//motoB+
		TIM_SetCompare2(TIM4,0);//motoB-
	
		TIM_SetCompare3(TIM4,1000);//motoC+
		TIM_SetCompare4(TIM4,0);//motoC-

		TIM_SetCompare1(TIM3,0);//motoD+
		TIM_SetCompare2(TIM3,0);//motoD-
	}else{
		TIM_SetCompare1(TIM3,0);//motoD+
		TIM_SetCompare2(TIM3,(u16)motor0);//motoD-
		
		TIM_SetCompare3(TIM3,(u16)motor1);//motoA+
		TIM_SetCompare4(TIM3,0);//motoA-

		TIM_SetCompare3(TIM4,0);//motoC+
		TIM_SetCompare4(TIM4,(u16)motor2);//motoC-
		
		TIM_SetCompare1(TIM4,(u16)motor3);//motoB+
		TIM_SetCompare2(TIM4,0);//motoB-	

	}
}


