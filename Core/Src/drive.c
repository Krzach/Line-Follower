/**
  *  @file drive.c
  *	@brief motor lib file
  *  Created on: May 9, 2024
  *      Author: kdul
  */


#include "drive.h"
#include "tim.h"


void drive_from_reg(int16_t left,  int16_t right){

		if(left<0){
			  left = -left;
			  HAL_GPIO_WritePin(GPIOC,IN1_Pin,GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC,IN2_Pin,GPIO_PIN_SET);
		}else{
			  HAL_GPIO_WritePin(GPIOC,IN1_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC,IN2_Pin,GPIO_PIN_RESET);
		}
		if(left < 100) left = 0;
		if(left>800) left=800;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);

		if(right<0){
  			  right = -right;
			  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_RESET);
			  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_SET);
		}else{
			  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_SET);
			  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_RESET);
		}
		if(right<100) right = 0;
		if(right>800) right=800;
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);

}


void drive_left(int16_t left){
	//int16_t left = speed - 127;
	if(left<0){
		  left = -left;
		  HAL_GPIO_WritePin(GPIOC,IN1_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC,IN2_Pin,GPIO_PIN_SET);
	}else{
		  HAL_GPIO_WritePin(GPIOC,IN1_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,IN2_Pin,GPIO_PIN_RESET);

	}
	//if(left < 100) left = 0;
	if(left>1000) left=700;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);
}


void drive_right(int16_t right){
	if(right<0){
		  right = -right;
		  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_SET);
	}else{
		  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_RESET);
	}
	if(right>1000) right=700;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
}


void calculate_PID_params(float T, float K, float Ti, float Td, PIDparams* p){
	p->r2 = K*Td/T;
	p->r1 = K*(T/(2*Ti)-2*Td/T-1);
	p->r0 = K*(1+T/(2*Ti)+Td/T);
}

float PID(float oldU, PIDparams *p, float errors[]){
	return p->r2*errors[2]+p->r1*errors[1]+p->r0*errors[0]+oldU;
}

bool have_different_signs(float a, float b) {
    uint32_t a_bits = *(uint32_t*)&a;
    uint32_t b_bits = *(uint32_t*)&b;

    return ((a_bits ^ b_bits) & 0x80000000) != 0;
}

