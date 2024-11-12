/**
  *  @file drive.c
  *	@brief motor lib file
  *  Created on: May 9, 2024
  *      Author: morda
  */


#include "drive.h"
#include "tim.h"

/**
  * @brief  Drives both wheels with values from regulator.
  * @param  left speed of left wheel (-1000, 1000)
  * @param  right speed of right wheel (-1000, 1000)
  * @retval None
  */
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
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);

}

/**
  * @brief  Drives left wheel.
  * @param  speed speed of left wheel (0, 255)
  * @retval None
  */
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
	if(left>1000) left=600;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, left);
}

/**
  * @brief  Drives right wheel.
  * @param  speed speed of right wheel (0, 255)
  * @retval None
  */
void drive_right(int16_t right){
	//int16_t right = speed - 127;
	if(right<0){
		  right = -right;
		  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_RESET);
		  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_SET);
	}else{
		  HAL_GPIO_WritePin(GPIOC,IN3_Pin,GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC,IN4_Pin,GPIO_PIN_RESET);
	}
	//if(right<100) right = 0;
	if(right>1000) right=600;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, right);
}

/**
  * @brief  Calculates parameters for PID controller in discrete form.
  * @param  T sampling time
  * @param  K proportional coefficient
  * @param  Ti integral coefficient
  * @param  Td derivative coefficient
  * @param  p pointer to a data structure with discrete PID parameters
  * @retval None
  */
void calculate_PID_params(float T, float K, float Ti, float Td, PIDparams* p){
	p->r2 = K*Td/T;
	p->r1 = K*(T/(2*Ti)-2*Td/T-1);
	p->r0 = K*(1+T/(2*Ti)+Td/T);
}

/**
  * @brief  Discrete PID controller
  * @param  oldU last control value
  * @param  p pointer to a data structure with discrete PID parameters
  * @param  errors array of errors
  * @retval current control value
  */
float PID(float oldU, PIDparams *p, float errors[]){
	return p->r2*errors[2]+p->r1*errors[1]+p->r0*errors[0]+oldU;
}

