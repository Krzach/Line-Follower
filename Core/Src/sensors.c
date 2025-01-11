/**
  *  @file sensors.c
  *	@brief sensor lib file
  *  Created on: May 9, 2024
  *      Author: morda
  */

#include "sensors.h"
#include "tim.h"
#include "gpio.h"
#include "usart.h"

#define MAX_VAL 2000
/**
  * @brief  Sets given pin as input pullup.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral
  * @param  GPIO_Pin specifies the port bit to set.
  * @retval None
  */
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  Sets given pin as output.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral
  * @param  GPIO_Pin specifies the port bit to set.
  * @retval None
  */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

/**
  * @brief  Reads data from 8 sensors and saves it to an array.
  * @param  sn_data pointer to an array.
  * @retval None
  */
void read(uint16_t sn_data[]){
	uint8_t state = 0;
	for(uint8_t i = 0; i<8;++i){
		sn_data[i]=MAX_VAL;
	}
	for(;;){
		switch(state){
			case 0:
				HAL_GPIO_WritePin(Sn1_GPIO_Port,Sn1_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn2_GPIO_Port,Sn2_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn3_GPIO_Port,Sn3_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn4_GPIO_Port,Sn4_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn5_GPIO_Port,Sn5_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn6_GPIO_Port,Sn6_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn7_GPIO_Port,Sn7_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(Sn8_GPIO_Port,Sn8_Pin,GPIO_PIN_SET);
				state = 1;
				HAL_TIM_Base_Start(&htim11);
				HAL_TIM_IC_Start(&htim11,TIM_CHANNEL_1);
				__HAL_TIM_SET_COUNTER(&htim11,0);
				break;
			case 1:
				if(__HAL_TIM_GET_COUNTER(&htim11)>11) state = 2;
				break;
			case 2:
				__HAL_TIM_SET_COUNTER(&htim11,0);
				Set_Pin_Input(Sn1_GPIO_Port,Sn1_Pin);
				Set_Pin_Input(Sn2_GPIO_Port,Sn2_Pin);
				Set_Pin_Input(Sn3_GPIO_Port,Sn3_Pin);
				Set_Pin_Input(Sn4_GPIO_Port,Sn4_Pin);
				Set_Pin_Input(Sn5_GPIO_Port,Sn5_Pin);
				Set_Pin_Input(Sn6_GPIO_Port,Sn6_Pin);
				Set_Pin_Input(Sn7_GPIO_Port,Sn7_Pin);
				Set_Pin_Input(Sn8_GPIO_Port,Sn8_Pin);
				state = 3;
				break;
			case 3:
				if(__HAL_TIM_GET_COUNTER(&htim11)>=MAX_VAL) state = 4;

				if(!HAL_GPIO_ReadPin(Sn1_GPIO_Port, Sn1_Pin) && sn_data[0] == MAX_VAL){
					sn_data[0] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn2_GPIO_Port, Sn2_Pin) && sn_data[1] == MAX_VAL){
					sn_data[1] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn3_GPIO_Port, Sn3_Pin) && sn_data[2] == MAX_VAL){
					sn_data[2] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn4_GPIO_Port, Sn4_Pin) && sn_data[3] == MAX_VAL){
					sn_data[3] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn5_GPIO_Port, Sn5_Pin) && sn_data[4] == MAX_VAL){
					sn_data[4] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn6_GPIO_Port, Sn6_Pin) && sn_data[5] == MAX_VAL){
					sn_data[5] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn7_GPIO_Port, Sn7_Pin) && sn_data[6] == MAX_VAL){
					sn_data[6] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				if(!HAL_GPIO_ReadPin(Sn8_GPIO_Port, Sn8_Pin) && sn_data[7] == MAX_VAL){
					sn_data[7] = __HAL_TIM_GET_COUNTER(&htim11);
				}
				break;
			case 4:
				Set_Pin_Output(Sn1_GPIO_Port,Sn1_Pin);
				Set_Pin_Output(Sn2_GPIO_Port,Sn2_Pin);
				Set_Pin_Output(Sn3_GPIO_Port,Sn3_Pin);
				Set_Pin_Output(Sn4_GPIO_Port,Sn4_Pin);
				Set_Pin_Output(Sn5_GPIO_Port,Sn5_Pin);
				Set_Pin_Output(Sn6_GPIO_Port,Sn6_Pin);
				Set_Pin_Output(Sn7_GPIO_Port,Sn7_Pin);
				Set_Pin_Output(Sn8_GPIO_Port,Sn8_Pin);

				return;
		}
	}
}

/**
  * @brief  Linearly transforms the input.
  * @param  x input
  * @param 	in_min input minimal value
  * @param  in_max input maximal value
  * @param 	out_min output minimal value
  * @param  out_max output maximal value
  * @retval output value
  */
float map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max) {
  return (float)((x - in_min) * (out_max - out_min)) / (float)(in_max - in_min) + out_min;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min) * (out_max - out_min)) / (in_max - in_min) + out_min;
}

/**
  * @brief  Calibrates sensor.
  * @param  min_values array of minimal values for each sensor
  * @retval None
  */
void calibrate(uint16_t min_values[]){
	for(uint8_t j=0; j<8; ++j){
		min_values[j]=MAX_VAL;
	}
	uint16_t sn_data[8];
	for(uint8_t i=0; i<20; ++i){
		read(sn_data);
		for(uint8_t j=0; j<8; ++j){
			if(min_values[j]>sn_data[j]){
				min_values[j]=sn_data[j];
			}
		}
		HAL_Delay(100);
	}
}

/**
  * @brief  Reads data from sensors and calculates error.
  * @param  min_values array of minimal values for each sensor
  * @param  sn_data sensor data array
  * @param  errors array of errors
  * @retval None
  */
void get_and_Format_Sn_Data(uint16_t min_values[], uint16_t sn_data[], float errors[]){
	read(sn_data);
	float temp_data[8];
	for(uint8_t i=0;i<8;++i){
		temp_data[i]=map(sn_data[i],min_values[i],MAX_VAL,0,1);
		if(temp_data[i]<0.01) temp_data[i]=0;
	}
	for(uint8_t i = 2; i>0; i--){
		errors[i]=errors[i-1];
	}
	errors[0]=0;
	float sum = 0;
	for(uint8_t i = 0; i<8; ++i){
		errors[0]+=i*temp_data[i];
		sum+=temp_data[i];
	}
	if(sum < 0.5){
		if(errors[1]>0.2){
			errors[0] = 4;
		}else if(errors[1]<-0.2){
			errors[0] = -4;
		}else{
			errors[0]=0;
		}
	}else{
		errors[0]=3.5-errors[0]/sum;
	}

}


void sendSensorPosition(float position){
	uint16_t value = (uint16_t)fmap(position, -4.0, 4.0, 0.0, 800.0);
	uint8_t hundreds = (value - value%100)/100;
	uint8_t tens = (value%100 - value%10)/10;
	uint8_t ones = value%10;
	if(hundreds!=0){
		uint8_t buff[]={hundreds+'0',tens+'0',ones+'0','\n'};
		HAL_UART_Transmit(&huart1,buff,4,1000);
	}else if(tens!=0){
		uint8_t buff[]={tens+'0',ones+'0','\n'};
		HAL_UART_Transmit(&huart1,buff,3,1000);
	}else{
		uint8_t buff[]={ones+'0','\n'};
		HAL_UART_Transmit(&huart1,buff,2,1000);
	}
}
