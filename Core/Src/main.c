/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "drive.h"
#include "sensors.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t ReceiveBuffer[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t ifPID=0;
uint8_t applyStearing=0;
uint8_t recv_char;
uint8_t recv_str[20];
int i=0;

uint16_t sn_data[8];
uint16_t min_values[8];
float errors[3]={0};

PIDparams params;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_TIM10_Init();
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, 16);
  HAL_TIM_Base_Start_IT(&htim10);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  calibrate(min_values);
  calculate_PID_params(0.01,30.0,0.8,0.05,&params);
  HAL_Delay(1000);

  float U = 0;
  int16_t left =0;
  int16_t right=0;
  while (1)
  {
	  if(ifPID){
		  HAL_TIM_Base_Start_IT(&htim9);
		  ifPID =0;

	  }
	  if(applyStearing){
		  get_and_Format_Sn_Data(min_values, sn_data, errors); //get sensor data
		  U=PID(U,&params, errors); //calculate control value
		  left = (int16_t)(130+U);
		  right = (int16_t)(130-U);
		  drive_from_reg(left,right); //apply control
		  applyStearing = 0;
		  ifPID=1;
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Handles receiving data from UART.
  * @param  huart UART handle.
  * @param  Size size of received data.
  * @retval None
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{

    if(huart->Instance == USART1)
    {
    	for(uint8_t i = 0; i < Size; ++i){
    		if((char)ReceiveBuffer[i] == 'r'){
    			uint8_t right = 0;
    			i+=2;
    			//chR = 1;
    			for(; i < Size; ++i){
    				if((char)ReceiveBuffer[i]=='\n') break;
    				right = right * 10 + (ReceiveBuffer[i] - '0');
    			}
    			drive_right(right);
    		}
    		if((char)ReceiveBuffer[i] == 'l'){
    			uint8_t left = 0;
				i+=2;
				//chL = 1;
				for(; i < Size; ++i){
					if((char)ReceiveBuffer[i]=='\n') break;
					left = left * 10 + (ReceiveBuffer[i] - '0');
				}
				drive_left(left);
			}
    		if((char)ReceiveBuffer[i] == 'p'){
    			ifPID=1;
    		}
    		if((char)ReceiveBuffer[i] == 's'){
    			drive_from_reg(0,0);
    			ifPID=0;
    			HAL_TIM_Base_Stop_IT(&htim9);
    		}
    	}
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, 16);
    }
}

/**
  * @brief  This timer callback transmits battery voltage information through UART every 2 seconds.
  * @param  htim TIM handle.
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM10){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		uint16_t value = HAL_ADC_GetValue(&hadc1);
		value = (uint16_t)(value*0.33557 - 943.5);
		uint8_t hundreds = (value - value%100)/100;
		uint8_t tens = (value%100 - value%10)/10;
		uint8_t ones = value%10;
		if(hundreds!=0){
			uint8_t buff[]={'b',' ',hundreds+'0',tens+'0',ones+'0','\n'};
			HAL_UART_Transmit(&huart1,buff,6,1000);
		}else if(tens!=0){
			uint8_t buff[]={'b',' ',tens+'0',ones+'0','\n'};
			HAL_UART_Transmit(&huart1,buff,5,1000);
		}else{
			uint8_t buff[]={'b',' ',ones+'0','\n'};
			HAL_UART_Transmit(&huart1,buff,4,1000);
		}
		__HAL_TIM_SET_COUNTER(&htim10, 0);
		HAL_TIM_Base_Start_IT(&htim10);
	}
	if(htim->Instance == TIM9){
		applyStearing = 1;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
