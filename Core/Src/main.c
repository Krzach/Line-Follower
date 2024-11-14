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
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdlib.h>
#include "drive.h"
#include "sensors.h"
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

//uint8_t ReceiveBuffer[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//uint8_t ifPID=0;
//uint8_t applyStearing=0;
//uint8_t recv_char;
//uint8_t recv_str[20];
//int i=0;
//
//
//
//PIDparams params;
//
//uint16_t K = 80;
//uint16_t baseSpeed = 160;
//float Ti = 1;
//float Td = 0.05;


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
  MX_ADC1_Init();
  MX_TIM11_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
//  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, 16);
  //HAL_TIM_Base_Start_IT(&htim10);

  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  HAL_TIM_Base_Start(&htim10);
  HAL_TIM_IC_Start(&htim10,TIM_CHANNEL_1);

  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //calibrate(min_values);
//  calculate_PID_params(0.01,K,Ti,Td,&params);
//  HAL_Delay(1000);


//  float U = 0;
//  int16_t left =0;
//  int16_t right=0;

  while (1)
  {
//	  right = TIM2->CNT;
//	  left = TIM4->CNT;
//
//
//	  if(ifPID){
//		  HAL_TIM_Base_Start_IT(&htim9);
//		  ifPID =0;
//
//	  }
//	  if(applyStearing){
//		  get_and_Format_Sn_Data(min_values, sn_data, errors); //get sensor data
//		  U=PID(U,&params, errors); //calculate control value
//		  left = (int16_t)(baseSpeed+U);
//		  right = (int16_t)(baseSpeed-U);
//		  drive_from_reg(left,right); //apply control
//		  applyStearing = 0;
//		  ifPID=1;
//	  }
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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
//    	for(uint8_t i = 0; i < Size; ++i){
//    		if((char)ReceiveBuffer[i] == 'k'){
//    			uint8_t value = 0;
//    			i+=2;
//    			//chR = 1;
//    			for(; i < Size; ++i){
//    				if((char)ReceiveBuffer[i]=='\n') break;
//    				value = value * 10 + (ReceiveBuffer[i] - '0');
//    			}
//    			K = value;
//    			calculate_PID_params(0.01,K,Ti,Td,&params);
//    		}
//    		if((char)ReceiveBuffer[i] == 'i'){
//    			uint8_t value = 0;
//    			i+=2;
//    			//chR = 1;
//    			for(; i < Size; ++i){
//    				if((char)ReceiveBuffer[i]=='\n') break;
//    				value = value * 10 + (ReceiveBuffer[i] - '0');
//    			}
//    			Ti = value * 0.05;
//    			calculate_PID_params(0.01,K,Ti,Td,&params);
//    		}
//    		if((char)ReceiveBuffer[i] == 'd'){
//    			uint8_t value = 0;
//    			i+=2;
//    			//chR = 1;
//    			for(; i < Size; ++i){
//    				if((char)ReceiveBuffer[i]=='\n') break;
//    				value = value * 10 + (ReceiveBuffer[i] - '0');
//    			}
//    			Td = value * 0.01;
//    			calculate_PID_params(0.01,K,Ti,Td,&params);
//    		}
//    		if((char)ReceiveBuffer[i] == 'r'){
//    			uint8_t value = 0;
//    			i+=2;
//    			//chR = 1;
//    			for(; i < Size; ++i){
//    				if((char)ReceiveBuffer[i]=='\n') break;
//    				value = value * 10 + (ReceiveBuffer[i] - '0');
//    			}
//    			baseSpeed = value;
//    		}
//    		if((char)ReceiveBuffer[i] == 'p'){
//    			ifPID=1;
//    		}
//    		if((char)ReceiveBuffer[i] == 's'){
//    			drive_from_reg(0,0);
//    			ifPID=0;
//    			//HAL_TIM_Base_Stop_IT(&htim9);
//    		}
//    		if((char)ReceiveBuffer[i] == 'c'){
//    			//calibrate(min_values);
//    		}
//    	}
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, ReceiveBuffer, 16);
    }
}



/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
