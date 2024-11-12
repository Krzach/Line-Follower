/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "drive.h"
#include "tim.h"
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_TASK_PERIOD pdMS_TO_TICKS(10)
#define MPC_TASK_PERIOD pdMS_TO_TICKS(100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile float rightSpeed=0;
volatile float leftSpeed=0;

volatile int16_t rightGoal = 100;
volatile int16_t leftGoal = 100;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId pidTaskHandle;
osThreadId mpcTaskHandle;
osMutexId speedLeftMutexHandle;
osMutexId speedRightMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool have_different_signs(float a, float b);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartPIDTask(void const * argument);
void StartMPCTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* definition and creation of speedLeftMutex */
  osMutexDef(speedLeftMutex);
  speedLeftMutexHandle = osMutexCreate(osMutex(speedLeftMutex));

  /* definition and creation of speedRightMutex */
  osMutexDef(speedRightMutex);
  speedRightMutexHandle = osMutexCreate(osMutex(speedRightMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of pidTask */
  osThreadDef(pidTask, StartPIDTask, osPriorityBelowNormal, 0, 128);
  pidTaskHandle = osThreadCreate(osThread(pidTask), NULL);

  /* definition and creation of mpcTask */
  osThreadDef(mpcTask, StartMPCTask, osPriorityLow, 0, 128);
  mpcTaskHandle = osThreadCreate(osThread(mpcTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartPIDTask */
/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPIDTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */

	HAL_TIM_IC_Start(&htim10, TIM_CHANNEL_1);


	uint16_t encoderRightCount = 0;
	uint16_t lastEncoderRightCount = 0;

	uint16_t encoderLeftCount = 0;
	uint16_t lastEncoderLeftCount = 0;

	const float kp = 0.7;
	const float kd = 0.00;
	const float ki = 2;

	float rightE = 0;
	float rightEPrev = 0;
	const float deltaT = 0.01;
	float rightEintegral = 0;

	float leftE = 0;
	float leftEPrev = 0;
	float leftEintegral = 0;

	float diffE = 0;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	/* Infinite loop */
	for(;;)
	{
		/* Right motor */
		encoderRightCount = TIM2->CNT;

		rightSpeed = (encoderRightCount - lastEncoderRightCount)/deltaT;
		lastEncoderRightCount = encoderRightCount;

		rightE = rightGoal - rightSpeed;
		diffE = rightE - rightEPrev;
		if(have_different_signs(rightE, rightEPrev)){
			if(diffE>200 || diffE<-200) rightEintegral = 0;
		}

		rightEintegral = rightEintegral + rightE*deltaT;
		float rightU = kp*(rightE + kd*diffE/deltaT + ki*rightEintegral);
		rightEPrev = rightE;

		/*Left motor */
		encoderLeftCount = TIM4->CNT;

		leftSpeed = (lastEncoderLeftCount - encoderLeftCount)/deltaT;

		lastEncoderLeftCount = encoderLeftCount;

		leftE = leftGoal - leftSpeed;
		diffE = leftE - leftEPrev;
		if(have_different_signs(leftE, leftEPrev)){
			if(diffE>200 || diffE<-200) leftEintegral = 0; //anti-windup
		}
		leftEintegral = leftEintegral + leftE*deltaT;
		float leftU = kp*(leftE + kd*diffE/deltaT + ki*leftEintegral);
		leftEPrev = leftE;


		drive_right((int16_t)rightU);
		drive_left((int16_t)leftU);

		vTaskDelayUntil(&xLastWakeTime, PID_TASK_PERIOD);
	}
  /* USER CODE END StartPIDTask */
}

/* USER CODE BEGIN Header_StartMPCTask */
/**
* @brief Function implementing the mpcTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMPCTask */
void StartMPCTask(void const * argument)
{
  /* USER CODE BEGIN StartMPCTask */
  TickType_t xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */

  uint32_t counter=0;

  for(;;)
  {
	  if(osMutexWait(speedLeftMutexHandle, 100) == osOK){

		  if(counter<20){
			  rightGoal = 300;
			  leftGoal = 300;
		  }else if(counter <= 40){
			  rightGoal = 1300;
			  leftGoal = 1300;
		  }else{
			  counter = 0;
		  }
		  ++counter;
		  osMutexRelease(speedLeftMutexHandle);
	  }

	  vTaskDelayUntil(&xLastWakeTime, MPC_TASK_PERIOD);
  }
  /* USER CODE END StartMPCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
bool have_different_signs(float a, float b) {
    // Cast the float numbers to integers for bitwise operations
    uint32_t a_bits = *(uint32_t*)&a;
    uint32_t b_bits = *(uint32_t*)&b;

    // XOR the sign bits and check if they differ
    return ((a_bits ^ b_bits) & 0x80000000) != 0;
}
/* USER CODE END Application */
