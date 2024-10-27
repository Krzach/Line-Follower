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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PID_TASK_PERIOD pdMS_TO_TICKS(50)
#define MPC_TASK_PERIOD pdMS_TO_TICKS(100)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId pidTaskHandle;
osThreadId mpcTaskHandle;
osMutexId speedLeftMutexHandle;
osMutexId speedRightMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

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
//	PIDparams params;
//	const float K = 0.1;
//
//	const float Ti = 2;
//	const float Td = 0.05;
//	calculate_PID_params(0.05,K,Ti,Td,&params);
//
	uint16_t goal = 1200;
//
//	float errors[3]={0};
//
	volatile float rightSpeed=0;
	uint16_t lastCount = 0;
//
//	float U=0;

  const float kp = 0.16;
  const float kd = 0.04;
  float ki = 0.5;

  float e = 0;
  float eprev = 0;
  const float deltaT = 0.05;
  float eintegral = 0;

  uint32_t last_ticks=0;
  uint32_t diff = 0;

	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {

	  rightSpeed = (float)(TIM2->CNT - lastCount)/deltaT;

	  lastCount = TIM2->CNT;

	  diff = xLastWakeTime - last_ticks;

	  last_ticks = xLastWakeTime;

	  e = goal - rightSpeed;

	  float dedt = (e-eprev)/(deltaT);
	  eintegral = eintegral + e*deltaT;
	  float u = kp*(e + kd*dedt + ki*eintegral);

	  eprev = e;
//	  errors[2]=errors[1];
//	  errors[1]=errors[0];
//	  errors[0]=goal - right;
//
//	  U=PID(U,&params, errors);
//	  if(U>0) U+=100;
//	  if(U<0) U-=100;
	  drive_right((int16_t)u);

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
  /* Infinite loop */
  for(;;)
  {
	  vTaskDelay(pdMS_TO_TICKS(500));
  }
  /* USER CODE END StartMPCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
