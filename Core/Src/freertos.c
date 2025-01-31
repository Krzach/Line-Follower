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
#include "sensors.h"
#include "tim.h"
#include "MPC.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEAFAULT_TASK_PERIOD pdMS_TO_TICKS(500)
#define MOTOR_PID_TASK_PERIOD pdMS_TO_TICKS(10)
#define MPC_TASK_PERIOD pdMS_TO_TICKS(50)
#define PID_TASK_PERIOD pdMS_TO_TICKS(25)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define MAX_COUNTER 400

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile float rightSpeed=0;
volatile float leftSpeed=0;

volatile int16_t rightGoal = 100;
volatile int16_t leftGoal = 100;

float posHistory[MAX_COUNTER]={0};


uint16_t sn_data[8];
uint16_t min_values[8];
float errors[3]={0};

/* USER CODE END Variables */
osThreadId deafaultTaskHandle;
osThreadId PIDTaskHandle;
osThreadId motorPidTaskHandle;
osThreadId mpcTaskHandle;
osMutexId speedLeftMutexHandle;
osMutexId speedRightMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
//bool have_different_signs(float a, float b);

/* USER CODE END FunctionPrototypes */

void StartDeafaultTask(void const * argument);
void StartPIDTask(void const * argument);
void StartMotorPIDTask(void const * argument);
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

  calibrate(min_values);


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
  osThreadDef(PidTask, StartPIDTask, osPriorityNormal, 0, 128);
  PIDTaskHandle = osThreadCreate(osThread(PidTask), NULL);
  osThreadSuspend(PIDTaskHandle);

  /* definition and creation of pidTask */
  osThreadDef(motorPidTask, StartMotorPIDTask, osPriorityHigh, 0, 128);
  motorPidTaskHandle = osThreadCreate(osThread(motorPidTask), NULL);
  osThreadSuspend(motorPidTaskHandle);

  /* definition and creation of mpcTask */
  osThreadDef(mpcTask, StartMPCTask, osPriorityNormal, 0, 1024);
  mpcTaskHandle = osThreadCreate(osThread(mpcTask), NULL);
  osThreadSuspend(mpcTaskHandle);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadDef(deafaultTask, StartDeafaultTask, osPriorityHigh, 0, 128);
  deafaultTaskHandle = osThreadCreate(osThread(deafaultTask), NULL);
  /* USER CODE END RTOS_THREADS */

}

void StartDeafaultTask(void const * argument){
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();
	driveState oldCarState = STOP;
	for(;;){

		if(oldCarState != carState){
			if(carState == REG_PID){
				osThreadSuspend(motorPidTaskHandle);
				osThreadSuspend(mpcTaskHandle);
				osThreadResume(PIDTaskHandle);
			}else if(carState == REG_MPC){
				osThreadSuspend(PIDTaskHandle);
				osThreadResume(mpcTaskHandle);
				osThreadResume(motorPidTaskHandle);
			}else if(carState == STOP){
				osThreadSuspend(motorPidTaskHandle);
				osThreadSuspend(mpcTaskHandle);
				osThreadSuspend(PIDTaskHandle);
			}
			drive_from_reg(0,0);
			if(oldCarState != STOP && carState == STOP){
				  for(uint16_t i = 0; i < MAX_COUNTER; ++i){
					  sendSensorPosition(posHistory[i]);
				  }
			}
			oldCarState = carState;
		}

		vTaskDelayUntil(&xLastWakeTime, DEAFAULT_TASK_PERIOD);
	}
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	TickType_t xLastWakeTime;
	xLastWakeTime = xTaskGetTickCount();

	const float T=0.025;
	const float K = 60;
	const float Ti = 2;
	const float Td = 0.04;
	PIDparams pidParameters={0};
	calculate_PID_params(T,K,Ti,Td, &pidParameters);

	float U=0;
	float errors[3]={0};
	float baseSpeed = 200;
	uint16_t counter = 0;

  /* Infinite loop */

  for(;;)
  {
	 get_and_Format_Sn_Data(min_values, sn_data, errors);
	 U = PID(U, &pidParameters, errors);
	 baseSpeed = 300 - map((uint16_t)(fabsf(errors[0])*100),0,400,0,300);
	 drive_from_reg((int16_t)(baseSpeed+U),(int16_t)(baseSpeed-U));
	 vTaskDelayUntil(&xLastWakeTime, PID_TASK_PERIOD);
	  if(counter == MAX_COUNTER){
		  carState = STOP;

	  }else{
		  posHistory[counter] = errors[0];
		  ++counter;
	  }

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
void StartMotorPIDTask(void const * argument)
{
  /* USER CODE BEGIN StartPIDTask */
//
//	HAL_TIM_IC_Start(&htim10, TIM_CHANNEL_1);
	uint16_t encoderRightCount = 0;
	uint16_t lastEncoderRightCount = 0;

	uint16_t encoderLeftCount = 0;
	uint16_t lastEncoderLeftCount = 0;

	const float kp = 0.7;
	const float kd = 0.01;
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

		vTaskDelayUntil(&xLastWakeTime, MOTOR_PID_TASK_PERIOD);
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
  float Y[] = {0.14, 0};
  float U[] = {0, 0};

  uint16_t counter = 0;


  QProblem_setup(4,4,4);


  /* Infinite loop */

  for(;;)
  {
	  get_and_Format_Sn_Data(min_values, sn_data, errors);
	  Y[1]= errors[0]/100;
	  calculateControl(Y, U);

	  if(osMutexWait(speedLeftMutexHandle, 100) == osOK){
		  rightGoal = U[1]/(2*PI)*360;
		  leftGoal = U[0]/(2*PI)*360;
		  osMutexRelease(speedLeftMutexHandle);
	  }

	  if(counter == MAX_COUNTER-1){
		  carState = STOP;

	  }else{
	  posHistory[counter] = errors[0];
	  ++counter;
	  }
	  vTaskDelayUntil(&xLastWakeTime, MPC_TASK_PERIOD);
  }
  /* USER CODE END StartMPCTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
