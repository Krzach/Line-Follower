/*
 *  @file sensors.h
 *
 *  Created on: May 9, 2024
 *      Author: kdul
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"

/**
  * @brief  Sets given pin as input pullup.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral
  * @param  GPIO_Pin specifies the port bit to set.
  * @retval None
  */
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/**
  * @brief  Sets given pin as output.
  * @param  GPIOx where x can be (A..K) to select the GPIO peripheral
  * @param  GPIO_Pin specifies the port bit to set.
  * @retval None
  */
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

/**
  * @brief  Reads data from 8 sensors and saves it to an array.
  * @param  sn_data pointer to an array.
  * @retval None
  */
void read(uint16_t sn_data[]);

/**
  * @brief  Linearly transforms the input.
  * @param  x input
  * @param 	in_min input minimal value
  * @param  in_max input maximal value
  * @param 	out_min output minimal value
  * @param  out_max output maximal value
  * @retval output value
  */
float map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

/**
  * @brief  Calibrates sensor.
  * @param  min_values array of minimal values for each sensor
  * @retval None
  */
void calibrate(uint16_t min_values[]);

/**
  * @brief  Reads data from sensors and calculates error.
  * @param  min_values array of minimal values for each sensor
  * @param  sn_data sensor data array
  * @param  errors array of errors
  * @retval None
  */
void get_and_Format_Sn_Data(uint16_t min_values[], uint16_t sn_data[], float errors[]);

/**
 * @brief  Sends sensor read to bluetooth module
 * @param  position
 * @retval None
 */
void sendSensorPosition(float position);

#endif /* INC_SENSORS_H_ */
