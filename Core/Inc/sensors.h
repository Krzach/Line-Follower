/*
 *  @file sensors.h
 *
 *  Created on: May 9, 2024
 *      Author: morda
 */

#ifndef INC_SENSORS_H_
#define INC_SENSORS_H_

#include "main.h"

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void read(uint16_t sn_data[]);
float map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

void calibrate(uint16_t min_values[]);
void get_and_Format_Sn_Data(uint16_t min_values[], uint16_t sn_data[], float errors[]);

void sendSensorPosition(float position);

#endif /* INC_SENSORS_H_ */
