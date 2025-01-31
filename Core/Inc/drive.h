/*
 *  @file drive.h
 *
 *
 *  Created on: May 9, 2024
 *      Author: kdul
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_
#include <stdbool.h>
#include "main.h"

typedef struct {
	float r0;
	float r1;
	float r2;
} PIDparams;

/**
 * @brief  hecks if numbers have different signs.
 * @param  a first number
 * @param  b second number
 */
bool have_different_signs(float a, float b);

/**
  * @brief  Drives both wheels with values from regulator.
  * @param  left speed of left wheel (-1000, 1000)
  * @param  right speed of right wheel (-1000, 1000)
  * @retval None
  */
void drive_from_reg(int16_t left,  int16_t right);

/**
  * @brief  Drives left wheel.
  * @param  speed speed of left wheel (0, 255)
  * @retval None
  */
void drive_left(int16_t speed);

/**
  * @brief  Drives right wheel.
  * @param  speed speed of right wheel (0, 255)
  * @retval None
  */
void drive_right(int16_t speed);

/**
  * @brief  Calculates parameters for PID controller in discrete form.
  * @param  T sampling time
  * @param  K proportional coefficient
  * @param  Ti integral coefficient
  * @param  Td derivative coefficient
  * @param  p pointer to a data structure with discrete PID parameters
  * @retval None
  */
void calculate_PID_params(float T, float K, float Ti, float Td, PIDparams* p);

/**
  * @brief  Discrete PID controller
  * @param  oldU last control value
  * @param  p pointer to a data structure with discrete PID parameters
  * @param  errors array of errors
  * @retval current control value
  */
float PID(float oldU, PIDparams *p, float errors[]);

#endif /* INC_DRIVE_H_ */
