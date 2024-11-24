/*
 *  @file drive.h
 *
 *
 *  Created on: May 9, 2024
 *      Author: morda
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

bool have_different_signs(float a, float b);

void drive_from_reg(int16_t left,  int16_t right);
void drive_left(int16_t speed);
void drive_right(int16_t speed);
void calculate_PID_params(float T, float K, float Ti, float Td, PIDparams* p);
float PID(float oldU, PIDparams *p, float errors[]);

#endif /* INC_DRIVE_H_ */
