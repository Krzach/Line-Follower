/*
 *  @file drive.h
 *
 *
 *  Created on: May 9, 2024
 *      Author: morda
 */

#ifndef INC_DRIVE_H_
#define INC_DRIVE_H_
#include "main.h"

typedef struct {
	float r0;
	float r1;
	float r2;
} PIDparams;

void drive_from_reg(int16_t left,  int16_t right);
void drive_left(uint8_t speed);
void drive_right(uint8_t speed);
void calculate_PID_params(float T, float K, float Ti, float Td, PIDparams* p);
float PID(float oldU, PIDparams *p, float errors[]);

#endif /* INC_DRIVE_H_ */
