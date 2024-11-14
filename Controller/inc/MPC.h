/*
 * MPC.h
 *
 *  Created on: Nov 5, 2024
 *      Author: morda
 */

#ifndef MPC_H_
#define MPC_H_

#include <math.h>
#include <qpOASES_wrapper.h>

#define N 15
#define NU 3
#define PI 3.14159265

//extern float lbA[6];
//extern float ubA[6];
extern real_t lb[6];
extern real_t ub[6];

extern real_t Aqp[6*6];

extern real_t Hqp[6*6];

extern real_t fqp[6];



void calculateControl(float goalY[], float prevU[]);


#endif /* MPC_H_ */
