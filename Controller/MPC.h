/*
 * MPC.h
 *
 *  Created on: Nov 5, 2024
 *      Author: morda
 */

#ifndef MPC_H_
#define MPC_H_

#include <math.h>

#define N 15
#define NU 3

extern float lba[6];
extern float ubA[6];
extern double lb[6];
extern double ub[6];

extern double Aqp[6][6];

void calculateControl(float goalY[], float prevU[], float nextU[]);


#endif /* MPC_H_ */
