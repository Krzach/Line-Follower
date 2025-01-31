/*
 * MPC.h
 *
 *  Created on: Nov 5, 2024
 *      Author: kdul
 */

#ifndef MPC_H_
#define MPC_H_

#include <math.h>
#include <qpOASES_wrapper.h>

#define N 8
#define NU 2
#define PI 3.14159265

extern real_t lb[4];
extern real_t ub[4];

extern real_t Aqp[4*4];

extern real_t Hqp[4*4];

extern real_t fqp[4];


/**
 * @brief Calculates control values
 * @param goalY goal point for a car to travel to.
 * @param prevU previous control values
 */
void calculateControl(float goalY[], float prevU[]);


#endif /* MPC_H_ */
