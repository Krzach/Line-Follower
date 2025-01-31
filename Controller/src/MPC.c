/*
 * MPC.c
 *
 *  Created on: Nov 5, 2024
 *      Author: kdul
 */

#include "MPC.h"


real_t Aqp[4*4] = {
					1,	0,	0,	0,
					0,	1,	0,	0,
					1,	0,	1,	0,
					0,	1,	0,	1
};



real_t lb[4] = {-6 * PI, -6 * PI, -6 * PI, -6 * PI};
real_t ub[4] = {6 * PI, 6 * PI, 6 * PI, 6 * PI};

real_t Hqp[4*4]={0};

real_t fqp[4] = {0};



void calculateControl(float goalY[], float prevU[]){

	real_t ubA[4] = {
		6 * PI - prevU[0],
		6 * PI - prevU[1],
		6 * PI - prevU[0],
		6 * PI - prevU[1]
	};

	real_t lbA[4] = {
			-6 * PI - prevU[0],
			-6 * PI - prevU[1],
			-6 * PI - prevU[0],
			-6 * PI - prevU[1]
	};

	static const float b = 0.0005;

	real_t cputime = 700000;
	qpOASES_Options options ={0};

	real_t obj;
	int_t status;

	real_t uOpt[4] = {0};
	real_t yOpt[4+4] = {0};

	float u1 = prevU[0];
	float u2 = prevU[1];

	float goalY1 = goalY[0];
	float goalY2 = goalY[1];

	float a31 = b * (u1 + u2);

	float a31to2 = a31 * a31;


	Hqp[0] = (319 * a31to2) / 3240 + 3.0 / 2000;
	Hqp[1] = 1.0 / 10000 - (319 * a31to2) / 3240;
	Hqp[2] = (377 * a31to2) / 5400 + 1.0 / 12500;
	Hqp[3] = 1.0 / 12500 - (377 * a31to2) / 5400;

	Hqp[4] = Hqp[1];
	Hqp[5] = Hqp[0];
	Hqp[6] = Hqp[3];
	Hqp[7] = Hqp[2];

	Hqp[8] = Hqp[2];
	Hqp[9] = Hqp[3];
	Hqp[10] = (811 * a31to2) / 16200 + 367.0 / 250000;
	Hqp[11] = 17.0 / 250000 - (811 * a31to2) / 16200;

	Hqp[12] = Hqp[3];
	Hqp[13] = Hqp[2];
	Hqp[14] = Hqp[11];
	Hqp[15] = Hqp[10];



	fqp[0] = u1 / 10000 - (17 * goalY1) / 500 + u2 / 10000 - (415 * a31 * goalY2) / 18 + (1595 * a31to2 * u1) / 648 - (1595 * a31to2 * u2) / 648;
	fqp[1] = u1 / 10000 - (17 * goalY1) / 500 + u2 / 10000 + (415 * a31 * goalY2) / 18 - (1595 * a31to2 * u1) / 648 + (1595 * a31to2 * u2) / 648;
	fqp[2] = (81 * u1) / 1000000 - (13 * goalY1) / 500 + (81 * u2) / 1000000 - (275 * a31 * goalY2) / 18 + (377 * a31to2 * u1) / 216 - (377 * a31to2 * u2) / 216;
	fqp[3] = (81 * u1) / 1000000 - (13 * goalY1) / 500 + (81 * u2) / 1000000 + (275 * a31 * goalY2) / 18 - (377 * a31to2 * u1) / 216 + (377 * a31to2 * u2) / 216;

	int_t nWSR = 20;


	QProblem_init(Hqp,fqp, Aqp, lb, ub, lbA, ubA, (int_t* const ) &nWSR,
			(real_t* const ) &cputime, &options, uOpt, yOpt, &obj,
			(int_t* const ) &status);


	prevU[0] = uOpt[0] + prevU[0];
	prevU[1] = uOpt[1] + prevU[1];


}

