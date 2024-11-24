/*
 * MPC.c
 *
 *  Created on: Nov 5, 2024
 *      Author: morda
 */


#define DEF_nV 5    /** number of variables (np) */
#define DEF_nC 20    /** number of constraints (length of lbA) */
#define DEF_nWSR 20    /** maximum number of working set recalculations */
#define DEF_cputime 0.007000    /** maximum time allocated for qpOASES [sec] */




#include "MPC.h"


real_t Aqp[6*6] = {
					1,	0,	0,	0,	0,	0,
					0,	1,	0,	0,	0,	0,
					1,	0,	1,	0,	0,	0,
					0,	1,	0,	1,	0,	0,
					1,	0,	1,	0,	1,	0,
					0,	1,	0,	1,	0,	1
};



real_t lb[6] = {-6 * PI, -6 * PI, -6 * PI, -6 * PI, -6 * PI, -6 * PI}; // No lower bounds on x
real_t ub[6] = {6 * PI, 6 * PI, 6 * PI, 6 * PI, 6 * PI, 6 * PI};

real_t Hqp[6*6]={0};

real_t fqp[6] = { 0 };



void calculateControl(float goalY[], float prevU[]){

	real_t ubA[6] = {
		6 * PI - prevU[0],
		6 * PI - prevU[1],
		6 * PI - prevU[0],
		6 * PI - prevU[1],
		6 * PI - prevU[0],
		6 * PI - prevU[1]
	};

	real_t lbA[6] = {
			-6 * PI - prevU[0],
			-6 * PI - prevU[1],
			-6 * PI - prevU[0],
			-6 * PI - prevU[1],
			-6 * PI - prevU[0],
			-6 * PI - prevU[1]
	};

	static const float b = 0.0005;

	real_t cputime = 700000;
	qpOASES_Options options ={0};
	//options.printLevel = PL_NONE;
	//options.enableFarBounds = 0;
	real_t obj;
	int_t status;

	real_t uOpt[6] = {0};
	real_t yOpt[6+6] = {0};

	float u1 = prevU[0];
	float u2 = prevU[1];

	float goalY1 = goalY[0];
	float goalY2 = goalY[1];

	//linearyzacja modelu
	float a31 = b * (u1 + u2);

	float a31to2 = a31 * a31;




	Hqp[0] = (1229 * a31to2) / 4050 + 2381.0 / 2000000;
	Hqp[1] = 381.0 / 2000000 - (1229 * a31to2) / 4050;
	Hqp[2] = (1253 * a31to2) / 5400 + 161.0 / 1000000;
	Hqp[3] = 161.0 / 1000000 - (1253 * a31to2) / 5400;
	Hqp[4] = (461 * a31to2) / 2700 + 67.0 / 500000;
	Hqp[5] = 67.0 / 500000 - (461 * a31to2) / 2700;

	Hqp[6] = Hqp[1];
	Hqp[7] = Hqp[0];
	Hqp[8] = Hqp[3];
	Hqp[9] = Hqp[2];
	Hqp[10] = Hqp[5];
	Hqp[11] = Hqp[4];

	Hqp[12] = Hqp[2];
	Hqp[13] = Hqp[3];
	Hqp[14] = (2891 * a31to2) / 16200 + 2281.0 / 2000000;
	Hqp[15] = 281.0 / 2000000 - (2891 * a31to2) / 16200;
	Hqp[16] = (713 * a31to2) / 5400 + 29.0 / 250000;
	Hqp[17] = 29.0 / 250000 - (713 * a31to2) / 5400;

	Hqp[18] = Hqp[3];
	Hqp[19] = Hqp[2];
	Hqp[20] = Hqp[15];
	Hqp[21] = Hqp[14];
	Hqp[22] = Hqp[17];
	Hqp[23] = Hqp[16];

	Hqp[24] = Hqp[4];
	Hqp[25] = Hqp[5];
	Hqp[26] = Hqp[16];
	Hqp[27] = Hqp[17];
	Hqp[28] = (319 * a31to2) / 3240 + 11.0 / 10000;
	Hqp[29] = 1.0 / 10000 - (319 * a31to2) / 3240;

	Hqp[30] = Hqp[5];
	Hqp[31] = Hqp[4];
	Hqp[32] = Hqp[17];
	Hqp[33] = Hqp[16];
	Hqp[34] = Hqp[29];
	Hqp[35] = Hqp[28];

	double fqp[6] = { 0 };

	fqp[0] = (381 * u1) / 2000000 - (53 * goalY1) / 1000 + (381 * u2) / 2000000 - (328 * a31 * goalY2) / 9 + (2458 * a31to2 * u1) / 405 - (2458 * a31to2 * u2) / 405;
	fqp[1] = (381 * u1) / 2000000 - (53 * goalY1) / 1000 + (381 * u2) / 2000000 + (328 * a31 * goalY2) / 9 - (2458 * a31to2 * u1) / 405 + (2458 * a31to2 * u2) / 405;
	fqp[2] = (81 * u1) / 500000 - (43 * goalY1) / 1000 + (81 * u2) / 500000 - (238 * a31 * goalY2) / 9 + (1253 * a31to2 * u1) / 270 - (1253 * a31to2 * u2) / 270;
	fqp[3] = (81 * u1) / 500000 - (43 * goalY1) / 1000 + (81 * u2) / 500000 + (238 * a31 * goalY2) / 9 - (1253 * a31to2 * u1) / 270 + (1253 * a31to2 * u2) / 270;
	fqp[4] = (67 * u1) / 500000 - (17 * goalY1) / 500 + (67 * u2) / 500000 - (166 * a31 * goalY2) / 9 + (461 * a31to2 * u1) / 135 - (461 * a31to2 * u2) / 135;
	fqp[5] = (67 * u1) / 500000 - (17 * goalY1) / 500 + (67 * u2) / 500000 + (166 * a31 * goalY2) / 9 - (461 * a31to2 * u1) / 135 + (461 * a31to2 * u2) / 135;
	int_t nWSR = 20;



	QProblem_init(Hqp,fqp, Aqp, lb, ub, lbA, ubA, (int_t* const ) &nWSR,
			(real_t* const ) &cputime, &options, uOpt, yOpt, &obj,
			(int_t* const ) &status);


	prevU[0] = uOpt[0] + prevU[0];
	prevU[1] = uOpt[1] + prevU[1];


}

