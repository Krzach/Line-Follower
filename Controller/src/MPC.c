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



real_t lb[6] = {-3 * PI, -3 * PI, -3 * PI, -3 * PI, -3 * PI, -3 * PI}; // No lower bounds on x
real_t ub[6] = {3 * PI, 3 * PI, 3 * PI, 3 * PI, 3 * PI, 3 * PI};

real_t Hqp[6*6]={0};

real_t fqp[6] = { 0 };



void calculateControl(float goalY[], float prevU[]){

	real_t ubA[6] = {
		3 * PI - prevU[0],
		3 * PI - prevU[1],
		3 * PI - prevU[0],
		3 * PI - prevU[1],
		3 * PI - prevU[0],
		3 * PI - prevU[1]
	};

	real_t lbA[6] = {
			-3 * PI - prevU[0],
			-3 * PI - prevU[1],
			-3 * PI - prevU[0],
			-3 * PI - prevU[1],
			-3 * PI - prevU[0],
			-3 * PI - prevU[1]
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




	Hqp[0] = (37687 * a31to2) / 16200 + +10309.0 / 500000;
	Hqp[1] = 309.0 / 500000 - (37687 * a31to2) / 16200;
	Hqp[2] = (2111 * a31to2) / 1080 + 139.0 / 250000;
	Hqp[3] = 139.0 / 250000 - (2111 * a31to2) / 1080;
	Hqp[4] = (4367 * a31to2) / 2700 + 993.0 / 2000000;
	Hqp[5] = 993.0 / 2000000 - (4367 * a31to2) / 2700;

	Hqp[6] = Hqp[1];
	Hqp[7] = Hqp[0];
	Hqp[8] = Hqp[3];
	Hqp[9] = Hqp[2];
	Hqp[10] = Hqp[5];
	Hqp[11] = Hqp[4];

	Hqp[12] = Hqp[2];
	Hqp[13] = Hqp[3];
	Hqp[14] = (13331 * a31to2) / 8100 + 41011.0 / 2000000;
	Hqp[15] = 1011.0 / 2000000 - (13331 * a31to2) / 8100;
	Hqp[16] = (737 * a31to2) / 540 + 451.0 / 1000000;
	Hqp[17] = 451.0 / 1000000 - (737 * a31to2) / 540;

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
	Hqp[28] = (6127 * a31to2) / 5400 + 8163.0 / 400000;
	Hqp[29] = 163.0 / 400000 - (6127 * a31to2) / 5400;

	Hqp[30] = Hqp[5];
	Hqp[31] = Hqp[4];
	Hqp[32] = Hqp[17];
	Hqp[33] = Hqp[16];
	Hqp[34] = Hqp[29];
	Hqp[35] = Hqp[28];

	real_t fqp[6] = { 0 };

	fqp[0] = (309 * u1) / 100000 - (59 * goalY1) / 100 + (309 * u2) / 100000 - (2795 * a31 * goalY2) / 3 + (37687 * a31to2 * u1) / 108 - (37687 * a31to2 * u2) / 108;
	fqp[1] = (309 * u1) / 100000 - (59 * goalY1) / 100 + (309 * u2) / 100000 + (2795 * a31 * goalY2) / 3 - (37687 * a31to2 * u1) / 108 + (37687 * a31to2 * u2) / 108;
	fqp[2] = (557 * u1) / 200000 - (103 * goalY1) / 200 + (557 * u2) / 200000 - (2270 * a31 * goalY2) / 3 + (10555 * a31to2 * u1) / 36 - (10555 * a31to2 * u2) / 36;
	fqp[3] = (557 * u1) / 200000 - (103 * goalY1) / 200 + (557 * u2) / 200000 + (2270 * a31 * goalY2) / 3 - (10555 * a31to2 * u1) / 36 + (10555 * a31to2 * u2) / 36;
	fqp[4] = (993 * u1) / 400000 - (89 * goalY1) / 200 + (993 * u2) / 400000 - 605 * a31 * goalY2 + (4367 * a31to2 * u1) / 18 - (4367 * a31to2 * u2) / 18;
	fqp[5] = (993 * u1) / 400000 - (89 * goalY1) / 200 + (993 * u2) / 400000 + 605 * a31 * goalY2 - (4367 * a31to2 * u1) / 18 + (4367 * a31to2 * u2) / 18;
	int_t nWSR = 20;



	QProblem_init(Hqp,fqp, Aqp, lb, ub, lbA, ubA, (int_t* const ) &nWSR,
			(real_t* const ) &cputime, &options, uOpt, yOpt, &obj,
			(int_t* const ) &status);


	prevU[0] = uOpt[0] + prevU[0];
	prevU[1] = uOpt[1] + prevU[1];


}

