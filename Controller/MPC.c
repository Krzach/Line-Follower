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


#include <qpOASES_wrapper.h>

#include "MPC.h"


double Aqp[6][6] = {
					{1,	0,	0,	0,	0,	0},
					{0,	1,	0,	0,	0,	0},
					{1,	0,	1,	0,	0,	0},
					{0,	1,	0,	1,	0,	0},
					{1,	0,	1,	0,	1,	0},
					{0,	1,	0,	1,	0,	1}
};

float ubA[6] = {
	3 * PI - prevU[0],
	3 * PI - prevU[1],
	3 * PI - prevU[0],
	3 * PI - prevU[1],
	3 * PI - prevU[0],
	3 * PI - prevU[1]
};

float lba[6] = {
		-3 * PI - prevU[0],
		-3 * PI - prevU[1],
		-3 * PI - prevU[0],
		-3 * PI - prevU[1],
		-3 * PI - prevU[0],
		-3 * PI - prevU[1]
};

double lb[NUM_VAR] = {-1e20, -1e20, -1e20, -1e20, -1e20, -1e20}; // No lower bounds on x
double ub[NUM_VAR] = {1e20, 1e20, 1e20, 1e20, 1e20, 1e20};



void calculateControl(float goalY[], float prevU[], float nextU[]){
	static const float b = 0.0005;
	int nSWR = DEF_nWSR;
	double cputime = 0.007000;

	float u1 = prevU[0];
	float u2 = prevU[1];

	float goalY1 = goalY[0];
	float goalY2 = goalY[1];

	//linearyzacja modelu
	float a31 = b * (u1 + u2);

	float a31to2 = a31 * a31;


	float Hqp[6][6]={0};

	Hqp[0][0] = (37687 * a31to2) / 16200 + 0.002618;
	Hqp[0][1] = 309.0 / 500000 - (37687 * a31to2) / 16200;
	Hqp[0][2] = (2111 * a31to2) / 1080 + 139.0 / 250000;
	Hqp[0][3] = 139.0 / 250000 - (2111 * a31to2) / 1080;
	Hqp[0][4] = (4367 * a31to2) / 2700 + 993.0 / 2000000;
	Hqp[0][5] = 993.0 / 2000000 - (4367 * a31to2) / 2700;

	Hqp[1][0] = Hqp[0][1];
	Hqp[1][1] = Hqp[0][0];
	Hqp[1][2] = Hqp[0][3];
	Hqp[1][3] = Hqp[0][2];
	Hqp[1][4] = Hqp[0][5];
	Hqp[1][5] = Hqp[0][4];

	Hqp[2][0] = Hqp[0][2];
	Hqp[2][1] = Hqp[0][3];
	Hqp[2][2] = (13331 * a31to2) / 8100 + 5011.0 / 2000000;
	Hqp[2][3] = 1011.0 / 2000000 - (13331 * a31to2) / 8100;
	Hqp[2][4] = (737 * a31to2) / 540 + 451.0 / 1000000;
	Hqp[2][5] = 451.0 / 1000000 - (737 * a31to2) / 540;

	Hqp[3][0] = Hqp[0][3];
	Hqp[3][1] = Hqp[0][2];
	Hqp[3][2] = Hqp[2][3];
	Hqp[3][3] = Hqp[2][2];
	Hqp[3][4] = Hqp[2][5];
	Hqp[3][5] = Hqp[2][4];

	Hqp[4][0] = Hqp[0][4];
	Hqp[4][1] = Hqp[0][5];
	Hqp[4][2] = Hqp[2][4];
	Hqp[4][3] = Hqp[2][5];
	Hqp[4][4] = (6127 * a31to2) / 5400 + 963.0 / 400000;
	Hqp[4][5] = 163.0 / 400000 - (6127 * a31to2) / 5400;

	Hqp[5][0] = Hqp[0][5];
	Hqp[5][1] = Hqp[0][4];
	Hqp[5][2] = Hqp[2][5];
	Hqp[5][3] = Hqp[2][4];
	Hqp[5][4] = Hqp[4][5];
	Hqp[5][5] = Hqp[4][4];

	float fqp[6] = { 0 };

	fqp[0] = (309 * u1) / 100000 - (59 * goalY1) / 100 + (309 * u2) / 100000 - (559 * a31 * goalY2) / 6 + (37687 * a31to2 * u1) / 1080 - (37687 * a31to2 * u2) / 1080;
	fqp[1] = (309 * u1) / 100000 - (59 * goalY1) / 100 + (309 * u2) / 100000 + (559 * a31 * goalY2) / 6 - (37687 * a31to2 * u1) / 1080 + (37687 * a31to2 * u2) / 1080;
	fqp[2] = (557 * u1) / 200000 - (103 * goalY1) / 200 + (557 * u2) / 200000 - (227 * a31 * goalY2) / 3 + (2111 * a31to2 * u1) / 72 - (2111 * a31to2 * u2) / 72;
	fqp[3] = (557 * u1) / 200000 - (103 * goalY1) / 200 + (557 * u2) / 200000 + (227 * a31 * goalY2) / 3 - (2111 * a31to2 * u1) / 72 + (2111 * a31to2 * u2) / 72;
	fqp[4] = (993 * u1) / 400000 - (89 * goalY1) / 200 + (993 * u2) / 400000 - (121 * a31 * goalY2) / 2 + (4367 * a31to2 * u1) / 180 - (4367 * a31to2 * u2) / 180;
	fqp[5] = (993 * u1) / 400000 - (89 * goalY1) / 200 + (993 * u2) / 400000 + (121 * a31 * goalY2) / 2 - (4367 * a31to2 * u1) / 180 + (4367 * a31to2 * u2) / 180;

	QProblem_init();


}

