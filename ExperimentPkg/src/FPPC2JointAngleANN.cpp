#include<iostream>
#include<math.h>
#include "ros/ros.h"
#include "ExperimentPkg/Parameter.h"
#include "ExperimentPkg/FPPC2JointAngleANN.h"
#include <iostream>

using namespace std;

double InMaxMinBucket[6][2] = HDY_InMaxMinBucket;
double OutMaxMinBucket[1][2] = HDY_OutMaxMinBucket;
double IWBucket[5][6] = HDY_IWBucket;
double Lw1Bucket[5][5] = HDY_Lw1Bucket;
double Lw2Bucket[5][5] = HDY_Lw2Bucket;
double Lw3Bucket[1][5] = HDY_Lw3Bucket;
double b1Bucket[5][1] = HDY_b1Bucket;
double b2Bucket[5][1] = HDY_b2Bucket;
double b3Bucket[5][1] = HDY_b3Bucket;
double b4Bucket = HDY_b4Bucket;

double InMaxMinArm[4][2] = HDY_InMaxMinArm;
double OutMaxMinArm[1][2] = HDY_OutMaxMinArm;
double IWArm[5][4] = HDY_IWArm;
double Lw1Arm[5][5] = HDY_Lw1Arm;
double Lw2Arm[5][5] = HDY_Lw2Arm;
double Lw3Arm[1][5] = HDY_Lw3Arm;
double b1Arm[5][1] = HDY_b1Arm;
double b2Arm[5][1] = HDY_b2Arm;
double b3Arm[5][1] = HDY_b3Arm;
double b4Arm = HDY_b4Arm;


double InMaxMinBoom[2][2] = HDY_InMaxMinBoom;
double OutMaxMinBoom[1][2] = HDY_OutMaxMinBoom;
double IWBoom[5][2] = HDY_IWBoom;
double Lw1Boom[5][5] = HDY_Lw1Boom;
double Lw2Boom[5][5] = HDY_Lw2Boom;
double Lw3Boom[1][5] = HDY_Lw3Boom;
double b1Boom[5][1] = HDY_b1Boom;
double b2Boom[5][1] = HDY_b2Boom;
double b3Boom[5][1] = HDY_b3Boom;
double b4Boom = HDY_b4Boom;

//The input is the pixel coordinates of the center of mass, and the output is the joint angle (in degrees)
JointAngle FunctionANN(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket)
{
	JointAngle Out;
	Out.Aerfa = ANN_BoomAngle(u_Boom, v_Boom);
	Out.Beita = ANN_ArmAngle(u_Boom, v_Boom, u_Arm, v_Arm);
	Out.Gama = ANN_BucketAngle(u_Boom, v_Boom, u_Arm, v_Arm, u_Bucket, v_Bucket);
	return Out;
}


//The input is the pixel coordinates of the center of mass, and the output is the joint angle (in radians)
double ANN_BucketAngle(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket)
{
	double InBucket[6][1] = { u_Boom, v_Boom, u_Arm, v_Arm, u_Bucket, v_Bucket };

	int i = 6, j = 5, k = 5, t = 5, m = 1;
	double y1[5][1] = { 0, 0, 0, 0, 0 };
	double y2[5][1] = { 0, 0, 0, 0, 0 };
	double y3[5][1] = { 0, 0, 0, 0, 0 };
	double y4 = 0;
	double in[6][1] = { 0, 0, 0, 0, 0, 0 };
	double out;

	for (int u = 0; u < i; u++)//normalization
	{
		in[u][0] = (InBucket[u][0] - InMaxMinBucket[u][1]) * 2 / (InMaxMinBucket[u][0] - InMaxMinBucket[u][1]) - 1;
		
	}

	for (int u = 0; u < j; u++)
	{
		double a = 0;
		for (int v = 0; v < i; v++)
		{
			a = a + IWBucket[u][v] * in[v][0];
		}

		y1[u][0] = 2 / (1 + exp(-2 * (a + b1Bucket[u][0]))) - 1;
		
	}

	for (int u = 0; u < k; u++)
	{
		double a = 0;
		for (int v = 0; v < j; v++)
		{
			a = a + Lw1Bucket[u][v] * y1[v][0];
		}
		y2[u][0] = 2 / (1 + exp(-2 * (a + b2Bucket[u][0]))) - 1;
	}


	for (int u = 0; u < t; u++)
	{
		double a = 0;
		for (int v = 0; v < k; v++)
		{
			a = a + Lw2Bucket[u][v] * y2[v][0];
		}
		y3[u][0] = 2 / (1 + exp(-2 * (a + b3Bucket[u][0]))) - 1;
	}

	double a = 0;
	for (int v = 0; v < t; v++)
	{
		a = a + Lw3Bucket[0][v] * y3[v][0];
	}
	y4 = a + b4Bucket;

	out = OutMaxMinBucket[0][1] + (y4 + 1) / 2 * (OutMaxMinBucket[0][0] - OutMaxMinBucket[0][1]);//Denormalization

	if (out > gamaMax)
	{
		out = gamaMax;
	}
	if (out < gamaMin)
	{
		out = gamaMin;
	}
	return out;
}



double ANN_ArmAngle(double u_Boom, double v_Boom, double u_Arm, double v_Arm)
{
	double InArm[4][1] = { u_Boom, v_Boom, u_Arm, v_Arm };

	int i = 4, j = 5, k = 5, t = 5, m = 1;
	double y1[5][1] = { 0, 0, 0, 0, 0 };
	double y2[5][1] = { 0, 0, 0, 0, 0 };
	double y3[5][1] = { 0, 0, 0, 0, 0 };
	double y4 = 0;
	double in[4][1] = { 0, 0, 0, 0 };
	double out;

	for (int u = 0; u < i; u++)/
	{
		in[u][0] = (InArm[u][0] - InMaxMinArm[u][1]) * 2 / (InMaxMinArm[u][0] - InMaxMinArm[u][1]) - 1;
	}

	for (int u = 0; u < j; u++)
	{
		double a = 0;
		for (int v = 0; v < i; v++)
		{
			a = a + IWArm[u][v] * in[v][0];
		}

		y1[u][0] = 2 / (1 + exp(-2 * (a + b1Arm[u][0]))) - 1;
	}

	for (int u = 0; u < k; u++)
	{
		double a = 0;
		for (int v = 0; v < j; v++)
		{
			a = a + Lw1Arm[u][v] * y1[v][0];
		}
		y2[u][0] = 2 / (1 + exp(-2 * (a + b2Arm[u][0]))) - 1;
	}


	for (int u = 0; u < t; u++)
	{
		double a = 0;
		for (int v = 0; v < k; v++)
		{
			a = a + Lw2Arm[u][v] * y2[v][0];
		}
		y3[u][0] = 2 / (1 + exp(-2 * (a + b3Arm[u][0]))) - 1;
	}

	double a = 0;
	for (int v = 0; v < t; v++)
	{
		a = a + Lw3Arm[0][v] * y3[v][0];
	}
	y4 = a + b4Arm;

	out = OutMaxMinArm[0][1] + (y4 + 1) / 2 * (OutMaxMinArm[0][0] - OutMaxMinArm[0][1]);
	
	if (out > beitaMax)
	{
		out = beitaMax;
	}
	if (out < beitaMin)
	{
		out = beitaMin;
	}
	return out;
}

double ANN_BoomAngle(double u_Boom, double v_Boom)
{
	double InBoom[2][1] = { u_Boom, v_Boom };

	int i = 2, j = 5, k = 5, t = 5, m = 1;
	double y1[5][1] = { 0, 0, 0, 0, 0 };
	double y2[5][1] = { 0, 0, 0, 0, 0 };
	double y3[5][1] = { 0, 0, 0, 0, 0 };
	double y4 = 0;
	double in[2][1] = { 0, 0 };
	double out;

	for (int u = 0; u < i; u++)
	{
		in[u][0] = (InBoom[u][0] - InMaxMinBoom[u][1]) * 2 / (InMaxMinBoom[u][0] - InMaxMinBoom[u][1]) - 1;
	}

	for (int u = 0; u < j; u++)
	{
		double a = 0;
		for (int v = 0; v < i; v++)
		{
			a = a + IWBoom[u][v] * in[v][0];
		}

		y1[u][0] = 2 / (1 + exp(-2 * (a + b1Boom[u][0]))) - 1;
	}

	for (int u = 0; u < k; u++)
	{
		double a = 0;
		for (int v = 0; v < j; v++)
		{
			a = a + Lw1Boom[u][v] * y1[v][0];
		}
		y2[u][0] = 2 / (1 + exp(-2 * (a + b2Boom[u][0]))) - 1;
	}


	for (int u = 0; u < t; u++)
	{
		double a = 0;
		for (int v = 0; v < k; v++)
		{
			a = a + Lw2Boom[u][v] * y2[v][0];
		}
		y3[u][0] = 2 / (1 + exp(-2 * (a + b3Boom[u][0]))) - 1;
	}

	double a = 0;
	for (int v = 0; v < t; v++)
	{
		a = a + Lw3Boom[0][v] * y3[v][0];
	}
	y4 = a + b4Boom;

	out = OutMaxMinBoom[0][1] + (y4 + 1) / 2 * (OutMaxMinBoom[0][0] - OutMaxMinBoom[0][1]);
	
	if (out > alphaMax)
	{
		out = alphaMax;
	}
	if (out < alphaMin)
	{
		out = alphaMin;
	}
	return out;
}