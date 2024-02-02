#include<iostream>
#include<math.h>
#include "ros/ros.h"
#include "ExperimentPkg/Parameter.h"
#include "ExperimentPkg/FPPC2JointAnglePMMMLM.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include<cmath>

using namespace std;
using namespace cv;

//The input is the pixel coordinates of the center of mass, and the output is the joint angle (in degrees)
JointAngle FunctionPMMMLM(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket)
{
	JointAngle out;
	XY xy1, xy2, xy3, xy4, xy5, xy6, xy7, xy8, xy9;
	xy1 = f_uv2xdbarydbarPMMMLM(u_Boom, v_Boom);
	xy2 = f_uv2xdbarydbarPMMMLM(u_Arm, v_Arm);
	xy3 = f_uv2xdbarydbarPMMMLM(u_Bucket, v_Bucket);
	xy4 = f_OnlineDistortPMMMLM(xy1.X, xy1.Y);
	xy5 = f_OnlineDistortPMMMLM(xy2.X, xy2.Y);
	xy6 = f_OnlineDistortPMMMLM(xy3.X, xy3.Y);
	xy7 = f_xibaryibar2XYPMMMLM(xy4.X, xy4.Y, Z1_PMMMLM);
	xy8 = f_xibaryibar2XYPMMMLM(xy5.X, xy5.Y, Z2_PMMMLM);
	xy9 = f_xibaryibar2XYPMMMLM(xy6.X, xy6.Y, Z3_PMMMLM);
	out = f_XY2anglePMMMLM(xy7.X, xy7.Y, xy8.X, xy8.Y, xy9.X, xy9.Y);

	return out;
}

XY f_uv2xdbarydbarPMMMLM(double u, double v)
{
	XY out;

	Mat K = (Mat_<double>(3, 3) << fx_PMMMLM, 0, u0_PMMMLM, 0, fy_PMMMLM, v0_PMMMLM, 0, 0, 1);
	Mat U = (Mat_<double>(3, 1) << u, v, 1);

	Mat invK;
	invert(K, invK);
	Mat Result = invK*U;

	out.X = Result.at<double>(0, 0);
	out.Y = Result.at<double>(1, 0);
	return out;
}


XY f_OnlineDistortPMMMLM(double x, double y)
{
	XY out;
	double k1 = k1_PMMMLM;
	double k2 = k2_PMMMLM;
	double p1 = p1_PMMMLM;
	double p2 = p2_PMMMLM;
	double k3 = k3_PMMMLM;
	double xi = x;
	double yi = y;

	double f1, f2, df1xi, df1dyi, df2dxi, df2yi;
	for (int i = 0; i < 1000; i++)
	{
		f1 = xi + k1*pow(xi, 3) + k1*xi*pow(yi, 2) + k2*pow(xi, 5) + k2*xi*pow(yi, 4) + 2 * k2*pow(xi, 3) * pow(yi, 2) + k3*pow(xi, 7) + k3*pow(xi, 3) * pow(yi, 4) + 2 * k3*pow(xi, 5) * pow(yi, 2) + k3*pow(xi, 5)* pow(yi, 2) + k3*xi*pow(yi, 6) + 2 * k3*pow(xi, 3) * pow(yi, 4) + 2 * p1*xi*yi + p2*(pow(xi, 2) + pow(yi, 2) + 2 * pow(xi , 2)) - x;
		f2 = yi + k1*pow(yi, 3) + k1*yi*pow(xi, 2) + k2*pow(yi, 5) + k2*yi*pow(xi, 4) + 2 * k2*pow(yi, 3) * pow(xi, 2) + k3*pow(yi, 7) + k3*pow(yi, 3) * pow(xi, 4) + 2 * k3*pow(yi, 5) * pow(xi, 2) + k3*pow(yi, 5) * pow(xi, 2) + k3*yi*pow(xi, 6) + 2 * k3*pow(yi, 3) * pow(xi, 4) + 2 * p2*xi*yi + p1*(pow(xi, 2) + pow(yi, 2) + 2 * pow(yi , 2)) - y;

		df1xi = 1 + 3 * k1*pow(xi, 2) + k1*pow(yi, 2) + 5 * k2*pow(xi, 4) + k2*pow(yi, 4) + 6 * k2*pow(xi, 2) * pow(yi, 2) + 7 * k3*pow(xi, 6) + 3 * k3*pow(xi, 2) * pow(yi, 4) + 10 * k3*pow(xi, 4) * pow(yi, 2) + 5 * k3*pow(xi, 4) * pow(yi, 2) + k3*pow(yi, 6) + 6 * k3*pow(xi, 2) * pow(yi,4) + 2 * p1*yi + 2 * p2*xi + 4 * p2*xi;
		df1dyi = 2 * k1*xi*yi + 4 * k2*xi*pow(yi, 3) + 4 * k2*pow(xi, 3) * yi + 4 * k3*pow(xi, 3) * pow(yi, 3) + 4 * k3*pow(xi, 5) * yi + 2 * k3*pow(xi, 5) * yi + 6 * k3*xi*pow(yi, 5) + 8 * k3*pow(xi, 3) * pow(yi,3) + 2 * p1*xi + 2 * p2*yi;

		df2dxi = 2 * k1*xi*yi + 4 * k2*yi*pow(xi, 3) + 4 * k2*pow(yi, 3) * xi + 4 * k3*pow(xi, 3) * pow(yi, 3) + 4 * k3*pow(yi, 5) * xi + 2 * k3*pow(yi, 5) * xi + 6 * k3*yi*pow(xi, 5) + 8 * k3*pow(xi, 3) * pow(yi,3) + 2 * p2*yi + 2 * p1*xi;
		df2yi = 1 + 3 * k1*pow(yi, 2) + k1*pow(xi, 2) + 5 * k2*pow(yi, 4) + k2*pow(xi, 4) + 6 * k2*pow(xi, 2) * pow(yi, 2) + 7 * k3*pow(yi, 6) + 3 * k3*pow(yi, 2) * pow(xi, 4) + 10 * k3*pow(yi, 4) * pow(xi, 2) + 5 * k3*pow(yi, 4) * pow(xi, 2) + k3*pow(xi, 6) + 6 * k3*pow(yi, 2) * pow(xi,4) + 2 * p2*xi + 6 * p1*yi;

		Mat J = (Mat_<double>(2, 2) << df1xi, df1dyi, df2dxi, df2yi);
		Mat f1f2 = (Mat_<double>(2, 1) << f1, f2);
		Mat invJ;
		invert(J, invJ);
		Mat deta = invJ*f1f2;

		xi = xi - deta.at<double>(0, 0);
		yi = yi - deta.at<double>(1, 0);

		out.X = xi;
		out.Y = yi;

		if (abs(deta.at<double>(0, 0)) < pow(10,-10) && abs(deta.at<double>(1, 0)) < pow(10,-10))
		{
			return out;
		}
	}

	return out;
}


XY f_xibaryibar2XYPMMMLM(double x, double y, double Z)
{
	XY out;

	Mat fai = (Mat_<double>(3, 1) << fai1_PMMMLM, fai2_PMMMLM, fai3_PMMMLM);
	Mat rou = (Mat_<double>(3, 1) << rou1_PMMMLM, rou2_PMMMLM, rou3_PMMMLM);


	double theita = norm(fai, cv::NORM_L2);
	Mat a = 1 / theita*fai;

	Mat RA = (Mat_<double>(3, 3) << 0, -a.at<double>(2, 0), a.at<double>(1, 0), a.at<double>(2, 0), 0, -a.at<double>(0, 0), -a.at<double>(1, 0), a.at<double>(0, 0), 0);

	Mat I3 = Mat::eye(3, 3, CV_64F);

	Mat aT;
	transpose(a, aT);

	Mat R = cos(theita)*I3 + (1 - cos(theita))*a*aT + sin(theita)*RA;
	Mat Jsose = sin(theita) / theita*I3 + (1 - sin(theita) / theita)*a*aT + ((1 - cos(theita)) / theita)*RA;
	Mat t = Jsose*rou;

	double t11 = R.at<double>(0, 0);
	double t12 = R.at<double>(0, 1);
	double t13 = R.at<double>(0, 2);
	double t14 = t.at<double>(0, 0);
	double t21 = R.at<double>(1, 0);
	double t22 = R.at<double>(1, 1);
	double t23 = R.at<double>(1, 2);
	double t24 = t.at<double>(1, 0);
	double t31 = R.at<double>(2, 0);
	double t32 = R.at<double>(2, 1);
	double t33 = R.at<double>(2, 2);
	double t34 = t.at<double>(2, 0);

	double aa=t11 - x*t31;
	double bb = t12 - x*t32;
	double cc = x*t33*Z + x*t34 - t13*Z - t14;
	double dd = t21 - y*t31;
	double ee = t22 - y*t32;
	double ff = y*t33*Z + y*t34 - t23*Z - t24;

	Mat abde = (Mat_<double>(2, 2) << aa, bb, dd, ee);
	Mat cf = (Mat_<double>(2, 1) << cc, ff);

	Mat inabde;
	invert(abde, inabde);
	Mat result = inabde*cf;

	out.X = result.at<double>(0, 0);
	out.Y = result.at<double>(1, 0);

	return out;
}

JointAngle f_XY2anglePMMMLM(double x1, double y1, double x2, double y2, double x3, double y3)
{
	JointAngle out;

	out.Aerfa = atan(y1 / x1) - aerfa1_PMMMLM;
	if (out.Aerfa<alphaMin)
	{
		out.Aerfa = alphaMin;
	}
	else if (out.Aerfa > alphaMax)
	{
		out.Aerfa = alphaMax;
	}

	double cosb = (x2 - L1*cos(out.Aerfa)) / lFM2_PMMMLM;
	double atanb = atan((y2 - L1*sin(out.Aerfa)) / (x2 - L1*cos(out.Aerfa)));
	if (cosb == 0)
	{
		out.Beita = -0.5*PI - out.Aerfa - beita1_PMMMLM;
	}
	else if (cosb > 0)
	{
		out.Beita = atanb - out.Aerfa - beita1_PMMMLM;
	}	
	else
	{
		out.Beita = atanb - PI - out.Aerfa - beita1_PMMMLM;
	}
	if (out.Beita < beitaMin)
	{
		out.Beita = beitaMin;
	}	
	else if (out.Beita > beitaMax)
	{
		out.Beita = beitaMax;
	}


	if (x3 - L1*cos(out.Aerfa) - L2*cos(out.Aerfa + out.Beita) == 0 && y3 - L1*sin(out.Aerfa) - L2*sin(out.Aerfa + out.Beita)<0)
	{
		out.Gama = -0.5*PI - out.Aerfa - out.Beita - gama1_PMMMLM;
	}
	else if (x3 - L1*cos(out.Aerfa) - L2*cos(out.Aerfa + out.Beita) == 0 && y3 - L1*sin(out.Aerfa) - L2*sin(out.Aerfa + out.Beita) > 0)
	{
		double gamaA = 0.5*PI - out.Aerfa - out.Beita - gama1_PMMMLM;
		double gamaB = -1.5*PI - out.Aerfa - out.Beita - gama1_PMMMLM;
		if (gamaA >= gamaMin && gamaA <= gamaMax && (gamaB > gamaMax || gamaB<gamaMin))
		{
			out.Gama = gamaA;
		}
		else if (gamaB >= gamaMin && gamaB <= gamaMax && (gamaA>gamaMax || gamaA < gamaMin))
		{
			out.Gama = gamaB;
		}
	}
	else if (x3 - L1*cos(out.Aerfa) - L2*cos(out.Aerfa + out.Beita)!= 0)
	{
		double atanc = atan((y3 - L1*sin(out.Aerfa) -L2*sin(out.Aerfa + out.Beita)) / (x3 - L1*cos(out.Aerfa) - L2*cos(out.Aerfa + out.Beita)));
		double cosc = (x3 - L1*cos(out.Aerfa) - L2*cos(out.Aerfa + out.Beita)) / lQM3_PMMMLM;
		double sinc = (y3 - L1*sin(out.Aerfa) - L2*sin(out.Aerfa + out.Beita)) / lQM3_PMMMLM;

		if (cosc > 0 && sinc < 0)
		{
			out.Gama = atanc - out.Aerfa - out.Beita - gama1_PMMMLM;
		}
		else if (cosc < 0 && sinc < 0)
		{
			out.Gama = atanc - PI - out.Aerfa - out.Beita - gama1_PMMMLM;
		}
		else if (cosc < 0 && sinc>0)
		{
			double gamaA = atanc - PI - out.Aerfa - out.Beita - gama1_PMMMLM;
			double gamaB = atanc + PI - out.Aerfa - out.Beita - gama1_PMMMLM;

			if (gamaA >= gamaMin && gamaA <= gamaMax && (gamaB > gamaMax || gamaB<gamaMin))
			{
				out.Gama = gamaA;
			}
			else if (gamaB >= gamaMin && gamaB <= gamaMax && (gamaA>gamaMax || gamaA < gamaMin))
			{
				out.Gama = gamaB;
			}
		}
		else if (cosc>0 && sinc > 0)
		{
			double gamaA = atanc - out.Aerfa - out.Beita - gama1_PMMMLM;
			double gamaB = atanc - 2 * PI - out.Aerfa - out.Beita - gama1_PMMMLM;
			if (gamaA >= gamaMin && gamaA <= gamaMax && (gamaB > gamaMax || gamaB<gamaMin))
			{
				out.Gama = gamaA;
			}
			else if (gamaB >= gamaMin && gamaB <= gamaMax && (gamaA>gamaMax || gamaA < gamaMin))
			{
				out.Gama = gamaB;
			}
		}
	}

	return out;
}