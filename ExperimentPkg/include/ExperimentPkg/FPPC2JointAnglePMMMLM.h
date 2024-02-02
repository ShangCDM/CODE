#ifndef FPPC2JOINTANGLEPMMMLM_H_
#define FPPC2JOINTANGLEPMMMLM_H_

#include "Parameter.h"

//FPPC to joint angle(PMMM+LM method)
XY f_uv2xdbarydbarPMMMLM(double u, double v);
XY f_OnlineDistortPMMMLM(double x, double y);
XY f_xibaryibar2XYPMMMLM(double x, double y, double Z);
JointAngle f_XY2anglePMMMLM(double x1, double y1, double x2, double y2, double x3, double y3);

JointAngle FunctionPMMMLM(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket);

#endif
