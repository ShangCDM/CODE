#ifndef FPPC2JOINTANGLETHISWORK_H_
#define FPPC2JOINTANGLETHISWORK_H_

#include "Parameter.h"

//FPPC to joint angle (this work method)

//system model
XY f_uv2xdbarydbarThisWork(double u, double v);//Input is FPPC,output is normalized coordinate
XY f_OnlineDistortThisWork(double x, double y);//Input is ideal normalized coordinate ,output is distorted normalized coordinate
XY f_xibaryibar2XYThisWork(double x, double y, double Z);//Input is ideal normalized coordinate ,output is word coordinate
JointAngle f_XY2angleThisWork(double x1, double y1, double x2, double y2, double x3, double y3);//Input is word coordinate ,output are joint angles

JointAngle FunctionThisWork(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket);

#endif
