#ifndef FPPC2JOINTANGLEANN_H_
#define FPPC2JOINTANGLEANN_H_


//FPPC to joint angle (ANN method)

//Input is FPPC
double ANN_BucketAngle(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket);
double ANN_ArmAngle(double u_Boom, double v_Boom, double u_Arm, double v_Arm);
double ANN_BoomAngle(double u_Boom, double v_Boom);


JointAngle FunctionANN(double u_Boom, double v_Boom, double u_Arm, double v_Arm, double u_Bucket, double v_Bucket);

#endif
