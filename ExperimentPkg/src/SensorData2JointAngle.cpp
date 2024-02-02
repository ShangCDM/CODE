#include<iostream>
#include<math.h>
#include "ExperimentPkg/SensorData2JointAngle.h"
#include "ExperimentPkg/Parameter.h"

//Function:Cylinder displacement sensor data joint angle
JointAngle FunctionSensorData2JointAngle(int DataBoom, int DataArm, int DataBucket)
{
	JointAngle Out;

	//Pulse conversion into displacement
	double Boom_Disp = DataBoom / Nboom+PositionBoom;
	double Arm_Disp = DataArm / Narm;
	double Bucket_Disp = DataBucket / Nbucket;

	//Calculate the joint angle of the boom
	double AB = 1311 + Boom_Disp;
	Out.Aerfa = (double)acos((AC*AC+BC*BC-AB*AB)/2/AC/BC)*180/PI-BCF-ACX;

	//arm
	double DE = 1300 + Arm_Disp;
	Out.Beita = 180 - (double)acos((EF*EF + DF*DF - DE*DE) / 2 / EF / DF) * 180 / PI - DFC - EFQ;

	//bucket
	double GM = 1120 + Bucket_Disp;
	double GNM = (double)acos((GN*GN+MN*MN-GM*GM)/2/GN/MN)*180/PI;
	double QNM = 360 - GNM - GNF - FNQ;
	double MQ = (double)sqrt(NQ*NQ+MN*MN-2*NQ*MN*(double)cos(QNM*PI/180));
	double NQM = (double)acos((NQ*NQ+ MQ*MQ-MN*MN)/2/NQ/MQ)*180/PI;
	double MQK = (double)acos((MQ*MQ + KQ*KQ - MK*MK) / 2 / MQ / KQ) * 180 / PI;
	Out.Gama = 180 - FQN - NQM - MQK - KQV;

	return Out;
}