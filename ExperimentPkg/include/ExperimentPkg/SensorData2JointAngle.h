#ifndef SENSORDATA2JOINTANGLE_H_
#define SENSORDATA2JOINTANGLE_H_

//Sensor data to joint angle function
#include "Parameter.h"
#include "math.h"


//Input:Boom, arm and bucket cylinder displacement sensor data
//Output:Boom, arm and bucket joint angle
JointAngle FunctionSensorData2JointAngle(int DataBoom,int DataArm,int DataBucket);


#endif
