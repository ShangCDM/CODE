#include "/opt/MVS/include/MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream> 
#include <stdlib.h> 
#include <ctime>
#include "string.h"
#include "ros/ros.h"
#include "ExperimentPkg/CameraPrepare.h"
//#include "ExperimentPkg/ImageProcess.h"//Imageprocess
#include "ExperimentPkg/VisualJointAngleMsg.h"
#include "ExperimentPkg/FPPC2JointAngleThisWork.h"
#include "ExperimentPkg/FPPC2JointAnglePMMMLM.h"
#include "ExperimentPkg/FPPC2JointAngleANN.h"
#include "ExperimentPkg/Parameter.h"

using namespace cv;
using namespace std;

//Function:ROS publish visual measurement

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
	ros::init(argc, argv, "PubVisualJointAngleNode");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<ExperimentPkg::VisualJointAngleMsg>("PubVisualJointAngleTopic", 10);
	ExperimentPkg::VisualJointAngleMsg msg;


	while (ros::ok())
	{

		/*
		Omit image processing, input as real-time image, output as FPPC, i.e. (u1, v1) (u2, v2) (u3, v3)
		*/

	    ros::Time current_time = ros::Time::now();
	    msg.header.stamp = current_time;
	    
	    JointAngle out=FunctionPMMMLM(u1,v1,u2,v2,u3,v3);    
	    msg.VisualAerfa_PMMMLM = out.Aerfa;
	    msg.VisualBeita_PMMMLM = out.Beita;
	    msg.VisualGama_PMMMLM = out.Gama;
	    
		out = FunctionThisWork(u1, v1, u2, v2, u3, v3);
	    msg.VisualAerfa_ThisWork = out.Aerfa;
	    msg.VisualBeita_ThisWork = out.Beita;
	    msg.VisualGama_ThisWork = out.Gama;    

		out = FunctionANN(u1, v1, u2, v2, u3, v3);
	    msg.VisualAerfa_ANN = out.Aerfa;
	    msg.VisualBeita_ANN = out.Beita;
	    msg.VisualGama_ANN = out.Gama;    
	    
	    pub.publish(msg);//publish message
	}
	return 1;
}


