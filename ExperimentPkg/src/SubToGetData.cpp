#include <fcntl.h>  
#include <time.h>  
#include <unistd.h>  
#include <termios.h> 
#include <stdio.h>
#include <stdlib.h> 
#include <string.h>
#include <fstream>
#include <iostream>
#include <errno.h> 
#include <sys/types.h> 
#include <sys/stat.h> 
#include <poll.h> 
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "ExperimentPkg/SensorJointAngleMsg.h"
#include "ExperimentPkg/VisualJointAngleMsg.h"
#include "ExperimentPkg/Parameter.h"

//Function:Simultaneously subscribe to visual measurement data and truth values

using namespace message_filters;
using namespace std;

double JointAngleBySensorBoom;
double JointAngleBySensorArm;
double JointAngleBySensorBucket;
double JointAngleByVisualBoom_ANN;
double JointAngleByVisualArm_ANN;
double JointAngleByVisualBucket_ANN;
double JointAngleByVisualBoom_PMMMLM;
double JointAngleByVisualArm_PMMMLM;
double JointAngleByVisualBucket_PMMMLM;
double JointAngleByVisualBoom_ThisWork;
double JointAngleByVisualArm_ThisWork;
double JointAngleByVisualBucket_ThisWork;

ofstream fdata("Data.txt");//save data

//callback：save data
void Callback(const ExperimentPkg::VisualJointAngleMsgConstPtr &VisualAngleMsg, const ExperimentPkg::SensorJointAngleMsgConstPtr &SensorAngleMsg)
{ 
  JointAngleBySensorBoom=SensorAngleMsg->SensorAngleBoom;
  JointAngleBySensorArm=SensorAngleMsg->SensorAngleArm;
  JointAngleBySensorBucket=SensorAngleMsg->SensorAngleBucket;
  
  JointAngleByVisualBoom_ANN=VisualAngleMsg->VisualAerfa_ANN;
  JointAngleByVisualArm_ANN=VisualAngleMsg->VisualBeita_ANN;
  JointAngleByVisualBucket_ANN=VisualAngleMsg->VisualGama_ANN;
  JointAngleByVisualBoom_PMMMLM=VisualAngleMsg->VisualAerfa_PMMMLM;
  JointAngleByVisualArm_PMMMLM=VisualAngleMsg->VisualBeita_PMMMLM;
  JointAngleByVisualBucket_PMMMLM=VisualAngleMsg->VisualGama_PMMMLM;
  JointAngleByVisualBoom_ThisWork=VisualAngleMsg->VisualAerfa_ThisWork;
  JointAngleByVisualArm_ThisWork=VisualAngleMsg->VisualBeita_ThisWork;
  JointAngleByVisualBucket_ThisWork=VisualAngleMsg->VisualGama_ThisWork;  
  
  fdata<<JointAngleBySensorBoom<<" "<<JointAngleBySensorArm<<" "<<JointAngleBySensorBucket<<" "<<JointAngleByVisualBoom_ANN<<" "<<JointAngleByVisualArm_ANN<<" "<<JointAngleByVisualBucket_ANN<<" "<<JointAngleByVisualBoom_PMMMLM<<" "<<JointAngleByVisualArm_PMMMLM<<" "<<JointAngleByVisualBucket_PMMMLM<<" "<<JointAngleByVisualBoom_ThisWork<<" "<<JointAngleByVisualArm_ThisWork<<" "<<JointAngleByVisualBucket_ThisWork<<endl;
}

int main(int argc, char** argv)
{
  setlocale(LC_ALL,"");  
  ros::init(argc, argv, "GetDataNode");
  ros::NodeHandle n;
  // Need to use message_ The filter container initializes data publishing for two topics
  message_filters::Subscriber<ExperimentPkg::VisualJointAngleMsg> SubVisualAngleData(n,"PubVisualJointAngleTopic",20);
  message_filters::Subscriber<ExperimentPkg::SensorJointAngleMsg> SubSensorAngleData(n,"PubSensorJointAngleTopic",20);

  //Data synchronization: timestamp against it
  typedef sync_policies::ApproximateTime<ExperimentPkg::VisualJointAngleMsg, ExperimentPkg::SensorJointAngleMsg> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(20),SubVisualAngleData, SubSensorAngleData); 
  sync.registerCallback(boost::bind(&Callback, _1, _2));

  ros::spin();
  return 0;
}

