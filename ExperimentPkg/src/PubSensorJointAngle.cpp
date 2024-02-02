#include<fcntl.h>  
#include<unistd.h>  
#include<termios.h> 
#include <iostream>
#include <fstream> 
#include<stdio.h>
#include<stdlib.h> 
#include<string.h>
#include<errno.h> 
#include<sys/types.h> 
#include<sys/stat.h> 
#include<poll.h> 
#include "ros/ros.h"
#include "ExperimentPkg/UsartTx2Basic.h"
#include "ExperimentPkg/SensorData2JointAngle.h"
#include "ExperimentPkg/SensorJointAngleMsg.h"
#include "ExperimentPkg/Parameter.h"

using namespace std;

//Function:ROS publish truth joint angle

int main(int argc , char **argv) 
{
  setlocale(LC_ALL,"");
  
  
    int encoder2,encoder3,encoder4;
    int fd;
    char receive_buf[100];
    int length=100;
    fd_set rd;
    fd = openUart();              
    uartInit(fd,115200, 8, 'n' , 1) ; 
    
    ros::init(argc,argv,"PuSensorJointAngleNode");
    ros::NodeHandle nh;
    ros::Publisher pub1=nh.advertise<ExperimentPkg::SensorJointAngleMsg>("PubSensorJointAngleTopic",10);
    ExperimentPkg::SensorJointAngleMsg msg1;
    

       
    while(ros::ok())
    {
      while(uartRead(fd,receive_buf,length)>0)
      {
        
        if(receive_buf[0]=='S' && receive_buf[7]=='F')
        {
	 if((receive_buf[1] & 0x80) ==0x80)
	 {
	   encoder2=-(int)((((receive_buf[1] & 0x7F)<<8)+receive_buf[2]));
	 }
	 else
	 {
	   encoder2=(int)(((receive_buf[1] & 0x7F)<<8)+receive_buf[2]) ;
	 }
	 if((receive_buf[3] & 0x80) ==0x80)
	 {
	   encoder3=-(int)(((receive_buf[3] & 0x7F)<<8)+receive_buf[4]) ;
	 }
	 else
	 {
	   encoder3=(int)(((receive_buf[3] & 0x7F)<<8)+receive_buf[4]) ;
	 }
	 if((receive_buf[5] & 0x80) ==0x80)
	 {
	   encoder4=-(int)(((receive_buf[5] & 0x7F)<<8)+receive_buf[6]) ;
	 }
	 else
	 {
	   encoder4=(int)(((receive_buf[5] & 0x7F)<<8)+receive_buf[6]) ;
	 }
	 printf("%d %d %d \n",encoder2,encoder3,encoder4);
	 
	 JointAngle out;
	 out=FunctionSensorData2JointAngle(encoder2,encoder3,encoder4);
	 msg1.header.stamp=ros::Time::now();
	 msg1.SensorAngleBoom=out.Aerfa;
	 msg1.SensorAngleArm=out.Beita;
	 msg1.SensorAngleBucket=out.Gama;
	 	 
	 pub1.publish(msg1);	 
        }
        else
        {
	 printf("error \n");
        }
        
      }
       
    }
    close(fd);
    return 0;
}