#ifndef CAMERAPREPARE_H_
#define CAMERAPREPARE_H_

#include "/opt/MVS/include/MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream> 
#include <stdlib.h> 
#include <ctime>
#include "string.h"

using namespace cv;
using namespace std;

//Start the camera and capture images related to it
struct Image
{
	Mat Frame;//Image matrix
	unsigned char * pData;
};

struct Came
{
	void * handle;
	unsigned int g_nPayloadSize;
};

Came CameraPrepare();//Preparation process before using the camera
void CameraClose(void *handle);//Closing process of camera shutdown
Image GetImage(void *handle, unsigned int g_nPayloadSize);//Collect camera images, input is the handle of the camera, output is the structure
Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char * pData);
int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight);

#endif
