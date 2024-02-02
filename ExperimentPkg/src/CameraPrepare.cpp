#include "/opt/MVS/include/MvCameraControl.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/core/core.hpp>
#include <iostream>
#include <fstream> 
#include <stdlib.h> 
#include <ctime>
#include "string.h"
#include "ExperimentPkg/CameraPrepare.h"
using namespace std;
using namespace cv;

//Preparation process before camera operation
Came CameraPrepare()
{
	Came C;
	int nRet = MV_OK;
	C.handle = NULL;

	//Scan USB devices
	MV_CC_DEVICE_INFO_LIST stDeviceList;//It is a structure, and the structural elements are the number of devices and the information of each device (represented by pointers)
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));//Opened memory initialization
	nRet = MV_CC_EnumDevices(MV_USB_DEVICE, &stDeviceList);//Get camera devices
	if (MV_OK != nRet)
	{
		cout << "Enum Devices fail! nRet [0x" << hex << nRet << "]" << endl;//
	}

	if (stDeviceList.nDeviceNum <= 0)
	{
		cout << "Find No Devices!" << endl;
	}

	// Create device handle
	nRet = MV_CC_CreateHandle(&C.handle, stDeviceList.pDeviceInfo[0]);
	if (MV_OK != nRet)
	{
		cout << "Create Handle fail! nRet [0x" << hex << nRet << "]" << endl;
	}

	//Create device handle
	nRet = MV_CC_OpenDevice(C.handle);
	if (MV_OK != nRet)
	{
		cout << "Open Device fail! nRet [0x" << hex << nRet << "]" << endl;
	}


	//Camera parameter acquisition.
	MVCC_INTVALUE stParam;
	memset(&stParam, 0, sizeof(MVCC_INTVALUE));
	nRet = MV_CC_GetIntValue(C.handle, "PayloadSize", &stParam);
	if (MV_OK != nRet)
	{
		cout << "Get PayloadSize fail! nRet [0x" << hex << nRet << "]" << endl;

	}
	C.g_nPayloadSize = stParam.nCurValue;//Resolution refers to the total number of pixels

	//Starting flow collection
	nRet = MV_CC_StartGrabbing(C.handle);
	if (MV_OK != nRet)
	{
		cout << "Start Grabbing fail! nRet [0x" << hex << nRet << "]" << endl;
	}
	return C;
}


//Camera shutdown function, input is the handle of the camera
void CameraClose(void *handle)
{
	int nRet;
	//Stop flow collection
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		cout << "Stop Grabbing fail! nRet [0x" << hex << nRet << "]" << endl;

	}

	//close device
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		cout << "ClosDevice fail! nRet [0x" << hex << nRet << "]" << endl;

	}

	//Destroy camera handle (destroy memory)
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		cout << "Destroy Handle fail! nRet [0x" << hex << nRet << "]" << endl;

	}

	//If the handle is not destroyed, destroy it again
	if (nRet != MV_OK)
	{
		if (handle != NULL)
		{
			MV_CC_DestroyHandle(handle);
			handle = NULL;
		}
	}
}

//The function that retrieves images from the camera, with input as the camera handle and output as the structure of the image
Image GetImage(void *handle, unsigned int g_nPayloadSize)
{
	Image I;
	int nRet;
	//Allocate memory
	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));

	unsigned char *pData = (unsigned char *)malloc(sizeof(unsigned char) * (g_nPayloadSize));
	if (pData == NULL)
	{
		cout << "Allocate memory failed" << endl;
	}

	//Obtain image information,
	nRet = MV_CC_GetOneFrameTimeout(handle, pData, g_nPayloadSize, &stImageInfo, 1000);
	if (nRet == MV_OK)
	{
	}
	else
	{
		cout << "No data! nRet [0x" << hex << nRet << "]" << endl;
		free(pData);
		pData = NULL;
	}

	I.Frame = Convert2Mat(&stImageInfo, pData);
	I.pData = pData;

	return I;
}

//Convert image information into matrix representation
Mat Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char * pData)
{
	cv::Mat srcImage;

	if (pstImageInfo->enPixelType == PixelType_Gvsp_RGB8_Packed)
	{
		RGB2BGR(pData, pstImageInfo->nWidth, pstImageInfo->nHeight);
		srcImage = cv::Mat(pstImageInfo->nHeight, pstImageInfo->nWidth, CV_8UC3, pData);
		return srcImage;
	}
	else
	{
		cout << "unsupported pixel format" << endl;
	}

	if (NULL == srcImage.data)
	{
	}

	//save converted image in a local file
	try {
#if defined (VC9_COMPILE)
		cvSaveImage("MatImage.bmp", &(IplImage(srcImage)));
#else
		cv::imwrite("MatImage.bmp", srcImage);
#endif
	}
	catch (cv::Exception& ex) {
		fprintf(stderr, "Exception saving image to bmp format: %s\n", ex.what());
	}

	srcImage.release();
}

//Convert BGR to RGB
int RGB2BGR(unsigned char* pRgbData, unsigned int nWidth, unsigned int nHeight)
{
	if (NULL == pRgbData)
	{
		return MV_E_PARAMETER;
	}

	for (unsigned int j = 0; j < nHeight; j++)
	{
		for (unsigned int i = 0; i < nWidth; i++)
		{
			unsigned char red = pRgbData[j * (nWidth * 3) + i * 3];
			pRgbData[j * (nWidth * 3) + i * 3] = pRgbData[j * (nWidth * 3) + i * 3 + 2];
			pRgbData[j * (nWidth * 3) + i * 3 + 2] = red;
		}
	}

	return MV_OK;
}
