#pragma once
#include <assert.h>  
#include "math.h"
#include <stdio.h>
#include <process.h> 
#include <string.h>
#include <windows.h>
#include <stdlib.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs/legacy/constants_c.h"
#include <opencv2/highgui/highgui_c.h>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/types_c.h" 
#include <opencv2/opencv.hpp>
#include <conio.h>
#include "MvCameraControl.h"
#include "calibration.h"
using namespace cv;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
	if (NULL == pstMVDevInfo)
	{
		printf("%s\n", "The Pointer of pstMVDevInfoList is NULL!");
		return false;
	}
	if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
	{
		printf("%s %x\n", "nCurrentIp:", pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp);                   //当前IP
		printf("%s %s\n\n", "chUserDefinedName:", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);     //用户定义名
	}
	else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
	{
		printf("UserDefinedName:%s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
	}
	else
	{
		printf("Not support.\n");
	}
	return true;
}

void calib_grab_image(string &save_path)
{
	int nRet = MV_OK;

	void* handle = NULL;

	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

	// 1. 枚举设备
	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
	}
	unsigned int nIndex = 0;
	if (stDeviceList.nDeviceNum > 0)
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				break;
			}
			PrintDeviceInfo(pDeviceInfo);
		}
	}
	else
	{
		printf("Find No GIGE Devices!\n");
	}

	// 2. 选择设备并创建句柄
	scanf_s("%d", &nIndex);
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
	}

	// 3.打开设备
	nRet = MV_CC_OpenDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
	}

	// 4. 设置参数
	// 关闭触发
	nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
	if (MV_OK != nRet)
	{
		printf("Set TriggerMode failed[%x]!\n", nRet);
	}

	// 使用连续模式
	nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
	if (MV_OK != nRet)
	{
		printf("Set AcquisitionMode failed[%x]!\n", nRet);
	}

	// 5.开始抓图
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
	}

	//nRet = MV_CC_Display(handle, MainWndID);
	//if (MV_OK != nRet)
	//{
	//	printf("MV_CC_Display fail! nRet [%x]\n", nRet);
	//}

	// 6.获取10张图,保存
	MVCC_INTVALUE stIntvalue = { 0 };
	nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stIntvalue);
	if (nRet != MV_OK)
	{
		printf("Get PayloadSize failed! nRet [%x]\n", nRet);
	}

	int nBufSizeForDriver = stIntvalue.nCurValue + 2048;  // 一帧数据大小 + 预留2048byte用于SDK内部处理
	unsigned char * pBufForDriver = (unsigned char *)malloc(nBufSizeForDriver);

	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	unsigned int nImageNum = 10;

	// BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
	unsigned int nBufSizeForSaveImage = 0;
	unsigned char* pBufForSaveImage = NULL;
	int c = 0;
	namedWindow("视频流", WINDOW_NORMAL);
	while (nImageNum)
	{
		nRet = MV_CC_GetOneFrameTimeout(handle, pBufForDriver, nBufSizeForDriver, &stImageInfo, 1000);
		Mat videocapture(Size(stImageInfo.nWidth, stImageInfo.nHeight), CV_8UC1, pBufForDriver);
		imshow("视频流", videocapture);
		c = waitKey(50);
		if (c == 13)
		{
			/*
			if (nRet == MV_OK)
			{
				printf("Width[%d],Height[%d],FrameNum[%d]\r\n", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
				nImageNum--;

				// 仅在第一次保存图像时申请缓存
				if (NULL == pBufForSaveImage)
				{
					// BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
					nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;

					pBufForSaveImage = (unsigned char*)malloc(nBufSizeForSaveImage);
					if (NULL == pBufForSaveImage)
					{
					}
				}

				//设置对应的相机参数
				MV_SAVE_IMAGE_PARAM_EX stParam;
				memset(&stParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
				stParam.enImageType = MV_Image_Bmp; //需要保存的图像类型
				stParam.enPixelType = stImageInfo.enPixelType;  //相机对应的像素格式
				stParam.nBufferSize = nBufSizeForSaveImage;  //存储节点的大小
				stParam.nWidth = stImageInfo.nWidth;         //相机对应的宽
				stParam.nHeight = stImageInfo.nHeight;          //相机对应的高
				stParam.nDataLen = stImageInfo.nFrameLen;
				stParam.pData = pBufForDriver;
				stParam.pImageBuffer = pBufForSaveImage;
				stParam.nJpgQuality = 80;

				nRet = MV_CC_SaveImageEx(&stParam);
				if (MV_OK != nRet)
				{
					printf("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
				}
				string string_path = save_path;
				char *char_path = (char *)string_path.c_str();
				char Name[32] = {0};
				sprintf_s(Name, 32, "%d.bmp", (10 - nImageNum));
				char pImageName[100];
				sprintf_s(pImageName, "%s%s", char_path, Name);
				//FILE** fp = nullptr;
				//cout << pImageName << endl;
				//fopen_s(fp, pImageName, "wb");
				//fwrite(pBufForSaveImage, 1, stParam.nImageLen, *fp);
				//fclose(*fp);
				FILE* fp = fopen(pImageName, "wb");
				fwrite(pBufForSaveImage, 1, stParam.nImageLen, fp);
				fclose(fp);
			}
			else
			{
				nImageNum--;
				printf("No Data!\n");
			}
			*/
			nImageNum--;
			string string_path = save_path;
			char *char_path = (char *)string_path.c_str();
			char Name[32] = { 0 };
			sprintf_s(Name, 32, "%d.bmp", (10 - nImageNum));
			char pImageName[100];
			sprintf_s(pImageName, "%s%s", char_path, Name);
			cout << pImageName << endl;
			imwrite(pImageName, videocapture);
		}
	}
	destroyWindow("视频流");
	// 释放内存
	if (pBufForDriver)
	{
		free(pBufForDriver);
		pBufForDriver = NULL;
	}
	if (pBufForSaveImage)
	{
		free(pBufForSaveImage);
		pBufForSaveImage = NULL;
	}

	// 7.结束抓图
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
	}

	// 8.关闭设备
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
	}

	// 9.销毁句柄
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
	}
}


void img_grab_image(string &save_path)
{
	int nRet = MV_OK;

	void* handle = NULL;

	MV_CC_DEVICE_INFO_LIST stDeviceList;
	memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

	// 1. 枚举设备
	nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
	if (MV_OK != nRet)
	{
		printf("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
	}
	unsigned int nIndex = 0;
	if (stDeviceList.nDeviceNum > 0)
	{
		for (int i = 0; i < stDeviceList.nDeviceNum; i++)
		{
			printf("[device %d]:\n", i);
			MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
			if (NULL == pDeviceInfo)
			{
				break;
			}
			PrintDeviceInfo(pDeviceInfo);
		}
	}
	else
	{
		printf("Find No GIGE Devices!\n");
	}

	// 2. 选择设备并创建句柄
	scanf_s("%d", &nIndex);
	nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
	}

	// 3.打开设备
	nRet = MV_CC_OpenDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
	}

	// 4. 设置参数
	// 关闭触发
	nRet = MV_CC_SetEnumValue(handle, "TriggerMode", MV_TRIGGER_MODE_OFF);
	if (MV_OK != nRet)
	{
		printf("Set TriggerMode failed[%x]!\n", nRet);
	}

	// 使用连续模式
	nRet = MV_CC_SetEnumValue(handle, "AcquisitionMode", MV_ACQ_MODE_CONTINUOUS);
	if (MV_OK != nRet)
	{
		printf("Set AcquisitionMode failed[%x]!\n", nRet);
	}

	// 5.开始抓图
	nRet = MV_CC_StartGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
	}

	//nRet = MV_CC_Display(handle, MainWndID);
	//if (MV_OK != nRet)
	//{
	//	printf("MV_CC_Display fail! nRet [%x]\n", nRet);
	//}

	// 6.获取10张图,保存
	MVCC_INTVALUE stIntvalue = { 0 };
	nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stIntvalue);
	if (nRet != MV_OK)
	{
		printf("Get PayloadSize failed! nRet [%x]\n", nRet);
	}

	int nBufSizeForDriver = stIntvalue.nCurValue + 2048;  // 一帧数据大小 + 预留2048byte用于SDK内部处理
	unsigned char * pBufForDriver = (unsigned char *)malloc(nBufSizeForDriver);

	MV_FRAME_OUT_INFO_EX stImageInfo = { 0 };
	memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
	unsigned int nImageNum = 10;

	// BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
	unsigned int nBufSizeForSaveImage = 0;
	unsigned char* pBufForSaveImage = NULL;
	int c = -1;
	Mat final_videocapture;
	namedWindow("视频流", WINDOW_NORMAL);
	while (c == -1) {
		nRet = MV_CC_GetOneFrameTimeout(handle, pBufForDriver, nBufSizeForDriver, &stImageInfo, 1000);
		Mat videocapture(Size(stImageInfo.nWidth, stImageInfo.nHeight), CV_8UC1, pBufForDriver);
		final_videocapture = videocapture;
		imshow("视频流", videocapture);
		c = waitKey(10);
	}
	destroyWindow("视频流");
	if (c == 13){
		while (nImageNum)
		{
			/*
			if (nRet == MV_OK)
			{
				printf("Width[%d],Height[%d],FrameNum[%d]\r\n", stImageInfo.nWidth, stImageInfo.nHeight, stImageInfo.nFrameNum);
				nImageNum--;

				// 仅在第一次保存图像时申请缓存
				if (NULL == pBufForSaveImage)
				{
					// BMP图片大小：width * height * 3 + 2048(预留BMP头大小)
					nBufSizeForSaveImage = stImageInfo.nWidth * stImageInfo.nHeight * 3 + 2048;

					pBufForSaveImage = (unsigned char*)malloc(nBufSizeForSaveImage);
					if (NULL == pBufForSaveImage)
					{
					}
				}

				//设置对应的相机参数
				MV_SAVE_IMAGE_PARAM_EX stParam;
				memset(&stParam, 0, sizeof(MV_SAVE_IMAGE_PARAM_EX));
				stParam.enImageType = MV_Image_Bmp; //需要保存的图像类型
				stParam.enPixelType = stImageInfo.enPixelType;  //相机对应的像素格式
				stParam.nBufferSize = nBufSizeForSaveImage;  //存储节点的大小
				stParam.nWidth = stImageInfo.nWidth;         //相机对应的宽
				stParam.nHeight = stImageInfo.nHeight;          //相机对应的高
				stParam.nDataLen = stImageInfo.nFrameLen;
				stParam.pData = pBufForDriver;
				stParam.pImageBuffer = pBufForSaveImage;
				stParam.nJpgQuality = 80;

				nRet = MV_CC_SaveImageEx(&stParam);
				if (MV_OK != nRet)
				{
					printf("failed in MV_CC_SaveImage,nRet[%x]\n", nRet);
				}
				string string_path = save_path;
				char *char_path = (char *)string_path.c_str();
				char Name[32] = { 0 };
				sprintf_s(Name, 32, "%d.bmp", stImageInfo.nFrameNum);
				char pImageName[100];
				sprintf_s(pImageName, "%s%s", char_path, Name);
				FILE** fp = nullptr;
				fopen_s(fp, pImageName, "wb");
				fwrite(pBufForSaveImage, 1, stParam.nImageLen, *fp);
				fclose(*fp);
			}
			else
			{
				nImageNum--;
				printf("No Data!\n");
			}
			*/
			nImageNum--;
			string string_path = save_path;
			char *char_path = (char *)string_path.c_str();
			char Name[32] = { 0 };
			sprintf_s(Name, 32, "%d.bmp", (10 - nImageNum));
			char pImageName[100];
			sprintf_s(pImageName, "%s%s", char_path, Name);
			cout << pImageName << endl;
			imwrite(pImageName, final_videocapture);
		}
	}

	// 释放内存
	if (pBufForDriver)
	{
		free(pBufForDriver);
		pBufForDriver = NULL;
	}
	if (pBufForSaveImage)
	{
		free(pBufForSaveImage);
		pBufForSaveImage = NULL;
	}

	// 7.结束抓图
	nRet = MV_CC_StopGrabbing(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
	}

	// 8.关闭设备
	nRet = MV_CC_CloseDevice(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_CloseDevice fail! nRet [%x]\n", nRet);
	}

	// 9.销毁句柄
	nRet = MV_CC_DestroyHandle(handle);
	if (MV_OK != nRet)
	{
		printf("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
	}
}