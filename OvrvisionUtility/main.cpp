#include <iostream>
#include <direct.h>
#include <string>
#include <opencv2\opencv.hpp>
#include <sstream>
#include "..\share\StereoCameraCalibration.h"

#include "ovrvisionsdk_windows\include\ovrvision.h"		//Ovrvision SDK
#if _DEBUG
#pragma comment(lib,"ovrvisionsdk_windows/bin/x86/ovrvisiond.lib")
#else
#pragma comment(lib,"ovrvisionsdk_windows/bin/x86/ovrvision.lib")
#endif
//Ovrvision System
#define OVRVISION 1

#ifdef OVRVISION
OVR::Ovrvision* g_pOvrvision;

#else
cv::VideoCapture cam1;
cv::VideoCapture cam2;

#endif

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define CHESS_SIZE cv::Size(7,4)

std::string FileNameFormat(const char* pre,const int no) {
	std::stringstream ss;
	ss<<pre<<no<<".png";
	return ss.str();
}


using namespace std;
const string leftwindowName="left_image";
const string rightwindowName="right_image";
const string default_left="left_default";
const string default_right="right_default";




unsigned char* g_u1 = NULL;
int main (int argc, char **argv){
	_mkdir("calibration_image");
	_mkdir("capture");
	// Create ovrvision object
#ifdef OVRVISION
	g_pOvrvision = new OVR::Ovrvision();

	g_pOvrvision->Open(0,OVR::OV_CAMVGA_FULL);
#else
	cam1.open(0);
	cam2.open(1);
#endif


	cv::Mat leftImg(IMAGE_HEIGHT, IMAGE_WIDTH,CV_8UC3);
	cv::Mat rightImg(IMAGE_HEIGHT, IMAGE_WIDTH,CV_8UC3);

	cv::Mat homographyLeft=cv::Mat::eye(3, 3, CV_64FC1);
	cv::Mat homographyRight=cv::Mat::eye(3, 3, CV_64FC1);
	cv::namedWindow(leftwindowName );
	cv::namedWindow(rightwindowName );
	
	cv::Mat undisortRight(IMAGE_HEIGHT, IMAGE_WIDTH,CV_8UC3);


	cv::Mat rightImg_chess(IMAGE_HEIGHT, IMAGE_WIDTH,CV_8UC1);

	StereoCameraCalibration calib(leftImg.size(),CHESS_SIZE,30);

	cv::Mat renderleft;
	cv::Mat renderright;
	bool Solved=false;
	int key=0;
	while(true){
		int key=cv::waitKey(3);
#ifdef OVRVISION
		// Get ovrvision image
		g_pOvrvision->PreStoreCamData();
		g_pOvrvision->GetCamImage(leftImg.data, OVR::OV_CAMEYE_LEFT);
		g_pOvrvision->GetCamImage(rightImg.data, OVR::OV_CAMEYE_RIGHT);
#else
		cam1>> leftImg;
		cam2>> rightImg;
#endif
		cv::cvtColor(leftImg,  leftImg,  CV_RGB2BGR);
		cv::cvtColor(rightImg, rightImg, CV_RGB2BGR);
	
	
	if(key =='c')
	{
		bool is = calib.findChess(leftImg,rightImg);

		if(is)
		{
			cv::imwrite(FileNameFormat("calibration_image\\left",calib.getImageCount()),leftImg);
			cv::imwrite(FileNameFormat("calibration_image\\right",calib.getImageCount()),rightImg);

		}
		calib.drawChessboardCorners(&leftImg,&leftImg,StereoCameraCalibration::STEREO_CALIB_LEFT);
		calib.drawChessboardCorners(&rightImg,&rightImg,StereoCameraCalibration::STEREO_CALIB_RIGHT);
		if(is)
		{
			cv::imwrite(FileNameFormat("calibration_image\\left_chess",calib.getImageCount()),leftImg);
			cv::imwrite(FileNameFormat("calibration_image\\right_chess",calib.getImageCount()),rightImg);
			std::cout<<"file saving"<<calib.getImageCount()<<std::endl;
		}
	}

	if(key =='p')
	{
		bool isRectification=true;

		calib.solveStereoParameter();
		calib.showIntrinsicParameters();

		calib.getRectificationMatrix(&homographyLeft,&homographyRight);
		calib.showRectificationHomography();
		Solved=true;
	}

#ifdef OVRVISION	
	if(key=='w'){
		int brigt=g_pOvrvision->GetBrightness()+1;
		g_pOvrvision->SetBrightness(brigt);
		std::cout<<"bright:"<<brigt<<std::endl;
	}
	if(key=='s'){
		int brigt=g_pOvrvision->GetBrightness()-1;
		g_pOvrvision->SetBrightness(brigt);
		std::cout<<"bright:"<<brigt<<std::endl;
	}
	if(key=='a'){
		int cont=g_pOvrvision->GetContrast()-1;
		g_pOvrvision->SetContrast(cont);
		std::cout<<"contrast:"<<cont<<std::endl;

	}
	if(key=='d'){
		int cont=g_pOvrvision->GetContrast()+1;
		g_pOvrvision->SetContrast(cont);
		std::cout<<"contrast:"<<cont<<std::endl;

	}
#endif	
	if(Solved==true){
		cv::undistort(leftImg,renderleft,calib.leftCamera.intrinsic,calib.leftCamera.distortion);
		cv::undistort(rightImg,renderright,calib.rightCamera.intrinsic,calib.rightCamera.distortion);
		cv::imshow(default_left,leftImg);
		cv::imshow(default_right,rightImg);

	}else{
		renderleft=leftImg.clone();
		renderright=rightImg.clone();
	}
	if(key =='q'){
		break;
	}


	cv::imshow(leftwindowName,renderleft);
	cv::imshow(rightwindowName,renderright);

	}
#ifdef OVRVISION
	g_pOvrvision->Close();

#endif
	return 0;
}
