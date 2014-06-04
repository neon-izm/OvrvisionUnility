#include "CameraCalibration.h"
#include <iostream>
void CameraCalibration::Init(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH_MM, int _max_chess_count/*=10000*/){
	image_count = 0;
	image_size = img_size;
	pattern_size=_pattern_size;
	CHESS_LENGTH_MM=_CHESS_LENGTH_MM;

	intrinsic=cv::Mat::eye(3, 3, CV_64FC1);
	rodrigues.create(1, 3, CV_64FC1);
	rotation.create (3, 3, CV_64FC1);
	translation.create (1, 3, CV_64FC1);
	distortion.create (1, 8, CV_64FC1);//distortion param 8

	MAX_CHESS_COUNT = _max_chess_count;
	
	return;
}
CameraCalibration::CameraCalibration(void)
{
}
CameraCalibration::CameraCalibration(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH, int _max_chess_count/*=10000*/){
	Init(img_size, _pattern_size, _CHESS_LENGTH, _max_chess_count);
}

CameraCalibration::~CameraCalibration(void)
{
}

bool CameraCalibration::FindChessBoardCorners(const cv::Mat& src_img, bool isStore/*=true*/)
{
	cv::vector<cv::Point2f> current_corners;
	isFound=cv::findChessboardCorners (src_img, pattern_size, current_corners);
	if(isFound==false){return false;}

	if(isStore){
	cv::TermCriteria criteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001 );
	cv::Mat src_gray;
	cv::cvtColor(src_img, src_gray ,CV_BGR2GRAY);
	cv::cornerSubPix( src_gray, current_corners, cv::Size( 11, 11 ), cv::Size( -1, -1 ), criteria );
	subpix_corners.push_back(current_corners);
	image_count++;
	}
	return true;
}

void CameraCalibration::DrawChessboardCorners(const cv::Mat& src, cv::Mat& dest)
{
	dest=src.clone();
	if(image_count-1<0){return;}
	cv::drawChessboardCorners(dest,pattern_size,subpix_corners[image_count-1],isFound);

}

void CameraCalibration::SolveExtrinsic(const int pattern_number)
{
	//early return
	if(pattern_number>image_count){return;}

	//cv::Size::area() ==row*col
	std::vector<cv::Point3f > objects(pattern_size.area());
	// ê¢äEç¿ïWÇåàÇﬂÇÈ
	for (int j = 0; j < pattern_size.height; j++) 
	{
		for (int k = 0; k < pattern_size.width; k++) 
		{
			objects[j * pattern_size.width + k].x = (float)(j * CHESS_LENGTH_MM);
			objects[j * pattern_size.width + k].y = (float)(k * CHESS_LENGTH_MM);
			objects[j * pattern_size.width + k].z = 0.0f;
		}
	}
	cv::solvePnP(objects, subpix_corners[pattern_number], intrinsic, distortion, rodrigues, translation);
	cv::Rodrigues(rodrigues,rotation);
	std::cout<<"Rotation"<<std::endl<<rotation;
	std::cout<<"Translation"<<std::endl<<translation;

	return;
}

double CameraCalibration::SolveIntrinsic()
{
	if(image_count<3)
	{
		std::cout<<"3 or more input images are required"<<std::endl;
		return -1.0;
	}
	std::vector <std::vector <cv::Point3f> > object_points_all;

	for (int i=0; i<image_count; i++)
	{
	std::vector<cv::Point3f > objects(pattern_size.area());
	// ê¢äEç¿ïWÇåàÇﬂÇÈ
	for (int j = 0; j < pattern_size.height; j++) 
	{
		for (int k = 0; k < pattern_size.width; k++) 
		{
			objects[j * pattern_size.width + k].x = (float)(j * CHESS_LENGTH_MM);
			objects[j * pattern_size.width + k].y = (float)(k * CHESS_LENGTH_MM);
			objects[j * pattern_size.width + k].z = 0.0f;
		}
	}
	object_points_all.push_back(objects);
	}
	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	double result=	cv::calibrateCamera(object_points_all,subpix_corners,image_size,intrinsic,distortion,rvecs,tvecs);
	std::cout<<" intrinsic:"<<std::endl<<intrinsic<<std::endl;
	std::cout<<"\n distortion:"<<std::endl<<distortion<<std::endl;
	return result;
}
