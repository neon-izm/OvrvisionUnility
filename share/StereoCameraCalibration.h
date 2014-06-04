#ifndef _STEREO_CAMERA_CALIBRATION_H_
#define _STEREO_CAMERA_CALIBRATION_H_
#include "CameraCalibration.h"


class StereoCameraCalibration : protected CameraCalibration
{
public:

	enum stereo_left_right
	{
		STEREO_CALIB_LEFT=0,
		STEREO_CALIB_RIGHT
	};
	StereoCameraCalibration(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH, int _max_chess_count=1000);
	~StereoCameraCalibration(){};

	bool findChess(cv::Mat& left_img,cv::Mat& right_img);
	void drawChessboardCorners(const cv::Mat* src, cv::Mat* dest,int left_or_right=STEREO_CALIB_LEFT);

	void showRectificationHomography()const;
	void showExtrinsicParameters()const;
	void showIntrinsicParameters()const;

	void solveStereoParameter();
	void getRectificationMatrix(cv::Mat* h_left, cv::Mat* h_right);
	void rectifyImageRemap(cv::Mat* src, cv::Mat* dest, stereo_left_right left_right);

	int getImageCount()const{return image_count;};

	CameraCalibration leftCamera;
	CameraCalibration rightCamera;

private:
	
	cv::Mat relate_rot;
	cv::Mat relate_trans;

	cv::Mat E;//essential matrix
	cv::Mat F;//fundamental matrix

	cv::Mat rectification_H_left;
	cv::Mat rectification_H_right;

	cv::Mat remap_left_x;
	cv::Mat remap_left_y;
	cv::Mat remap_right_x;
	cv::Mat remap_right_y;

	void getKRK(const cv::Mat& srcK,cv::Mat* R, cv::Mat* destK, cv::Mat* dest);
	cv::Mat getKRK(const cv::Mat &srcK,const cv::Mat &R)const ;
};

#endif