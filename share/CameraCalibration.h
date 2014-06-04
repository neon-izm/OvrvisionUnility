#ifndef _CAMERA_CALIBRATION_5752FB0E_H_
#define _CAMERA_CALIBRATION_5752FB0E_H_

#include <opencv2/opencv.hpp>
#include "opencv_lib.hpp"
#include <vector>
class CameraCalibration
{
public:
	void Init(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH_MM, int _max_chess_count=10000);
	CameraCalibration(void);
	CameraCalibration(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH_MM, int _max_chess_count=10000);
	~CameraCalibration(void);
	cv::Mat intrinsic;
	cv::Mat rodrigues;
	cv::Mat rotation;
	cv::Mat translation;
	cv::Mat distortion;
	int GetImageCount()const {return image_count;};

	bool FindChessBoardCorners(const cv::Mat& src_img, bool isStore=true);
	void DrawChessboardCorners(const cv::Mat& src, cv::Mat& dest);
	double SolveIntrinsic();//return correctness_val 

	//for_debug
	void SolveExtrinsic(const int pattern_number);

	std::vector< std::vector< cv::Point2f> > subpix_corners;

protected:
	int image_count;
	cv::Size image_size;//image resolution
	cv::Size pattern_size;//num. of col and row in a pattern

	double CHESS_LENGTH_MM;//length(mm)
	
	int MAX_CHESS_COUNT;

	int isFound;

};

#endif