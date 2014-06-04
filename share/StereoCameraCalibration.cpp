#include "StereoCameraCalibration.h"
#include <iostream>


void StereoCameraCalibration::getKRK(const cv::Mat& srcK,cv::Mat* R, cv::Mat* destK, cv::Mat* dest)
{
	cv::Mat tmp=srcK.clone();
	cv::Mat swap=srcK.clone();
	*R=tmp.inv()*swap;

	*destK=swap* *dest;

	return;
}

cv::Mat StereoCameraCalibration::getKRK(const cv::Mat &srcK,const cv::Mat &R) const
{
	return srcK.inv()*R*srcK;
}

StereoCameraCalibration::StereoCameraCalibration(cv::Size img_size, cv::Size _pattern_size,double _CHESS_LENGTH, int _max_chess_count/*=1000*/)
{
	Init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);

	leftCamera.Init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);
	rightCamera.Init(img_size,_pattern_size,_CHESS_LENGTH,_max_chess_count);
	relate_rot.create (1, 3, CV_64FC1);
	relate_trans.create (1, 3, CV_64FC1);
	E=cv::Mat::eye(3, 3, CV_64FC1);
	F=cv::Mat::eye(3, 3, CV_64FC1);
	rectification_H_left=cv::Mat::eye(3, 3, CV_64FC1);
	rectification_H_right=cv::Mat::eye(3, 3, CV_64FC1);
	
	remap_left_x=cv::Mat(img_size,CV_32FC1);
	remap_left_y=cv::Mat(img_size,CV_32FC1);
	remap_right_x=cv::Mat(img_size,CV_32FC1);
	remap_right_y=cv::Mat(img_size,CV_32FC1);

}

bool StereoCameraCalibration::findChess(cv::Mat& left_img,cv::Mat& right_img)
{
	bool ret=false;
	ret = leftCamera.FindChessBoardCorners(left_img,false);
	if(ret)ret = rightCamera.FindChessBoardCorners(right_img,false);

	if (ret) 
	{	
		leftCamera.FindChessBoardCorners(left_img,true);
		rightCamera.FindChessBoardCorners(right_img,true);
		//p_countはどうする？
		image_count++;
	}	
	return ret;
}

void StereoCameraCalibration::drawChessboardCorners(const cv::Mat* src, cv::Mat* dest,int left_or_right/*=STEREO_CALIB_LEFT*/)
{

	if(left_or_right==STEREO_CALIB_LEFT)
	{
		leftCamera.DrawChessboardCorners(*src,*dest);			
	}
	else
	{
		rightCamera.DrawChessboardCorners(*src,*dest);			
	}
}

void StereoCameraCalibration::showIntrinsicParameters() const
{
	std::cout<<"Left Intrinsic Parameters:"<<std::endl;
	std::cout<<leftCamera.intrinsic<<std::endl;
	std::cout<<"Left Distortion Parameters:"<<std::endl;
	std::cout<<leftCamera.distortion<<std::endl;

	std::cout<<"Right Intrinsic Parameters:"<<std::endl;
	std::cout<<rightCamera.intrinsic<<std::endl;
	std::cout<<"Right Distortion Parameters:"<<std::endl;
	std::cout<<rightCamera.distortion<<std::endl;
}

void StereoCameraCalibration::showExtrinsicParameters() const
{
	std::cout<<"Left Translation:"<<std::endl;
	std::cout<<leftCamera.translation<<std::endl;
	std::cout<<"Left Rotation:"<<std::endl;
	std::cout<<leftCamera.rotation<<std::endl;

	std::cout<<"Right Translation:"<<std::endl;
	std::cout<<rightCamera.translation<<std::endl;
	std::cout<<"Right Rotation:"<<std::endl;
	std::cout<<rightCamera.rotation<<std::endl;

	std::cout<<"Relative Rotation:"<<std::endl;
	std::cout<<relate_rot<<std::endl;
	std::cout<<"Relative Translation:"<<std::endl;
	std::cout<<relate_trans<<std::endl;

	std::cout<<"Essential Matrix:"<<std::endl;
	std::cout<<E<<std::endl;
	std::cout<<"Fundamental Matrix:"<<std::endl;
	std::cout<<F<<std::endl;
}

void StereoCameraCalibration::showRectificationHomography() const
{

	std::cout<<"Left Rectification Homography:"<<std::endl;
	std::cout<<rectification_H_left<<std::endl;
	std::cout<<"Right Rectification Homography:"<<std::endl;
	std::cout<<rectification_H_right<<std::endl;

}

void StereoCameraCalibration::solveStereoParameter()
{
	if(image_count<3)
	{
		std::cout<<"3 or more input images are required"<<std::endl;
		return;
	}

	//setup 2D-3D points data

	//all data
	std::vector <std::vector <cv::Point3f> > object_points;
	std::vector <std::vector <cv::Point2f> > image_points1;
	std::vector <std::vector <cv::Point2f> > image_points2;

	//each picture cache
	std::vector<cv::Point3f> objects(pattern_size.area());
	std::vector<cv::Point2f> corners1(pattern_size.area());
	std::vector<cv::Point2f> corners2(pattern_size.area());
	
	for (int i = 0; i < image_count; i++)
	{
		for (int j = 0; j < pattern_size.height; j++) 
		{
			for (int k = 0; k < pattern_size.width; k++) 
			{
				objects[ j * pattern_size.width + k].x = (float)(j * CHESS_LENGTH_MM);
				objects[ j * pattern_size.width + k].y = (float)(k * CHESS_LENGTH_MM);
				objects[ j * pattern_size.width + k].z = 0.0f;

				corners1[ j * pattern_size.width + k].x = leftCamera.subpix_corners[i][j * pattern_size.width + k].x;
				corners1[ j * pattern_size.width + k].y = leftCamera.subpix_corners[i][j * pattern_size.width + k].y;
				corners2[ j * pattern_size.width + k].x = rightCamera.subpix_corners[i][j * pattern_size.width + k].x;
				corners2[ j * pattern_size.width + k].y = rightCamera.subpix_corners[i][j * pattern_size.width + k].y;

			}
		}
		//copy current points to all
		object_points.push_back(objects);
		image_points1.push_back(corners1);
		image_points2.push_back(corners2);
	}		
	std::cout<<"setup 2D-3D data ok"<<std::endl;

	std::vector<cv::Mat> rvecs;
	std::vector<cv::Mat> tvecs;
	double result_left=	cv::calibrateCamera(object_points,image_points1,image_size,leftCamera.intrinsic,leftCamera.distortion,rvecs,tvecs);
	std::cout<<"left: intrinsic:"<<std::endl<<leftCamera.intrinsic<<std::endl;

	double result_right=	cv::calibrateCamera(object_points,image_points2,image_size,rightCamera.intrinsic,rightCamera.distortion,rvecs,tvecs);
	std::cout<<"right: intrinsic:"<<std::endl<<leftCamera.intrinsic<<std::endl;
	// save intrinsic parameters
	cv::FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "LeftCameraInstric" << leftCamera.intrinsic << "LeftCameraDistortion" << leftCamera.distortion <<
			"RightCameraInstrics" << rightCamera.intrinsic << "RightCameraDistortion" << rightCamera.distortion;
		fs.release();
	} else{
		std::cout << "Error: can not save the intrinsic parameters\n";
	}
	cv::TermCriteria criteria( CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.001 );

	double rms=cv::stereoCalibrate(object_points,image_points1,image_points2,
		leftCamera.intrinsic,leftCamera.distortion,
		rightCamera.intrinsic,rightCamera.distortion,
		image_size,relate_rot,relate_trans,E,F,criteria,CV_CALIB_RATIONAL_MODEL| CV_CALIB_USE_INTRINSIC_GUESS| CV_CALIB_FIX_PRINCIPAL_POINT);

	std::cout << "done with RMS error=" << rms << std::endl;
	std::cout << "LeftCameraInstric" << leftCamera.intrinsic << "LeftCameraDistortion" << leftCamera.distortion <<
		"RightCameraInstrics" << rightCamera.intrinsic << "RightCameraDistortion" << rightCamera.distortion;
	
	//クオリティチェックするべき？
	//もしやるなら、こんな感じ
	//
	/*
	double err = 0;
	int npoints = 0;
	std::vector<Vec3f> lines[2];
	for(int i = 0; i < image_count; i++ )
	{
		int npt = (int)imagePoints[0][i].size();
		cv::Mat imgpt[2];
		for( k = 0; k < 2; k++ )
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k+1, F, lines[k]);
		}
		for( j = 0; j < npt; j++ )
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " <<  err/npoints << endl;

	*/

	return;
}

void StereoCameraCalibration::getRectificationMatrix(cv::Mat* h_left, cv::Mat* h_right)
{

	cv::Mat R1(3,3,CV_64FC1);
	cv::Mat R2(3,3,CV_64FC1);
	cv::Mat P1(3,4,CV_64FC1);
	cv::Mat P2(3,4,CV_64FC1);
	cv::Mat k1(3,3,CV_64FC1);
	cv::Mat k2(3,3,CV_64FC1);
	cv::Mat Q(4,4,CV_64FC1);//4 x4 視差とデプス間のマッピング行列 reprojectImageTo3D参照
	cv::stereoRectify(leftCamera.intrinsic,leftCamera.distortion,
		rightCamera.intrinsic,rightCamera.distortion,image_size,relate_rot,relate_trans,
		R1,R2,P1,P2,Q);
	cv::FileStorage fs; 
	fs.open("stereoExtrinsics.yml", CV_STORAGE_WRITE);
	if( fs.isOpened() )
	{
		fs << "LeftCameraInstric" << leftCamera.intrinsic << "LeftCameraDistortion" << leftCamera.distortion <<
			"RightCameraInstrics" << rightCamera.intrinsic << "RightCameraDistortion" << rightCamera.distortion;
		fs << "R" << relate_rot << "T" << relate_trans << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else{
		std::cout << "Error: can not save the intrinsic parameters\n";
	}
	//縦ステレオか横ステレオか
	bool isVerticalStereo = fabs(P2.at<double>(1, 3)) > fabs(P2.at<double>(0, 3));

	//Precompute maps for cv::remap()
	getKRK(leftCamera.intrinsic,&R1,&k1,&rectification_H_left);
	getKRK(rightCamera.intrinsic,&R2,&k2,&rectification_H_right);

	h_left= &rectification_H_left.clone();
	h_right= &rectification_H_right.clone();

	initUndistortRectifyMap(leftCamera.intrinsic, leftCamera.distortion, R1, P1, image_size, CV_16SC2, remap_left_x, remap_left_y);
	initUndistortRectifyMap(rightCamera.intrinsic, rightCamera.distortion, R2, P2, image_size, CV_16SC2, remap_right_x, remap_right_y);
	
	return;
}

void StereoCameraCalibration::rectifyImageRemap(cv::Mat* src, cv::Mat* dest, stereo_left_right left_right)
{
	cv::Mat _dest=src->clone();
	if(left_right==STEREO_CALIB_LEFT){
	cv::remap(*src,_dest,remap_left_x,remap_left_y,cv::INTER_LINEAR);
	}
	if(left_right==STEREO_CALIB_RIGHT){
	cv::remap(*src,_dest,remap_right_x,remap_right_y,cv::INTER_LINEAR);
	}

	*dest=_dest.clone();

	return;
}
