#pragma once
#include "opencv2/core.hpp"
#include "viso_stereo.h"

class KeyFrame
{
public:
	KeyFrame() {
		frameL = cv::Mat();
		frameR = cv::Mat();
		disparity = cv::Mat();
		disp_3d = cv::Mat();
		colorL = cv::Mat();
	}
	cv::Mat frameL;
	cv::Mat frameR;
	cv::Mat poseMatrix;
	cv::Mat disparity;
	cv::Mat disp_3d;
	cv::Mat colorL;
	
};
