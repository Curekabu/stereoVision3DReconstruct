#pragma once
#include "System.h"
#include "opencv2/core.hpp"

class SLAM
{

public:
	SLAM(const string& strVocFile, const string& strSettingsFile) :
		slam(strVocFile, strSettingsFile, ORB_SLAM2::System::STEREO, true) {};

	ORB_SLAM2::System slam;

	cv::Mat track(const cv::Mat& imageL, const cv::Mat& imageR);

	void shoutdown();

	int GetTrackingState();

	std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

	std::vector<ORB_SLAM2::MapPoint*> GetTrackedMapPoints();
};

