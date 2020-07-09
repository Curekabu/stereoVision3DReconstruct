#include "ORB_SLAM2_VO.h"

cv::Mat SLAM::track(const cv::Mat& imageL, const cv::Mat& imageR)
{
	cv::Mat pose;
	static double timestamp = 0;
	pose = slam.TrackStereo(imageL, imageR, timestamp);
	timestamp += 0.01;
	return pose;
}

void SLAM::shoutdown()
{
	slam.Shutdown();
}

int SLAM::GetTrackingState()
{
	int state = slam.GetTrackingState();
	return state;
}

std::vector<cv::KeyPoint> SLAM::GetTrackedKeyPointsUn()
{
	return slam.GetTrackedKeyPointsUn();
}

std::vector<ORB_SLAM2::MapPoint*> SLAM::GetTrackedMapPoints()
{
	return slam.GetTrackedMapPoints();
}
