#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d.hpp"
#include <mutex>
using namespace std;

#define FRAME_WIDTH (640)  //640
#define FRAME_HEIGHT (360)  //360

class Capture
{
public:
	Capture() {  //构造函数
		if (capture.isOpened()) {
			capture.set(cv::CAP_PROP_FRAME_WIDTH, 2560);
			capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
		}
	}
	void initRectifyMap();         //初始化校正map
	void readFrame2remap(cv::Mat& frameL, cv::Mat& frameR, cv::Mat& colorL);  //读取一帧并进行分割并转为灰度图加校正 存放在frameL 和frameR 中
	void displayframe(cv::Mat &frameL, cv::Mat &frameR);  //显示一对捕获到的图像
	cv::Mat Q;
private:
	cv::VideoCapture capture = cv::VideoCapture(0);
	cv::Mat left_map1, left_map2;
	cv::Mat right_map1, right_map2;
	
	
};



