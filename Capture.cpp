#include "Capture.h"
using namespace cv;
extern std::mutex frameMutex;

void Capture::initRectifyMap() {
    Mat left_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
    left_distCoeffs.at<double>(0, 0) = 0.1472;//k1
    left_distCoeffs.at<double>(1, 0) = -0.2688;//k2
    left_distCoeffs.at<double>(2, 0) = 7.8973e-04;//p1
    left_distCoeffs.at<double>(3, 0) = -3.5670e-04;//p2
    left_distCoeffs.at<double>(4, 0) = 0.1101;

    Mat right_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
    right_distCoeffs.at<double>(0, 0) = 0.1448;//k1
    right_distCoeffs.at<double>(1, 0) = -0.2640;//k2
    right_distCoeffs.at<double>(2, 0) = 0.0013;//p1
    right_distCoeffs.at<double>(3, 0) = 2.5988e-04;//p2
    right_distCoeffs.at<double>(4, 0) = 0.1080;

    Mat R = Mat::zeros(3, 3, CV_64F); //旋转关系矩阵
    R.at<double>(0, 0) = 1.0;
    R.at<double>(0, 1) = -1.1446e-04;
    R.at<double>(0, 2) = 0.0032;
    R.at<double>(1, 0) = 1.3084e-04;
    R.at<double>(1, 1) = 1.0;
    R.at<double>(1, 2) = -0.0051;
    R.at<double>(2, 0) = -0.0032;
    R.at<double>(2, 1) = 0.0051;
    R.at<double>(2, 2) = 1.0;

    Mat T = Mat::zeros(3, 1, CV_64F); //平移关系矩阵
    T.at<double>(0, 0) = -58.8364;
    T.at<double>(1, 0) = -0.0910;
    T.at<double>(2, 0) = 0.6528;

    Mat camera1Matrix = (Mat_<double>(3, 3) <<
        367.4534, 0.0, 308.0705,
        0.0, 367.7149, 170.8232,
        0.0, 0.0, 1.0);

    Mat camera2Matrix = (Mat_<double>(3, 3) <<
        367.0244, 0.0, 327.2887,
        0.0, 367.1504, 173.5917,
        0.0, 0.0, 1.0);
    //Mat left_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
    //left_distCoeffs.at<double>(0, 0) = 0.1222;//k1
    //left_distCoeffs.at<double>(1, 0) = -0.2129;//k2
    //left_distCoeffs.at<double>(2, 0) = 5.2616e-04;//p1
    //left_distCoeffs.at<double>(3, 0) = 0.0016;//p2
    //left_distCoeffs.at<double>(4, 0) = 0.0598;

    //Mat right_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
    //right_distCoeffs.at<double>(0, 0) = 0.1291;//k1
    //right_distCoeffs.at<double>(1, 0) = -0.2361;//k2
    //right_distCoeffs.at<double>(2, 0) = 8.6797e-04;//p1
    //right_distCoeffs.at<double>(3, 0) = 0.001;//p2
    //right_distCoeffs.at<double>(4, 0) = 0.0839;

    //Mat R = Mat::zeros(3, 3, CV_64F); //旋转关系矩阵
    //R.at<double>(0, 0) = 1.0;
    //R.at<double>(0, 1) = -8.3864e-05;
    //R.at<double>(0, 2) = 0.0016;
    //R.at<double>(1, 0) = 9.0364e-05;
    //R.at<double>(1, 1) = 1.0;
    //R.at<double>(1, 2) = -0.0042;
    //R.at<double>(2, 0) = -0.0016;
    //R.at<double>(2, 1) = 0.0042;
    //R.at<double>(2, 2) = 1.0;

    //Mat T = Mat::zeros(3, 1, CV_64F); //平移关系矩阵
    //T.at<double>(0, 0) = -58.8787;
    //T.at<double>(1, 0) = -0.0445;
    //T.at<double>(2, 0) = -0.3016;

    //Mat camera1Matrix = (Mat_<double>(3, 3) <<
    //    720.5104, 0.0, 616.3840,
    //    0.0, 720.2441, 337.2850,
    //    0.0, 0.0, 1.0);

    //Mat camera2Matrix = (Mat_<double>(3, 3) <<
    //    730.7860, 0.0, 654.2299,
    //    0.0, 720.2101, 342.4267,
    //    0.0, 0.0, 1.0);

    Size imageSize;
    imageSize = Size(FRAME_WIDTH, FRAME_HEIGHT);
    Mat R1, R2, P1, P2;
    stereoRectify(camera1Matrix, left_distCoeffs, camera2Matrix, right_distCoeffs, imageSize, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, 0, imageSize);
    initUndistortRectifyMap(camera1Matrix, left_distCoeffs, R1, P1, imageSize, CV_16SC2, left_map1, left_map2);
    initUndistortRectifyMap(camera2Matrix, right_distCoeffs, R2, P2, imageSize, CV_16SC2, right_map1, right_map2);
}

void Capture::readFrame2remap(Mat& frameL, Mat& frameR, Mat& colorL) {
    //读取frame并分割为左右
    Mat frame;
    capture.read(frame);

    Mat frame_L = frame.colRange(0, FRAME_WIDTH * 2);
    Mat frame_R = frame.colRange(FRAME_WIDTH * 2, FRAME_WIDTH * 4);

    Mat resize_frame_L, resize_frame_R;
    resize(frame_L, resize_frame_L, cv::Size(0, 0), 0.5, 0.5, INTER_AREA);
    resize(frame_R, resize_frame_R, cv::Size(0, 0), 0.5, 0.5, INTER_AREA);

    //remap
    remap(resize_frame_L, frame_L, left_map1, left_map2, INTER_LINEAR);
    remap(resize_frame_R, frame_R, right_map1, right_map2, INTER_LINEAR);

    //转换为灰度图
    lock_guard<mutex> lockGuard(frameMutex);
    colorL = frame_L.clone();
    cvtColor(frame_L, frameL, COLOR_BGR2GRAY);
    cvtColor(frame_R, frameR, COLOR_BGR2GRAY);

}

void Capture::displayframe(Mat &frameL,Mat &frameR) {
    Mat view(cv::Size(FRAME_WIDTH * 2, FRAME_HEIGHT), CV_8U);
    Mat viewL = view(Rect(0, 0, FRAME_WIDTH, FRAME_HEIGHT));
    Mat viewR = view(Rect(FRAME_WIDTH, 0, 640, FRAME_HEIGHT));
    {
        lock_guard<mutex> lockGuard(frameMutex);
        frameL.copyTo(viewL);
        frameR.copyTo(viewR);
    }
    cv::imshow("view:", view);
    waitKey(1);
}