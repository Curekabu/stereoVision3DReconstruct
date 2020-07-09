/*
Copyright 2011. All rights reserved.
Institute of Measurement and Control Systems
Karlsruhe Institute of Technology, Germany

This file is part of libelas.
Authors: Andreas Geiger

libelas is free software; you can redistribute it and/or modify it under the
terms of the GNU General Public License as published by the Free Software
Foundation; either version 3 of the License, or any later version.

libelas is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
libelas; if not, write to the Free Software Foundation, Inc., 51 Franklin
Street, Fifth Floor, Boston, MA 02110-1301, USA 
*/

// Demo program showing how libelas can be used, try "./elas -h" for help

#include <iostream>
#include "elas.h"
#include "image.h"
#include <algorithm>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/video.hpp"
#include <opencv2/opencv.hpp>
#include "opencv2/calib3d.hpp"



using namespace cv;
using namespace std;

// compute disparities of pgm image input pair file_1, file_2
void process (const char* file_1,const char* file_2) {

  cout << "Processing: " << file_1 << ", " << file_2 << endl;

  // load images
  image<uchar> *I1,*I2;
  I1 = loadPGM(file_1);
  I2 = loadPGM(file_2);

  // check for correct size
  if (I1->width()<=0 || I1->height() <=0 || I2->width()<=0 || I2->height() <=0 ||
      I1->width()!=I2->width() || I1->height()!=I2->height()) {
    cout << "ERROR: Images must be of same size, but" << endl;
    cout << "       I1: " << I1->width() <<  " x " << I1->height() << 
                 ", I2: " << I2->width() <<  " x " << I2->height() << endl;
    delete I1;
    delete I2;
    return;    
  }

  // get image width and height
  int32_t width  = I1->width();
  int32_t height = I1->height();

  // allocate memory for disparity images
  const int32_t dims[3] = {width,height,width}; // bytes per line = width
  float* D1_data = (float*)malloc(width*height*sizeof(float));
  float* D2_data = (float*)malloc(width*height*sizeof(float));

  // process
  Elas::parameters param;
  param.postprocess_only_left = false;
  Elas elas(param);
  elas.process(I1->data,I2->data,D1_data,D2_data,dims);

  // find maximum disparity for scaling output disparity images to [0..255]
  float disp_max = 0;
  for (int32_t i=0; i<width*height; i++) {
    if (D1_data[i]>disp_max) disp_max = D1_data[i];
    if (D2_data[i]>disp_max) disp_max = D2_data[i];
  }

  // copy float to uchar
  image<uchar> *D1 = new image<uchar>(width,height);
  image<uchar> *D2 = new image<uchar>(width,height);
  for (int32_t i=0; i<width*height; i++) {
    D1->data[i] = (uint8_t)max(255.0*D1_data[i]/disp_max,0.0);
    D2->data[i] = (uint8_t)max(255.0*D2_data[i]/disp_max,0.0);
  }

  // save disparity images
  char output_1[1024];
  char output_2[1024];
  strncpy(output_1,file_1,strlen(file_1)-4);
  strncpy(output_2,file_2,strlen(file_2)-4);
  output_1[strlen(file_1)-4] = '\0';
  output_2[strlen(file_2)-4] = '\0';
  strcat(output_1,"_disp.pgm");
  strcat(output_2,"_disp.pgm");
  savePGM(D1,output_1);
  savePGM(D2,output_2);

  // free memory
  delete I1;
  delete I2;
  delete D1;
  delete D2;
  free(D1_data);
  free(D2_data);
}

int main (int argc, char** argv) {
  // run demo
  if (argc==2 && !strcmp(argv[1],"demo")) {
    //process("D:/GitHub/libelas/Bulid/Debug/img/cones_left.pgm",   "D:/GitHub/libelas/Bulid/Debug/img/cones_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/aloe_left.pgm",    "D:/GitHub/libelas/Bulid/Debug/img/aloe_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/raindeer_left.pgm","D:/GitHub/libelas/Bulid/Debug/img/raindeer_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/urban1_left.pgm",  "D:/GitHub/libelas/Bulid/Debug/img/urban1_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/urban2_left.pgm",  "D:/GitHub/libelas/Bulid/Debug/img/urban2_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/urban3_left.pgm",  "D:/GitHub/libelas/Bulid/Debug/img/urban3_right.pgm");
    //process("D:/GitHub/libelas/Bulid/Debug/img/urban4_left.pgm",  "D:/GitHub/libelas/Bulid/Debug/img/urban4_right.pgm");
    /*cout << "... done!" << endl;*/
	VideoCapture Capture(2);
	Mat frame;

#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720

	Mat left_cameraMatrix = Mat::eye(3, 3, CV_64F);//左相机内参矩阵
	left_cameraMatrix.at<double>(0, 0) = 1.918336153824488e+03;//Fx
	left_cameraMatrix.at<double>(0, 1) = 0;
	left_cameraMatrix.at<double>(0, 2) = 6.907303693220415e+02;//Cx
	left_cameraMatrix.at<double>(1, 1) = 1.929292488197410e+03;//Fy
	left_cameraMatrix.at<double>(1, 2) = 6.284631474799424e+02;//Cy

	Mat left_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
	left_distCoeffs.at<double>(0, 0) = -0.035177753004162;//k1
	left_distCoeffs.at<double>(1, 0) = 0.529741452707019;//k2
	left_distCoeffs.at<double>(2, 0) = 0;//p1
	left_distCoeffs.at<double>(3, 0) = 0;//p2
	left_distCoeffs.at<double>(4, 0) = 0;

	Mat right_cameraMatrix = Mat::eye(3, 3, CV_64F);//右相机内参矩阵
	right_cameraMatrix.at<double>(0, 0) = 1.915849735633710e+03;//Fx
	right_cameraMatrix.at<double>(0, 1) = 0;
	right_cameraMatrix.at<double>(0, 2) = 6.147645852519341e+02;//Cx
	right_cameraMatrix.at<double>(1, 1) = 1.925977816564622e+03;//Fy
	right_cameraMatrix.at<double>(1, 2) = 6.307232887078274e+02;//Cy

	Mat right_distCoeffs = Mat::zeros(5, 1, CV_64F);//畸变系数
	right_distCoeffs.at<double>(0, 0) = -0.005231683363986;//k1
	right_distCoeffs.at<double>(1, 0) = 0.060025441793370;//k2
	right_distCoeffs.at<double>(2, 0) = 0;//p1
	right_distCoeffs.at<double>(3, 0) = 0;//p2
	right_distCoeffs.at<double>(4, 0) = 0;


	Mat left_map1, left_map2;
	Mat right_map1, right_map2;
	Size imageSize;
	imageSize = Size(FRAME_WIDTH >> 1, FRAME_HEIGHT);
	initUndistortRectifyMap(left_cameraMatrix, left_distCoeffs, Mat(), getOptimalNewCameraMatrix(left_cameraMatrix, left_distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, left_map1, left_map2);
	initUndistortRectifyMap(right_cameraMatrix, right_distCoeffs, Mat(), getOptimalNewCameraMatrix(right_cameraMatrix, right_distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, right_map1, right_map2);

	if (Capture.isOpened()) {
		Capture.set(CAP_PROP_FRAME_WIDTH, 2560);
		Capture.set(CAP_PROP_FRAME_HEIGHT, 720);

		while (1) {
			Capture.read(frame);
			Mat frame_L = frame.colRange(0, 1280);
			Mat frame_R = frame.colRange(1280, 2560);
			Mat frame_L_gray,frame_R_gray;
			Mat frame_calibrated_L, frame_calibrated_R;
			cvtColor(frame_L, frame_L_gray, COLOR_BGR2GRAY);
			cvtColor(frame_R, frame_R_gray, COLOR_BGR2GRAY);

			//remap(frame_L, rectifyImageL, rmap[0][0], rmap[0][1], INTER_LINEAR);
			//remap(frame_R, rectifyImageR, rmap[1][0], rmap[1][1], INTER_LINEAR);

			remap(frame_L_gray, frame_calibrated_L, left_map1, left_map2, INTER_LINEAR);
			remap(frame_R_gray, frame_calibrated_R, right_map1, right_map2, INTER_LINEAR);

			imshow("frameR:", frame_L_gray);
			imshow("frameL:", frame_R_gray);
			int Key = waitKey(1);
			if (Key == 'q') break;
			if (Key == 'p') {
				cout << "L:" << frame_L_gray.cols << endl;
				cout << "R:" << frame_R_gray.cols << endl;
			}
			if (Key == 'w') {
				if (imwrite("D:/GitHub/libelas/Bulid/Debug/img/testL.pgm", frame_L_gray))
					cout << "L Output complet" << endl;
				if (imwrite("D:/GitHub/libelas/Bulid/Debug/img/testR.pgm", frame_R_gray))
					cout << "R Output complet" << endl;
				process("D:/GitHub/libelas/Bulid/Debug/img/testL.pgm", "D:/GitHub/libelas/Bulid/Debug/img/testR.pgm");
				Mat image = imread("D:/GitHub/libelas/Bulid/Debug/img/testL_disp.pgm");
				imshow("Process:", image);
				waitKey();
				break;
			}
		}
	}
	
	Capture.release();
  // compute disparity from input pair
  } else if (argc==3) {
    process(argv[1],argv[2]);
    cout << "... done!" << endl;

  // display help
  } else {
    cout << endl;
    cout << "ELAS demo program usage: " << endl;
    cout << "./elas demo ................ process all test images (image dir)" << endl;
    cout << "./elas left.pgm right.pgm .. process a single stereo pair" << endl;
    cout << "./elas -h .................. shows this help" << endl;
    cout << endl;
    cout << "Note: All images must be pgm greylevel images. All output" << endl;
    cout << "      disparities will be scaled such that disp_max = 255." << endl;
    cout << endl;
  }

  return 0;
}


