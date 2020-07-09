#include <iostream>
#include "elas.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d.hpp"
#include "viso_stereo.h"


#define DEBUG_ELAS

using namespace cv;
using namespace std;

int main2 (int argc, char* argv[]) {

#define FRAME_WIDTH (640)  //640
#define FRAME_HEIGHT (360)  //360

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

    Mat left_map1, left_map2;
    Mat right_map1, right_map2;
    Size imageSize;
    Mat resize_frame_L, resize_frame_R, frame_L_gray_unidstort, frame_R_gray_unidstort;
    imageSize = Size(FRAME_WIDTH, FRAME_HEIGHT);
    Mat R1,R2,P1,P2,Q;
    stereoRectify(camera1Matrix,left_distCoeffs,camera2Matrix,right_distCoeffs,imageSize,R,T,R1,R2,P1,P2,Q,CALIB_ZERO_DISPARITY,0,imageSize);
    initUndistortRectifyMap(camera1Matrix, left_distCoeffs, R1, P1, imageSize, CV_16SC2, left_map1, left_map2);
    initUndistortRectifyMap(camera2Matrix, right_distCoeffs, R2, P2, imageSize, CV_16SC2, right_map1, right_map2);
    
    VideoCapture Capture(0);
    Mat frame;

    //init stereoBM
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 21);

    Rect validROIL, validROIR;

    // calibration parameters 
    VisualOdometryStereo::parameters visualOdometryParam;
    visualOdometryParam.calib.f = 367.4534; // focal length in pixels
    visualOdometryParam.calib.cu = 308.0705; // principal point (u-coordinate) in pixels
    visualOdometryParam.calib.cv = 170.8232; // principal point (v-coordinate) in pixels
    visualOdometryParam.base = 0.0588364; // baseline in meters   //基线， 两个摄像头光心的距离

    // current pose (this matrix transforms a point from the current
    // frame's camera coordinates to the first frame's camera coordinates)
    // init visual odometry
    VisualOdometryStereo viso(visualOdometryParam);
    Matrix pose = Matrix::eye(4);

    
    Mat disp, disp_3d, disp8, nor_disp8;
    long points = 0;
    int count = 0;

    float* data = new float[100* FRAME_HEIGHT * FRAME_WIDTH * 3];

    namedWindow("set:", WINDOW_NORMAL);
    int BlockSize = 9,
        NumDisparities = 64,
        TextureThreshold = 100,
        UniquenessRatio = 20,
        SpeckleRange = 32,
        SpeckleWindowSize = 200,
        PreFilterType = 0,
        PreFilterSize = 15,
        PreFilterCap = 31;
    createTrackbar("BlockSize", "set:", &BlockSize, 40);
    createTrackbar("NumDisparities", "set:", &NumDisparities, 128);
    createTrackbar("TextureThreshold", "set:", &TextureThreshold, 200);
    createTrackbar("UniquenessRatio", "set:", &UniquenessRatio, 40);
    createTrackbar("SpeckleRange", "set:", &SpeckleRange, 200);
    createTrackbar("SpeckleWindowSize", "set:", &SpeckleWindowSize, 200);
    createTrackbar("PreFilterType", "set:", &PreFilterType, 2);
    createTrackbar("PreFilterSize", "set:", &PreFilterSize, 21);
    createTrackbar("PreFilterCap", "set:", &PreFilterCap, 31);

    if (Capture.isOpened()) {
        Capture.set(CAP_PROP_FRAME_WIDTH, 2560);
        Capture.set(CAP_PROP_FRAME_HEIGHT, 720);

        while (1) {
            //读取frame并分割为左右
            Capture.read(frame);
            Mat frame_L = frame.colRange(0, 1280);
            Mat frame_R = frame.colRange(1280, 2560);


            //转换为灰度图
            Mat frame_L_gray,frame_R_gray;
            Mat frame_calibrated_L, frame_calibrated_R;
            cvtColor(frame_L, frame_L_gray, COLOR_BGR2GRAY);
            cvtColor(frame_R, frame_R_gray, COLOR_BGR2GRAY);
            
            resize(frame_L_gray, resize_frame_L, cv::Size(0, 0), 0.5, 0.5);
            resize(frame_R_gray, resize_frame_R, cv::Size(0, 0), 0.5, 0.5);

            //remap
            remap(resize_frame_L, frame_calibrated_L, left_map1, left_map2, INTER_LINEAR);
            remap(resize_frame_R, frame_calibrated_R, right_map1, right_map2, INTER_LINEAR);

            Mat view(cv::Size(1280, 360), CV_8U);
            Mat viewL = view(Rect(0, 0, 640, 360));
            frame_calibrated_L.copyTo(viewL);

            Mat viewR = view(Rect(640, 0, 640, 360));
            frame_calibrated_R.copyTo(viewR);

         /*   for (int i = 0; i < view.rows; i += 16)
                line(view, Point(0, i), Point(view.cols, i), Scalar(255, 255, 255), 1, 8);*/

            //imshow("frameR_c:", frame_calibrated_R);
            //imshow("frameL_c:", frame_calibrated_L);

            cv::imshow("view:", view);

            
            ////初始化参数
            //int32_t width = FRAME_WIDTH;
            //int32_t height = FRAME_HEIGHT;
            //const int32_t dims[3] = {width,height,width}; // bytes per line = width
            //Elas::parameters elasParam;
            //elasParam.postprocess_only_left = false;
		    
            //// process
			
            //Elas elas(elasParam);
            //elas.process(frame_calibrated_L.data, frame_calibrated_R.data,(float*)D1_image.data,(float*)D2_image.data,dims);


        #ifdef DEBUG_ELAS
            

            //bm->setPreFilterType(1);
            //bm->setPreFilterSize(91);
            //bm->setPreFilterCap(63);

            //bm->setBlockSize(21);
            //bm->setMinDisparity(0);   //确定匹配搜索从哪里开始  默认值是0  

            //bm->setNumDisparities(64);  //在该数值确定的视差范围内进行搜索,视差窗口  
            //                            //即最大视差值与最小视差值之差, 大小必须是16的整数倍  

            //bm->setTextureThreshold(20);   //保证有足够的纹理以克服噪声  
            //                               //低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，
            //                               //则该窗口对应的像素点的视差值为 0

            //bm->setUniquenessRatio(20);   //使用匹配功能模式 
            //                              //视差唯一性百分比， 视差窗口范围内最低代价是次低代价的
            //                              //(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0
            //                              //该参数不能为负值，一般5-15左右的值比较合适

            //bm->setSpeckleWindowSize(100);  //检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
            //bm->setSpeckleRange(16);  // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零 

            bm->setPreFilterType(PreFilterType);
            if (PreFilterSize >= 5 && PreFilterSize % 2 != 0)
                bm->setPreFilterSize(PreFilterSize);
            if (PreFilterCap > 1)
                bm->setPreFilterCap(PreFilterCap);
            bm->setDisp12MaxDiff(-1);

            if (BlockSize % 2 != 0 && BlockSize > 4)
                bm->setBlockSize(BlockSize);
            else if (BlockSize > 4)
                bm->setBlockSize(BlockSize - 1);

            bm->setMinDisparity(0);   //确定匹配搜索从哪里开始  默认值是0  

            if (NumDisparities > 16) {
                NumDisparities -= NumDisparities % 16;
                bm->setNumDisparities(NumDisparities);  //在该数值确定的视差范围内进行搜索,视差窗口  
                                        //即最大视差值与最小视差值之差, 大小必须是16的整数倍  
            }
              
            bm->setTextureThreshold(TextureThreshold);   //保证有足够的纹理以克服噪声  
                                           //低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，
                                           //则该窗口对应的像素点的视差值为 0

            bm->setUniquenessRatio(UniquenessRatio);   //使用匹配功能模式 
                                          //视差唯一性百分比， 视差窗口范围内最低代价是次低代价的
                                          //(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0
                                          //该参数不能为负值，一般5-15左右的值比较合适

            bm->setSpeckleWindowSize(SpeckleWindowSize);  //检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
            bm->setSpeckleRange(SpeckleRange);  // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零 
            bm->compute(frame_calibrated_L, frame_calibrated_R, disp);
            
            disp.convertTo(disp8, CV_8U, 255 / ((64 * 16 + 16) * 16.));
            //disp.convertTo(disp8, CV_8U);

            //归一化
            normalize(disp8,nor_disp8,0,255,NORM_MINMAX);

			imshow("disp:",nor_disp8);

        #endif // DEBUG_ELAS

#define DEBUG_VISO

        #ifdef DEBUG_VISO
            // current pose (this matrix transforms a point from the current
            // frame's camera coordinates to the first frame's camera coordinates)
            

            // image dimensions
            int32_t width  = frame_calibrated_L.cols;
            int32_t height = frame_calibrated_R.rows;
             
            // convert input images to uint8_t buffer
            uint8_t* left_img_data  = (uint8_t*)malloc(width*height*sizeof(uint8_t));
            uint8_t* right_img_data = (uint8_t*)malloc(width*height*sizeof(uint8_t));

            int32_t k = 0;
            for (int32_t v = 0; v < height; v++) {
                for (int32_t u = 0; u < width; u++) {
                    left_img_data[k] = (uint8_t)frame_calibrated_L.at<uchar>(v, u);
                    right_img_data[k] = (uint8_t)frame_calibrated_R.at<uchar>(v, u);
                    k++;
                }
            }

            // compute visual odometry
            int32_t visoDims[] = {width,height,width};
            if (viso.process(left_img_data,right_img_data,visoDims)) {
                pose = pose * Matrix::inv(viso.getMotion());
                cout << "pose:" << pose << endl;
            } else {
                cout << "Pose Compute failed!" << endl;
            }

            
            // release uint8_t buffers
            std::free(left_img_data);
            std::free(right_img_data);

        #endif
         
            int Key = waitKey(1);
            if (Key == 'q') break;
#ifdef DEBUG_ELAS

            if (Key == 'w')
            {
                disp = disp / 16.0f;
                reprojectImageTo3D(disp, disp_3d, Q, true, -1);
                disp_3d = disp_3d / 1000;     //坐标轴单位转换为米
                FLOAT val_pose[16];
                pose.getData(val_pose);
                //Mat _pose = (Mat_<float>(4, 4) << val[0], val[1], val[2], val[3], val[4], val[5], val[6], val[7], val[8], val[9], val[10], val[11], val[12], val[13], val[14], val[15]);

                int32_t width = disp_3d.cols;
                int32_t height = disp_3d.rows;

                for (size_t r = 0; r < height; r++)
                {
                    for (size_t c = 0; c < width; c++)
                    {
                        if (disp_3d.at<Vec3f>(r, c)[2] != 10) {
                            
                            double val_disp3dw[4];
                            val_disp3dw[0] = (double)disp_3d.at<Vec3f>(r, c)[0];
                            val_disp3dw[1] = (double)disp_3d.at<Vec3f>(r, c)[1];
                            val_disp3dw[2] = (double)disp_3d.at<Vec3f>(r, c)[2];
                            val_disp3dw[3] = 1.0f;
                            Matrix disp_3dw(4, 1, (double*)val_disp3dw);
                            disp_3dw = pose * disp_3dw;
                            disp_3dw.getData((double*)val_disp3dw);
                            /*data[k * 3 + 0] = disp_3d.at<Vec3f>(r, c)[0];
                            data[k * 3 + 1] = -disp_3d.at<Vec3f>(r, c)[1];
                            data[k * 3 + 2] = -disp_3d.at<Vec3f>(r, c)[2];*/

                            data[points * 3 + 0] = (float)val_disp3dw[0];
                            data[points * 3 + 1] = (float)-val_disp3dw[1];
                            data[points * 3 + 2] = (float)-val_disp3dw[2];
                            points++;
                        }
                        if (c == 320 && r == 180)
                            cout << disp_3d.at<Vec3f>(r, c)[0] << "," << disp_3d.at<Vec3f>(r, c)[1] << "," << disp_3d.at<Vec3f>(r, c)[2] << endl;
                    }
                }
                count++;
            }
            if (Key == 'e') {
                //main2(data, points);
            }
#endif // DEBUG_ELAS

        }
        delete[] data;
    }
    Capture.release();
    return 0;
}
