#include "ORB_SLAM2_VO.h"
#include "Frame.h"
#include "Capture.h"
#include <thread>
#include "viso_stereo.h"
#include "view3D.h"
#include <mutex>
#include "elas.h"

using namespace cv;

Capture capture;
KeyFrame keyFrame;  //Frame类的一帧图像 包含了图像本身的Mat和pose以及disparity的信息
cv::Mat frameL, frameR, colorL;     //Mat类的一帧图像。
Matrix poseMatrix = Matrix::eye(4);
cv::Mat disparity;
SLAM slam("ORBvoc.txt", "stereoCamera.yaml");
View3D view3d;

vector<PointsArray> points;

std::mutex keyFrameMutex;
std::mutex frameMutex;
std::mutex pointsMutex;

// Tracking states
enum eTrackingState {
    SYSTEM_NOT_READY = -1,
    NO_IMAGES_YET = 0,
    NOT_INITIALIZED = 1,
    OK = 2,
    LOST = 3
};


struct Pose
{
    Pose(float _yaw, float _pitch, float _roll, float _x, float _y, float _z)
    {
        yaw = _yaw;
        pitch = _pitch;
        roll = _roll;
        x = _x;
        y = _y;
        z = _z;
    }
    float yaw, pitch, roll, x, y, z;
};
vector<Pose> poses;

bool STOP = false;
bool PAUSE = true;


//显示frame线程入口函数
void _display() {          
	while (true) {
        if (STOP) break;
		capture.displayframe(frameL, frameR);
	}
}

//读取frame线程入口函数
void _read() {              
	while (true) {
        if (STOP) break;
        capture.readFrame2remap(frameL, frameR, colorL);
	}
}

//视觉里程计处理函数---------------------已更换为ORB_SLAM2
//int visoProcess(VisualOdometryStereo &viso)
//{
//    static int count = 0;
//    // convert input images to uint8_t buffer
//    int32_t height = FRAME_HEIGHT;
//    int32_t width = FRAME_WIDTH;
//    uint8_t* left_img_data = (uint8_t*)malloc(width * height * sizeof(uint8_t));
//    uint8_t* right_img_data = (uint8_t*)malloc(width * height * sizeof(uint8_t));
//
//    Mat _frameL, _frameR;
//
//    frameMutex.lock();
//    frameL.copyTo(_frameL);
//    frameR.copyTo(_frameR);
//    frameMutex.unlock();
//
//    int32_t k = 0;
//    {
//        for (int32_t v = 0; v < height; v++) {
//            for (int32_t u = 0; u < width; u++) {
//                left_img_data[k] = (uint8_t)_frameL.at<uchar>(v, u);
//                right_img_data[k] = (uint8_t)_frameR.at<uchar>(v, u);
//                k++;
//            }
//        }
//    }
//    // compute visual odometry
//    int32_t visoDims[] = { width,height,width };
//
//    if (viso.process(right_img_data, left_img_data, visoDims)) {
//        poseMatrix = poseMatrix * Matrix::inv(viso.getMotion());
//
//        keyFrameMutex.lock();
//        _frameL.copyTo(frame.frameL);
//        _frameR.copyTo(frame.frameR);
//        colorL.copyTo(frame.colorL);
//        frame.poseMatrix = poseMatrix;
//        keyFrameMutex.unlock();
//
//        cout << "poseMatrix:" << poseMatrix << endl;
//    }
//    else {
//        cout << "Pose Compute failed!" << endl;
//        count++;
//        if (count > 1) {
//            return -1;
//        }
//    }
//
//    // release uint8_t buffers
//    std::free(left_img_data);
//    std::free(right_img_data);
//    return 0;
//}  

//视觉里程计线程入口函数
//void _viso(VisualOdometryStereo viso) {
//    while (true) {
//        if (STOP) break;
//        //if (visoProcess(viso) == -1) VISOfailed = true;
//
//    }
//    
//}

void _slam_vo() {
    while (true) {
        if (STOP) break;
        Mat _frameL, _frameR, _colorL;

        frameMutex.lock();
        _frameL = frameL.clone();
        _frameR = frameR.clone();
        _colorL = colorL.clone();
        frameMutex.unlock();
        
        Mat _pose = slam.track(_frameL, _frameR);
        
        cout << "POSE:" << endl;
        cout << _pose.at<float>(0, 0) <<
            _pose.at<float>(1, 0) <<
            _pose.at<float>(2, 0) <<
            _pose.at<float>(3, 0) << endl;

        cout << _pose.at<float>(0, 1) <<
            _pose.at<float>(1, 1) <<
            _pose.at<float>(2, 1) <<
            _pose.at<float>(3, 1) << endl;

        cout << _pose.at<float>(0, 2) <<
            _pose.at<float>(1, 2) <<
            _pose.at<float>(2, 2) <<
            _pose.at<float>(3, 2) << endl;

        cout << _pose.at<float>(0, 3) <<
            _pose.at<float>(1, 3) <<
            _pose.at<float>(2, 3) <<
            _pose.at<float>(3, 3) << endl;
        cout << "--------------" << endl;

        keyFrameMutex.lock();
        _frameL.copyTo(keyFrame.frameL);
        _frameR.copyTo(keyFrame.frameR);
        _colorL.copyTo(keyFrame.colorL);
        _pose.copyTo(keyFrame.poseMatrix);

        keyFrameMutex.unlock();


    }
    slam.shoutdown();
}

//深度计算线程入口函数
void _bm() {
    //init stereoBM
    cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 21);
    int BlockSize = 9,
        NumDisparities = 64,
        TextureThreshold = 100,
        UniquenessRatio = 20,
        SpeckleRange = 16,
        SpeckleWindowSize = 200,
        PreFilterType = 0,
        PreFilterSize = 15,
        PreFilterCap = 31;
    bm->setPreFilterType(PreFilterType);
    bm->setPreFilterSize(PreFilterSize);
    bm->setPreFilterCap(PreFilterCap);
    bm->setDisp12MaxDiff(-1);
    bm->setBlockSize(BlockSize);
    bm->setMinDisparity(0);   //确定匹配搜索从哪里开始  默认值是0  
    bm->setNumDisparities(NumDisparities);  //在该数值确定的视差范围内进行搜索,视差窗口  //即最大视差值与最小视差值之差, 大小必须是16的整数倍                                 
    bm->setTextureThreshold(TextureThreshold);   //保证有足够的纹理以克服噪声  //低纹理区域的判断阈值。如果当前SAD窗口内所有邻居像素点的x导数绝对值之和小于指定阈值，//则该窗口对应的像素点的视差值为 0                          
    bm->setUniquenessRatio(UniquenessRatio);   //使用匹配功能模式 //视差唯一性百分比， 视差窗口范围内最低代价是次低代价的//(1 + uniquenessRatio/100)倍时，最低代价对应的视差值才是该像素点的视差，否则该像素点的视差为 0//该参数不能为负值，一般5-15左右的值比较合适
    bm->setSpeckleWindowSize(SpeckleWindowSize);  //检查视差连通区域变化度的窗口大小, 值为0时取消 speckle 检查  
    bm->setSpeckleRange(SpeckleRange);  // 视差变化阈值，当窗口内视差变化大于阈值时，该窗口内的视差清零 

    while (true) {
        if (STOP) break;
        
        Mat _frameL, _frameR, _colorL,_pose;
        

        keyFrameMutex.lock();
        if (keyFrame.poseMatrix.empty())
        {
            keyFrameMutex.unlock();
            continue;
        }
        
        keyFrame.frameL.copyTo(_frameL);
        keyFrame.frameR.copyTo(_frameR);
        keyFrame.colorL.copyTo(_colorL);
        keyFrame.poseMatrix.copyTo(_pose);
        keyFrameMutex.unlock();
        
        //Get Current OpenGL Camera Matrix
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);

        Rwc = _pose.rowRange(0, 3).colRange(0, 3).t();
        twc = -Rwc * _pose.rowRange(0, 3).col(3);

        _pose.at<float>(0, 0) = Rwc.at<float>(0, 0);
        _pose.at<float>(1, 0) = Rwc.at<float>(1, 0);
        _pose.at<float>(2, 0) = Rwc.at<float>(2, 0);
        _pose.at<float>(3, 0) = 0.0;

        _pose.at<float>(0, 1) = Rwc.at<float>(0, 1);
        _pose.at<float>(1, 1) = Rwc.at<float>(1, 1);
        _pose.at<float>(2, 1) = Rwc.at<float>(2, 1);
        _pose.at<float>(3, 1) = 0.0;

        _pose.at<float>(0, 2) = Rwc.at<float>(0, 2);
        _pose.at<float>(1, 2) = Rwc.at<float>(1, 2);
        _pose.at<float>(2, 2) = Rwc.at<float>(2, 2);
        _pose.at<float>(3, 2) = 0.0;

        _pose.at<float>(0, 3) = twc.at<float>(0);
        _pose.at<float>(1, 3) = twc.at<float>(1);
        _pose.at<float>(2, 3) = twc.at<float>(2);
        _pose.at<float>(3, 3) = 1.0;

        //pose矩阵解算出camera欧拉角和坐标
        float r11 = _pose.at<float>(0, 0);
        float r21 = _pose.at<float>(1, 0);
        float r31 = _pose.at<float>(2, 0);
        float r32 = _pose.at<float>(2, 1);
        float r33 = _pose.at<float>(2, 2);

        float roll = atan2(r21, r11) / PI * 180;
        float yaw = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / PI * 180;
        float pitch = atan2(r32, r33) / PI * 180;
        float x = twc.at<float>(0);
        float y = twc.at<float>(1);
        float z = twc.at<float>(2);

        //遍历poses判断是否为重复帧
        bool isRepetitive = false;
        for (auto i : poses)
        {
            if (i.x - 0.15 < x && x < i.x + 0.15)
                if (i.y - 0.15 < y && y < i.y + 0.15)
                    if (i.z - 0.15 < z && z < i.z + 0.15)
                        if (i.yaw - 40 < yaw && yaw < i.yaw + 40)
                            if (i.pitch - 20 < pitch && pitch < i.pitch + 20)
                            {
                                isRepetitive = true;   //标记为重复帧
                                break;
                            }
        }
        if (isRepetitive == true)  //检测到重复帧，返回循环顶部
            continue;


        //当前pose加入poses
        poses.push_back(Pose(yaw, pitch, roll, x, y, z));

        //计算视差图
        Mat disp8, nor_disp8;
        Mat disp_3d;
        PointsArray _points;
        bm->compute(_frameL, _frameR, disparity);

        //视差图转换到3D坐标
        disparity.convertTo(disp8, CV_8U, 255 / ((64 * 16 + 16) * 16.));
        normalize(disp8, nor_disp8, 0, 255, NORM_MINMAX); 
        imshow("disp:", nor_disp8);
        waitKey(1);

        disparity = disparity / 16.0f;

        reprojectImageTo3D(disparity, disp_3d, capture.Q, true, -1);
        disp_3d = disp_3d / 1000;     //坐标轴单位转换为米

        for (int r = 0; r < FRAME_HEIGHT; r++)
        {
            for (int c = 0; c < FRAME_WIDTH; c++)
            {
                Vec3f _disp_3d = disp_3d.at<Vec3f>(r, c);
                Vec3f p;
                if (_disp_3d[2] != 10) {
                    for (int k = 0; k < 3; k++)
                        p[k] = _pose.at<float>(k, 0) * _disp_3d[0] + _pose.at<float>(k, 1) * _disp_3d[1] + _pose.at<float>(k, 2) * _disp_3d[2] + _pose.at<float>(k, 3);

                    _points.data.push_back((float)p[0]);
                    _points.data.push_back((float)-p[1]);
                    _points.data.push_back((float)-p[2]);
                    
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[2] / 255.0f);
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[1] / 255.0f);
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[0] / 255.0f);
                    _points.quantity++;
                }
            }
        }
        std::lock_guard<std::mutex> lockGuard(pointsMutex);
        points.push_back(_points);
    }  
}

void _elas() {
    const int32_t dims[3] = { FRAME_WIDTH,FRAME_HEIGHT,FRAME_WIDTH };
    Elas::parameters elasParam(Elas::ROBOTICS);
    {
        elasParam.disp_min = 0;
        elasParam.disp_max = 255;
        elasParam.support_threshold = 0.9;//0.85最大唯一性比率（最佳与次最佳支持匹配）。
        elasParam.support_texture = 100;//10支持点的最小纹理
        elasParam.candidate_stepsize = 5;//5支持点被匹配到的常规网格的步数大小
        elasParam.incon_window_size = 5;//5不一致支持点检查的窗口大小
        elasParam.incon_threshold = 3;//5支持点的差异相似度阈值被认为是一致的支持点
        elasParam.incon_min_support = 5;//5最小的一致支持点数量
        elasParam.add_corners = 0;//0在图像的四角添加支持点，并以最近的邻接点的差值添加支持点。
        elasParam.grid_size = 20;//20用于额外支持点外推的邻域大小。
        elasParam.beta = 0.02;//0.02图像似然参数
        elasParam.gamma = 3;///3先验常数
        elasParam.sigma = 1;// 1前期sigma
        elasParam.sradius = 2;// 2前期西格玛半径
        elasParam.match_texture = 2; //1最小的密集匹配纹理。
        elasParam.lr_threshold = 2; // 2用于左右一致性检查的差异阈值
        elasParam.speckle_sim_threshold = 3;//1斑点分割的相似度阈值
        elasParam.speckle_size = 400;//200最大的斑点大小（小的斑点会被移除）。
        elasParam.ipol_gap_width = 3;//3插补小间隙(左<->右，上<->下)
        elasParam.filter_median = 0;//0可选的中位数过滤器(近似值)
        elasParam.filter_adaptive_mean = 1;//1可选的自适应均值过滤器(近似值)
        elasParam.postprocess_only_left = 0;//1不对正确的图像进行后处理可以节省时间。
        elasParam.subsampling = 0;//0只计算每个第二像素的差异，节省时间。
    }

    Elas elas(elasParam);
    while (true)
    {
        if (STOP) break;
        if (PAUSE) continue;
        keyFrameMutex.lock();
        if (keyFrame.poseMatrix.empty())
        {
            keyFrameMutex.unlock();
            continue;
        }
        Mat _frameL, _frameR, _colorL, _pose;
        keyFrame.poseMatrix.copyTo(_pose);
        keyFrame.frameL.copyTo(_frameL);
        keyFrame.frameR.copyTo(_frameR);
        keyFrame.colorL.copyTo(_colorL);
        keyFrameMutex.unlock();

        //Get Current OpenGL Camera Matrix
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);

        Rwc = _pose.rowRange(0, 3).colRange(0, 3).t();
        twc = -Rwc * _pose.rowRange(0, 3).col(3);

        _pose.at<float>(0, 0) = Rwc.at<float>(0, 0);
        _pose.at<float>(1, 0) = Rwc.at<float>(1, 0);
        _pose.at<float>(2, 0) = Rwc.at<float>(2, 0);
        _pose.at<float>(3, 0) = 0.0;

        _pose.at<float>(0, 1) = Rwc.at<float>(0, 1);
        _pose.at<float>(1, 1) = Rwc.at<float>(1, 1);
        _pose.at<float>(2, 1) = Rwc.at<float>(2, 1);
        _pose.at<float>(3, 1) = 0.0;

        _pose.at<float>(0, 2) = Rwc.at<float>(0, 2);
        _pose.at<float>(1, 2) = Rwc.at<float>(1, 2);
        _pose.at<float>(2, 2) = Rwc.at<float>(2, 2);
        _pose.at<float>(3, 2) = 0.0;

        _pose.at<float>(0, 3) = twc.at<float>(0);
        _pose.at<float>(1, 3) = twc.at<float>(1);
        _pose.at<float>(2, 3) = twc.at<float>(2);
        _pose.at<float>(3, 3) = 1.0;

        //pose矩阵解算出camera欧拉角和坐标
        float r11 = _pose.at<float>(0, 0);
        float r21 = _pose.at<float>(1, 0);
        float r31 = _pose.at<float>(2, 0);
        float r32 = _pose.at<float>(2, 1);
        float r33 = _pose.at<float>(2, 2);

        float roll = atan2(r21, r11) / PI * 180;
        float yaw = atan2(-1 * r31, sqrt(r32 * r32 + r33 * r33)) / PI * 180;
        float pitch = atan2(r32, r33) / PI * 180;
        float x = twc.at<float>(0);
        float y = twc.at<float>(1);
        float z = twc.at<float>(2);

        //遍历poses判断是否为重复帧
        bool isRepetitive = false;
        for (auto i : poses)
        {
            if (i.x - 0.1 < x && x < i.x + 0.1)
                if (i.y - 0.1 < y && y < i.y + 0.1)
                    if (i.z - 0.1 < z && z < i.z + 0.1)
                        if (i.yaw - 10 < yaw && yaw < i.yaw + 10)
                            if (i.pitch - 10 < pitch && pitch < i.pitch + 10)
                            {
                                isRepetitive = true;   //标记为重复帧
                                break;
                            }
        }
        if (isRepetitive == true)  //检测到重复帧，返回循环顶部
            continue;

                                
        //当前pose加入poses
        poses.push_back(Pose(yaw, pitch, roll, x, y, z));


        //计算视差图
        Mat* L_image = new Mat(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_32F);
        Mat* R_image = new Mat(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_32F);
        Mat disp_3d;
        elas.process(_frameL.data, _frameR.data, (float*)L_image->data, (float*)R_image->data, dims);

        //计算3D坐标
        reprojectImageTo3D(*L_image, disp_3d, capture.Q, true, -1);
        disp_3d = disp_3d / 1000;     //坐标轴单位转换为米

        delete L_image;
        delete R_image;

        //点云的坐标乘以pose矩阵转换为世界坐标
        PointsArray _points;
        for (int r = 96; r < FRAME_HEIGHT - 96; r++) //裁减掉边缘
        {
            for (int c = 54; c < FRAME_WIDTH - 54; c++)
            {
                Vec3f _disp_3d = disp_3d.at<Vec3f>(r, c);  //临时变量
                Vec3f p;
                if (_disp_3d[2] != 10 && _disp_3d[2] < 3) {
                    for (int k = 0; k < 3; k++)
                        p[k] = _pose.at<float>(k, 0) * _disp_3d[0] + _pose.at<float>(k, 1) * _disp_3d[1] + _pose.at<float>(k, 2) * _disp_3d[2] + _pose.at<float>(k, 3);  //单个坐标与pose矩阵相乘
                    
                    _points.data.push_back((float)p[0]);  //世界坐标pushback到data中
                    _points.data.push_back((float)-p[1]);
                    _points.data.push_back((float)-p[2]);
                    
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[2] / 255.0f);  //像素点颜色pushback到data中
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[1] / 255.0f);
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[0] / 255.0f);
                    _points.quantity++;
                }
            }
        }
        std::lock_guard<std::mutex> lockGuard(pointsMutex);
        points.push_back(_points);
        
    }
}

int main() {

	capture.initRectifyMap();

	//init viso---------------已更换为ORB_SLAM2
	//VisualOdometryStereo::parameters visualOdometryParam;
	//visualOdometryParam.calib.f = 367.4534; // focal length in pixels
	//visualOdometryParam.calib.cu = 308.0705; // principal point (u-coordinate) in pixels
	//visualOdometryParam.calib.cv = 170.8232; // principal point (v-coordinate) in pixels
	//visualOdometryParam.base = 0.0588364; // baseline in meters   //基线， 两个摄像头光心的距离
	//VisualOdometryStereo viso(visualOdometryParam);

    capture.readFrame2remap(frameL, frameR, colorL);  //先读入一帧图像 确保其他线程的数据不为空

	thread read(_read); //摄像头读取线程
	thread display(_display);//画面预览线程
    thread visoProcess(_slam_vo);//视觉里程计线程
    thread dispthread(_elas);//深度计算to3D线程

    //rander loop
    view3d.draw();

	display.join();
    read.join();
    visoProcess.join();
    dispthread.join();

}


