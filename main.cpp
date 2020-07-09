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
KeyFrame keyFrame;  //Frame���һ֡ͼ�� ������ͼ�����Mat��pose�Լ�disparity����Ϣ
cv::Mat frameL, frameR, colorL;     //Mat���һ֡ͼ��
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


//��ʾframe�߳���ں���
void _display() {          
	while (true) {
        if (STOP) break;
		capture.displayframe(frameL, frameR);
	}
}

//��ȡframe�߳���ں���
void _read() {              
	while (true) {
        if (STOP) break;
        capture.readFrame2remap(frameL, frameR, colorL);
	}
}

//�Ӿ���̼ƴ�����---------------------�Ѹ���ΪORB_SLAM2
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

//�Ӿ���̼��߳���ں���
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

//��ȼ����߳���ں���
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
    bm->setMinDisparity(0);   //ȷ��ƥ�����������￪ʼ  Ĭ��ֵ��0  
    bm->setNumDisparities(NumDisparities);  //�ڸ���ֵȷ�����ӲΧ�ڽ�������,�Ӳ��  //������Ӳ�ֵ����С�Ӳ�ֵ֮��, ��С������16��������                                 
    bm->setTextureThreshold(TextureThreshold);   //��֤���㹻�������Կ˷�����  //������������ж���ֵ�������ǰSAD�����������ھ����ص��x��������ֵ֮��С��ָ����ֵ��//��ô��ڶ�Ӧ�����ص���Ӳ�ֵΪ 0                          
    bm->setUniquenessRatio(UniquenessRatio);   //ʹ��ƥ�书��ģʽ //�Ӳ�Ψһ�԰ٷֱȣ� �Ӳ�ڷ�Χ����ʹ����Ǵεʹ��۵�//(1 + uniquenessRatio/100)��ʱ����ʹ��۶�Ӧ���Ӳ�ֵ���Ǹ����ص���Ӳ��������ص���Ӳ�Ϊ 0//�ò�������Ϊ��ֵ��һ��5-15���ҵ�ֵ�ȽϺ���
    bm->setSpeckleWindowSize(SpeckleWindowSize);  //����Ӳ���ͨ����仯�ȵĴ��ڴ�С, ֵΪ0ʱȡ�� speckle ���  
    bm->setSpeckleRange(SpeckleRange);  // �Ӳ�仯��ֵ�����������Ӳ�仯������ֵʱ���ô����ڵ��Ӳ����� 

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

        //pose��������cameraŷ���Ǻ�����
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

        //����poses�ж��Ƿ�Ϊ�ظ�֡
        bool isRepetitive = false;
        for (auto i : poses)
        {
            if (i.x - 0.15 < x && x < i.x + 0.15)
                if (i.y - 0.15 < y && y < i.y + 0.15)
                    if (i.z - 0.15 < z && z < i.z + 0.15)
                        if (i.yaw - 40 < yaw && yaw < i.yaw + 40)
                            if (i.pitch - 20 < pitch && pitch < i.pitch + 20)
                            {
                                isRepetitive = true;   //���Ϊ�ظ�֡
                                break;
                            }
        }
        if (isRepetitive == true)  //��⵽�ظ�֡������ѭ������
            continue;


        //��ǰpose����poses
        poses.push_back(Pose(yaw, pitch, roll, x, y, z));

        //�����Ӳ�ͼ
        Mat disp8, nor_disp8;
        Mat disp_3d;
        PointsArray _points;
        bm->compute(_frameL, _frameR, disparity);

        //�Ӳ�ͼת����3D����
        disparity.convertTo(disp8, CV_8U, 255 / ((64 * 16 + 16) * 16.));
        normalize(disp8, nor_disp8, 0, 255, NORM_MINMAX); 
        imshow("disp:", nor_disp8);
        waitKey(1);

        disparity = disparity / 16.0f;

        reprojectImageTo3D(disparity, disp_3d, capture.Q, true, -1);
        disp_3d = disp_3d / 1000;     //�����ᵥλת��Ϊ��

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
        elasParam.support_threshold = 0.9;//0.85���Ψһ�Ա��ʣ����������֧��ƥ�䣩��
        elasParam.support_texture = 100;//10֧�ֵ����С����
        elasParam.candidate_stepsize = 5;//5֧�ֵ㱻ƥ�䵽�ĳ�������Ĳ�����С
        elasParam.incon_window_size = 5;//5��һ��֧�ֵ���Ĵ��ڴ�С
        elasParam.incon_threshold = 3;//5֧�ֵ�Ĳ������ƶ���ֵ����Ϊ��һ�µ�֧�ֵ�
        elasParam.incon_min_support = 5;//5��С��һ��֧�ֵ�����
        elasParam.add_corners = 0;//0��ͼ����Ľ����֧�ֵ㣬����������ڽӵ�Ĳ�ֵ���֧�ֵ㡣
        elasParam.grid_size = 20;//20���ڶ���֧�ֵ����Ƶ������С��
        elasParam.beta = 0.02;//0.02ͼ����Ȼ����
        elasParam.gamma = 3;///3���鳣��
        elasParam.sigma = 1;// 1ǰ��sigma
        elasParam.sradius = 2;// 2ǰ��������뾶
        elasParam.match_texture = 2; //1��С���ܼ�ƥ������
        elasParam.lr_threshold = 2; // 2��������һ���Լ��Ĳ�����ֵ
        elasParam.speckle_sim_threshold = 3;//1�ߵ�ָ�����ƶ���ֵ
        elasParam.speckle_size = 400;//200���İߵ��С��С�İߵ�ᱻ�Ƴ�����
        elasParam.ipol_gap_width = 3;//3�岹С��϶(��<->�ң���<->��)
        elasParam.filter_median = 0;//0��ѡ����λ��������(����ֵ)
        elasParam.filter_adaptive_mean = 1;//1��ѡ������Ӧ��ֵ������(����ֵ)
        elasParam.postprocess_only_left = 0;//1������ȷ��ͼ����к�����Խ�ʡʱ�䡣
        elasParam.subsampling = 0;//0ֻ����ÿ���ڶ����صĲ��죬��ʡʱ�䡣
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

        //pose��������cameraŷ���Ǻ�����
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

        //����poses�ж��Ƿ�Ϊ�ظ�֡
        bool isRepetitive = false;
        for (auto i : poses)
        {
            if (i.x - 0.1 < x && x < i.x + 0.1)
                if (i.y - 0.1 < y && y < i.y + 0.1)
                    if (i.z - 0.1 < z && z < i.z + 0.1)
                        if (i.yaw - 10 < yaw && yaw < i.yaw + 10)
                            if (i.pitch - 10 < pitch && pitch < i.pitch + 10)
                            {
                                isRepetitive = true;   //���Ϊ�ظ�֡
                                break;
                            }
        }
        if (isRepetitive == true)  //��⵽�ظ�֡������ѭ������
            continue;

                                
        //��ǰpose����poses
        poses.push_back(Pose(yaw, pitch, roll, x, y, z));


        //�����Ӳ�ͼ
        Mat* L_image = new Mat(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_32F);
        Mat* R_image = new Mat(Size(FRAME_WIDTH, FRAME_HEIGHT), CV_32F);
        Mat disp_3d;
        elas.process(_frameL.data, _frameR.data, (float*)L_image->data, (float*)R_image->data, dims);

        //����3D����
        reprojectImageTo3D(*L_image, disp_3d, capture.Q, true, -1);
        disp_3d = disp_3d / 1000;     //�����ᵥλת��Ϊ��

        delete L_image;
        delete R_image;

        //���Ƶ��������pose����ת��Ϊ��������
        PointsArray _points;
        for (int r = 96; r < FRAME_HEIGHT - 96; r++) //�ü�����Ե
        {
            for (int c = 54; c < FRAME_WIDTH - 54; c++)
            {
                Vec3f _disp_3d = disp_3d.at<Vec3f>(r, c);  //��ʱ����
                Vec3f p;
                if (_disp_3d[2] != 10 && _disp_3d[2] < 3) {
                    for (int k = 0; k < 3; k++)
                        p[k] = _pose.at<float>(k, 0) * _disp_3d[0] + _pose.at<float>(k, 1) * _disp_3d[1] + _pose.at<float>(k, 2) * _disp_3d[2] + _pose.at<float>(k, 3);  //����������pose�������
                    
                    _points.data.push_back((float)p[0]);  //��������pushback��data��
                    _points.data.push_back((float)-p[1]);
                    _points.data.push_back((float)-p[2]);
                    
                    _points.data.push_back((float)_colorL.at<Vec3b>(r, c)[2] / 255.0f);  //���ص���ɫpushback��data��
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

	//init viso---------------�Ѹ���ΪORB_SLAM2
	//VisualOdometryStereo::parameters visualOdometryParam;
	//visualOdometryParam.calib.f = 367.4534; // focal length in pixels
	//visualOdometryParam.calib.cu = 308.0705; // principal point (u-coordinate) in pixels
	//visualOdometryParam.calib.cv = 170.8232; // principal point (v-coordinate) in pixels
	//visualOdometryParam.base = 0.0588364; // baseline in meters   //���ߣ� ��������ͷ���ĵľ���
	//VisualOdometryStereo viso(visualOdometryParam);

    capture.readFrame2remap(frameL, frameR, colorL);  //�ȶ���һ֡ͼ�� ȷ�������̵߳����ݲ�Ϊ��

	thread read(_read); //����ͷ��ȡ�߳�
	thread display(_display);//����Ԥ���߳�
    thread visoProcess(_slam_vo);//�Ӿ���̼��߳�
    thread dispthread(_elas);//��ȼ���to3D�߳�

    //rander loop
    view3d.draw();

	display.join();
    read.join();
    visoProcess.join();
    dispthread.join();

}


