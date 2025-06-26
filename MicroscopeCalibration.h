#ifndef MICROSCOPECALIBRATION
#define MICROSCOPECALIBRATION

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

#define LOCATION "E:/QT/Microscopic_Visual_Servoing/resources/data/"

using namespace cv;
using namespace std;


struct Microscopic_Parameter{
    double c_u;
    double c_v;
    double Z_f;
    double D_f_k_uv;
    Mat Tbc = cv::Mat::eye(4, 4, CV_64F);
};

struct Calibration_Data{
    Mat pointsPuvMoveXD0 = cv::Mat::zeros(2, 0, CV_64F); // 2:   焦平面, 沿x移动采点
    Mat pointsPXYZMoveXD0 = cv::Mat::zeros(3, 0, CV_64F);
    Mat pointsPuvMoveYD0 = cv::Mat::zeros(2, 0, CV_64F);  // 3:   焦平面, 沿y移动采点
    Mat pointsPXYZMoveYD0 = cv::Mat::zeros(3, 0, CV_64F);
    Mat pointsPuvMoveXYDn = cv::Mat::zeros(2, 0, CV_64F); // 4:   Z轴向上移动后，再沿xy移动采点
    Mat pointsPXYZMoveXYDn = cv::Mat::zeros(3, 0, CV_64F);
    Mat pointsPuvMoveXYDp = cv::Mat::zeros(2, 0, CV_64F); // 5:   Z轴向下移动后，再沿xy移动采点
    Mat pointsPXYZMoveXYDp = cv::Mat::zeros(3, 0, CV_64F);
    Mat pointsPuvRotateZD0 = cv::Mat::zeros(2, 0, CV_64F); // 7: 焦平面, 绕Z轴转动并采点
    Mat pointsPXYZRotateZD0 = cv::Mat::zeros(3, 0, CV_64F);
    std::vector<cv::Mat> posesTsRotateZD0;
};
// 1:   找到焦平面
// 2:   沿x移动采点
// 3:   沿y移动采点
// 4:   Z轴向上移动后，再沿xy移动采点
// 5:   Z轴向下移动后，再沿xy移动采点
// 6:   回到焦平面
// 7:   绕Z轴转动并采点
// 8:    计算参数

class MicroscopeCalibration
{
    public:
    Calibration_Data calibrationData;
    Microscopic_Parameter microscopicParameter;

    public:
        MicroscopeCalibration(int resolution_x=1600, int resolution_y=1200);
        void calibration(const Calibration_Data& calibrationData, Microscopic_Parameter& microscopicParameter);
        void writeCalibrationData(const Calibration_Data& calibrationData);
        void write_to_excel(Mat data, ofstream& oFile);
        void writeCalibrationResult(const Microscopic_Parameter& microscopicParameter);
};


#endif
