#ifndef VISUALSERVOINGCONTROLLER_H
#define VISUALSERVOINGCONTROLLER_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QDebug>
#include <QThread>
#include <QMutex>
#include <QVariantMap>
#include <QVariant>
#include <QFile>
#include <QTextStream>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "BaslerCameraControl.h"
#include "ParallelPlatform.h"
#include "direct_microscopic_visual_servoing.h"
#include "MicroscopeCalibration.h"

#define MODE_NULL                                  0
#define MODE_VISUAL_SERVOING       1
#define MODE_SHARPNESS                      2
#define MODE_CALIBRATION                 3

struct VS_Parameter {
    int resolution_x;
    int resolution_y;
    double lambda;
    double epsilon;
    double control_rate;
    camera_intrinsic camera_parameters;
    QString resource_location;
    QString image_desired_name;
    Mat pose_desired;
    Mat Tbc;
    int max_iteration;
    bool is_initialized = false;
};

struct RobotPoses{
        Eigen::VectorXd focusPose = Eigen::VectorXd::Zero(6); // 1:   显微镜标定，找到焦平面
        Eigen::VectorXd workPose = Eigen::VectorXd::Zero(6); // 工作的初始位姿
};

class VisualServoingController: public QObject
{
    Q_OBJECT
public:
    VisualServoingController();
    ~VisualServoingController();

    bool initializeSystem();
    void startServoing();
    void stopServoing();
    bool read_vs_parameter(QString location);
    void output_vs_parameter();
    void setMode(int mode);
    int getMode();
    void visualServoingControl();
    void sharpnessControl();
    void calibrationControl();
    void enableCheckCircle();
    void disableCheckCircle();
    void setCircleDxDyDr(int dx, int dy, int dr);
    void setStepCalibration(int step);
    void enableflagRecord(bool flag);
    void recordPiont(Mat& PuvList, Mat& PxyzList);
    Vec3f checkBestCircle(Mat img);
    void setWorkPose();
    void getRobotPoses(RobotPoses& poses);

signals:
    void systemStatusChanged(const QString& status);
    void servoingError(const QString& error);
    void updateVisualServoingData(const QVariantMap& visData);
    void sigCalibrationImage(QImage img);


public:
    BaslerCameraControl* m_camera = Q_NULLPTR;
    ParallelPlatform *m_robot= Q_NULLPTR;
    Direct_Microscopic_Visual_Servoing* m_algorithm_DMVS;
    VS_Parameter vs_parameter;
    MicroscopeCalibration *m_microscope_calibration;

private:
    QTimer* m_controlTimer;
    bool m_isRunning;
    QElapsedTimer cycleTimer;
    int cnt;
    int circle_du, circle_dv, circle_dr;
    int mode; // 0: Null; 1: Visual servoing; 2: Sharpness; 3: Microscope Calibration
    double sharpness;
    Mat midifyCircle;
    bool flagRecord;
    bool flagCheckCircle;
    RobotPoses robotPoses;
    Vec3f bestCircle;
    int stepCalibration; // 1:   找到焦平面
        // 2:   沿x移动采点
        // 3:   沿y移动采点
        // 4:   Z轴向上移动后，再沿xy移动采点
        // 5:   Z轴向下移动后，再沿xy移动采点
        // 6:   回到焦平面
        // 7:   绕Z轴转动并采点
        // 8:    计算参数
    void executeControlCycle();
};

#endif // VISUALSERVOINGCONTROLLER_H
