#ifndef PARALLELPLATFORM_H
#define PARALLELPLATFORM_H

#include <QObject>
#include <QTimer>
#include <QDebug>
#include <Eigen/Dense>
#include <QTimer>
#include <QThread>
#include <opencv2/opencv.hpp>

#include "NarpodControl.h"

using namespace Eigen;

class ParallelPlatform:  public QObject
{
    Q_OBJECT // 必须包含这个宏

public:
    ParallelPlatform(double hz);
    ~ParallelPlatform();

public:
    /// 初始化函数
    bool connect();
    bool disconnect();
    bool startupdate();
    bool stopupdate();
    bool findReferance();   // 使得各个关节轴回到标志位

    /// 功能性函数
    /// NTS那边给的库函数
    bool stop();
    // 单位nm
    bool moveAbsolute(signed int x,signed int y,signed int z,signed int rx,signed int ry,signed int rz);
    /// 自己通过运算写的函数
    //  输入矩阵T移动,自动转化为需求的向量
    bool moveAbsolute(Eigen::Matrix4d T);
    // 给定Δt和速度方向,通过位置控制移动到指定位置
    bool moveVelocity(Eigen::VectorXd VelocityList_b,double dt);

    /// 状态读取函数
    // bool getSpeed();
    bool getJointPositions(Eigen::VectorXd& positions);
    bool getTaskPositions(Eigen::VectorXd& positions);
    bool getTargetVelocity(Eigen::VectorXd& targetVelocity);
    bool getTaskMat(Eigen::Matrix4d& T);
    cv::Mat getTaskMat_cv();
    bool checkresult();// 用于判断函数释放顺利执行

    // 检查是否能够到达
    bool checkPoseReachable(int x,int y,int z,int rx,int ry,int rz);
    Eigen::Matrix4d getT(Eigen::VectorXd Tlist);
    Eigen::VectorXd getList(Eigen::Matrix4d T);
    bool setTargetVelocity(Eigen::VectorXd V);
    bool setTargetPose(Eigen::VectorXd P);

signals:
        void sig_updateRobotData(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose);

private:
    void controller();
    void update();  // 上传x，y，z，rx，ry，rz的位置

public:
    int controlMode_; // 0: velocity; 1: position; 2: stop; 3: moveToZero
    /// 辅助计算类函数

private:
    unsigned int ntHandle_;
    NARPOD_STATUS result_ =  NARPOD_OK;
    Eigen::Matrix4d b_T_e_;                      //  矩阵 b_T_e 当前位姿
    Eigen::VectorXd current_end_effector_pose_; //  向量 [x;y;z;rx;ry;rz]
    // 以下变量储存轴的
    int positionx_;
    int positiony_;
    int positionz_;
    int rotationx_;
    int rotationy_;
    int rotationz_;
    Eigen::VectorXd currentPose_Vec_; //  向量 [x;y;z;rx;ry;rz],记录当前的指令的目标位置,注意：实际是编码器读数
    Eigen::Matrix4d currentPose_Mat_; //  矩阵 T,记录当前的指令的目标位置
    Eigen::VectorXd targetVelocity_;
    Eigen::VectorXd targetPose_; //  向量 [x;y;z;rx;ry;rz]
    double epsilon_w_;
    double epsilon_v_;
    double dt_;
    QTimer *m_preciseTimer_;
    QElapsedTimer cycleTimer;

/// 任务步骤
    // STEP1 读取位姿函数完成并且调试完毕  ok！
    // STEP2 位姿计算函数调试完成   确定RXYZ 还是RZYX ok！
    // 1、写一个RXYZ为1，2，3的矩阵
    // 2、带入官方程序运行读取
    // STEP3 添加记录目标位置的函数 ok!
    // STEP4 老师需求的函数调试完成
    // 1、完成函数后，使用c++进行读取测试
    // 2、等待测试完成后，往Qt中写，并完成调试
};

#endif // ParallelPlatform_H
