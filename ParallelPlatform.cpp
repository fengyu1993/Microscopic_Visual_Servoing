#include <iostream>
#include <sstream>
#include "ParallelPlatform.h"
#include "modern_robotic_lib.h"

ParallelPlatform::ParallelPlatform(double hz)
{
    // ID_ = "usb:id:1234567895";
    this->dt_ = 1 / hz;
    current_end_effector_pose_ = Eigen::VectorXd::Zero(6);
    this->b_T_e_= Eigen::Matrix4d::Identity();

    this->m_preciseTimer_ = new QTimer();
    this->m_preciseTimer_->setInterval(this->dt_*1000);
    this-> m_preciseTimer_->setTimerType(Qt::PreciseTimer);
    QObject::connect(this->m_preciseTimer_, &QTimer::timeout, this, &ParallelPlatform::controller);

    this->positionx_ = 0;
    this->positiony_ = 0;
    this-> positionz_ = 0;
    this-> rotationx_ = 0;
    this->rotationy_ = 0;
    this->rotationz_ = 0;
    this->currentPose_Vec_ = Eigen::VectorXd::Zero(6);
    this->currentPose_Mat_=Eigen::Matrix4d::Identity();
    this->targetVelocity_ = Eigen::VectorXd::Zero(6);
    this->targetPose_ = Eigen::VectorXd::Zero(6);
    this->epsilon_v_ = 0.01;
    this->epsilon_w_ = 0.01;
    this->controlMode_ = 0;
}

ParallelPlatform::~ParallelPlatform()
{
    stop();
    disconnect();
    delete m_preciseTimer_;
}

void ParallelPlatform::update()
{
    // 关于x y z rx ry rz的更新
    result_ = Narpod_GetPosition(ntHandle_,&positionx_,&positiony_,&positionz_,&rotationx_,&rotationy_,&rotationz_);

    // qDebug() << "ntHandle_:" << ntHandle_;

    Eigen::VectorXd Tlist(6);
    Tlist(0) = positionx_; Tlist(1) = positiony_; Tlist(2) = positionz_;
    Tlist(3) = rotationx_; Tlist(4) = rotationy_; Tlist(5) = rotationz_;
    b_T_e_ = currentPose_Mat_;

    emit sig_updateRobotData(Tlist, targetVelocity_, currentPose_Vec_);
    // std::stringstream ss1;
    // ss1 << b_T_e_;
    // qDebug() << "b_T_e_:\n" << ss1.str().c_str();
    // std::stringstream ss2;
    // ss2 << Tlist;
    // qDebug() << "Tlist:\n" << ss2.str().c_str();
}

void ParallelPlatform::controller()
{
    // qDebug() << "Robot thread ID:" << QThread::currentThreadId();
    // qDebug() << "elapsed: " << cycleTimer.elapsed();
    // qDebug() << "controlMode: " << this->controlMode_;

    update();

    if(this->controlMode_ == 0){
        targetPose_ = currentPose_Vec_;
        if (targetVelocity_.head(3).norm() <   epsilon_v_ && targetVelocity_.tail(3).norm()  < epsilon_w_)
        {
            targetVelocity_ = Eigen::VectorXd::Zero(6);
        }
        moveVelocity(this->targetVelocity_, this->dt_);

            // std::stringstream ss;
            // ss << this->targetVelocity_;
            // qDebug() << "targetVelocity:\n" << ss.str().c_str();
    }else if(this->controlMode_ == 1){
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
        Eigen::VectorXd errorPose = currentPose_Vec_ - targetPose_;
        if(errorPose.segment(0, 3).norm()  >=  epsilon_v_ || errorPose.segment(3, 3).norm()  >= epsilon_w_){
            this->moveAbsolute(targetPose_(0),targetPose_(1),targetPose_(2),targetPose_(3),targetPose_(4),targetPose_(5));
        }
    }
    else if(this->controlMode_ == 2){
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
        moveVelocity(this->targetVelocity_, this->dt_);
    }
    else if(this->controlMode_ == 3){
        this->targetVelocity_ = Eigen::VectorXd::Zero(6);
        moveAbsolute(0,0,0,0,0,0);
    }

}

bool ParallelPlatform::connect()
{
    // 先调用NTS startactivate才能运动
    // result_ = Narpod_Open(&ntHandle_, "usb:id:9876543210", "1");
    result_ = Narpod_Open(&ntHandle_, "usb:ix:0", "1");
    result_ = Narpod_Set_Property(ntHandle_, HardwareModel, MODEL_HEXAPOD70U);
    result_ = Narpod_Set_Property(ntHandle_, PIVOT_MODE, PIVOT_FIXED);
    // result_ = Narpod_Set_Property(ntHandle_, PIVOT_MODE, PIVOT_RELATIVE);
    result_ = Narpod_SetPivot(ntHandle_, 0, 0, 0);
    result_ = Narpod_SetCoordinateSystem(ntHandle_,0,0,0,0,0,0);
    result_ = Narpod_FindReferenceMarks(ntHandle_);
    bool res = checkresult();
    if (res)
    {
        qDebug()<<"ParallelPlatform::connect()运行成功";
    }
    else
    {
        qDebug()<<"ParallelPlatform::connect()运行失败";
    }
    return res;
}

bool ParallelPlatform::disconnect()
{
    result_ = Narpod_Close(ntHandle_);
    bool res = checkresult();
    if (res)
    {
        qDebug()<<"ParallelPlatform::disconnect()运行成功";
    }
    else
    {
        qDebug()<<"ParallelPlatform::disconnect()运行失败";
    }
    return res;
}

bool ParallelPlatform::findReferance()
{
    result_ = NARPOD_OTHER_ERROR;
    qDebug() << "findReferance start: " << result_;
    result_ = Narpod_FindReferenceMarks(ntHandle_);
    qDebug() << "findReferance end: "  << result_;
    bool res = checkresult();
    if (res)
    {
        positionx_ = 0;
        positiony_ = 0;
        positionz_ = 0;
        rotationx_ = 0;
        rotationy_ = 0;
        rotationz_ = 0;
        currentPose_Vec_ = Eigen::VectorXd::Zero(6);
        currentPose_Mat_=Eigen::Matrix4d::Identity();
        current_end_effector_pose_ = Eigen::VectorXd::Zero(6);
        this->b_T_e_= Eigen::Matrix4d::Identity();
        qDebug()<<"ParallelPlatform::findreferance()运行成功";
    }
    else
    {
        qDebug()<<"ParallelPlatform::findreferance()运行失败";
    }
    return res;
}


bool ParallelPlatform::startupdate()
{
    this->m_preciseTimer_->start();
    return 1;
}


bool ParallelPlatform::stopupdate()
{
    this->m_preciseTimer_->stop();
    return 1;
}

/// 功能性函数
/// NTS那边给的库函数
bool ParallelPlatform::stop()
{
    result_ = Narpod_Stop(ntHandle_);
    this->targetVelocity_ = Eigen::VectorXd::Zero(6);
    bool res = checkresult();
    if (res)
    {
        qDebug()<<"ParallelPlatform::stop运行成功";
    }
    else
    {
        qDebug()<<"ParallelPlatform::stop运行失败";
    }
    return res;
}


bool ParallelPlatform::moveAbsolute(signed int x,signed int y,signed int z,
                                    signed int rx,signed int ry,signed int rz)
{
    // result_ = Narpod_Move(ntHandle_,x,y,z,rx,ry,rz);
    // return 1;
    // bool flag = checkPoseReachable(x,y,z,rx,ry,rz);
    // if (!flag)
    // {
    //     qDebug()<<"ParallelPlatform::moveAbsolute输入超限";
    //     return flag;
    // }
    // else
    // {

        result_ = Narpod_Move(ntHandle_,x,y,z,rx,ry,rz);
        result_ = Narpod_GetPosition(ntHandle_,&positionx_,&positiony_,&positionz_,&rotationx_,&rotationy_,&rotationz_);

        currentPose_Vec_(0)=x;currentPose_Vec_(1)=y;currentPose_Vec_(2)=z;
        currentPose_Vec_(3)=rx;currentPose_Vec_(4)=ry;currentPose_Vec_(5)=rz;
        currentPose_Mat_ = getT(currentPose_Vec_);
        bool res = checkresult();
        // if (res)
        // {
        //     qDebug()<<"ParallelPlatform::moveAbsolute运行成功";
        // }
        // else
        // {
        //     qDebug()<<"ParallelPlatform::moveAbsolute运行失败";
        // }
        return res;
    // }
}

bool ParallelPlatform::moveAbsolute(Eigen::Matrix4d T)
{
    Eigen::VectorXd target_list = getList(T);
    bool res = moveAbsolute(target_list(0),target_list(1),target_list(2),target_list(3),target_list(4),target_list(5));
    // if (res)
    // {
    //     qDebug()<<"ParallelPlatform::moveAbsolute(Eigen::Matrix4d T)运行成功";
    // }
    // else
    // {
    //     qDebug()<<"ParallelPlatform::moveAbsolute(Eigen::Matrix4d T)运行失败";
    // }
    return res;
}

/// 辅助计算类函数
bool ParallelPlatform::checkresult()
{
    switch(result_)
    {
    case  NARPOD_OK:
        return 1;
    default:
        qDebug()<<"该函数运行错误";
        return 0;
    }

}

bool ParallelPlatform::checkPoseReachable(int x,int y,int z,int rx,int ry,int rz)
{
    unsigned int reachable;
    result_ = Narpod_IsPoseReachable(ntHandle_,x,y,z,rx,ry,rz,&reachable);
    if(reachable == NARPOD_TRUE)
    {
        // qDebug()<<"checkPoseReachable，能达到指定位置";
        return 1;
    }
    else
    {
        qDebug()<<"checkPoseReachable，不能达到指定位置";
        return 0;
    }
}



bool ParallelPlatform::getJointPositions(Eigen::VectorXd& positions)
{
    bool result = Narpod_GetPosition(ntHandle_,&positionx_,&positiony_,&positionz_,&rotationx_,&rotationy_,&rotationz_);
    positions = Eigen::VectorXd::Zero(6);
    positions(0) = positionx_;positions(1) = positiony_;positions(2) = positionz_;
    positions(3) = rotationx_;positions(4) = rotationy_;positions(5) = rotationz_;
    return result;
}

bool ParallelPlatform::getTaskPositions(Eigen::VectorXd& positions)
{
    positions = Eigen::VectorXd::Zero(6);
    positions = currentPose_Vec_;
    return 1;
}

bool ParallelPlatform::getTargetVelocity(Eigen::VectorXd& targetVelocity)
{
    targetVelocity = targetVelocity_;
    return 1;
}

bool ParallelPlatform::getTaskMat(Eigen::Matrix4d& T)
{
    // result_ = Narpod_GetPosition(ntHandle_,&positionx_,&positiony_,&positionz_,&rotationx_,&rotationy_,&rotationz_);
    // Eigen::VectorXd Tlist(6);
    // Tlist << positionx_, positiony_, positiony_, positionz_, rotationx_, rotationy_, rotationz_;
    // std::cout << Tlist << std::endl;
    // T = getT(Tlist);
    T = currentPose_Mat_;
    return 1;
}

cv::Mat ParallelPlatform::getTaskMat_cv()
{
    cv::Mat T(4, 4, CV_64F);
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            T.at<double>(i, j) = currentPose_Mat_(i, j);
        }
    }
    return T.clone();
}

Eigen::Matrix4d ParallelPlatform::getT(Eigen::VectorXd Tlist)
{
    // STEP1 提取Tlist中的xyz，rxyz
    // 单位：xyz，rxyz为nm和u°
    double x = Tlist(0);double y = Tlist(1);double z = Tlist(2);
    double rx = Tlist(3)/1000000;double ry = Tlist(4)/1000000;double rz = Tlist(5)/1000000;
    double H = 0;// 机器人高度
    // STEP2 计算平移部分
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    Eigen::Vector3d values(x, y, z + H);
    T.col(3).head(3) = values;
    // Eigen::Matrix4d T;
    // T<<1,0,0,x,
    //     0,1,0,y,
    //     0,0,1,z + H,
    //     0,0,0,1;
    // STEP3 计算旋转部分
    Eigen::Matrix4d Rx = Eigen::Matrix4d::Identity();
    rx = rx * M_PI/180;
    Rx(1, 1) = cos(rx);
    Rx(1, 2) = -sin(rx);
    Rx(2, 1) = sin(rx);
    Rx(2, 2) = cos(rx);

    Eigen::Matrix4d Ry = Eigen::Matrix4d::Identity();
    ry = ry * M_PI/180;
    Ry(0, 0) = cos(ry);
    Ry(0, 2) = sin(ry);
    Ry(2, 0) = -sin(ry);
    Ry(2, 2) = cos(ry);

    Eigen::Matrix4d Rz = Eigen::Matrix4d::Identity();
    rz = rz * M_PI/180;
    Rz(0, 0) = cos(rz);
    Rz(0, 1) = -sin(rz);
    Rz(1, 0) = sin(rz);
    Rz(1, 1) = cos(rz);

    Eigen::Matrix4d e0_Rot_e1 = Rz * Ry * Rx;
    T = T * e0_Rot_e1;
    return T;
}

Eigen::VectorXd ParallelPlatform::getList(Eigen::Matrix4d T)
{
    // 移动部分的提取
    double H = 0; // H为平台高度
    double x = T(0,3);
    double y = T(1,3);
    double z = T(2,3) - H;
    // 旋转部分的提取
    double theta_y = atan2(-T(2, 0), sqrt(T(2, 1)*T(2, 1) + T(2, 2)*T(2, 2))); // ry
    double theta_x = atan2(T(2, 1), T(2, 2)); // rx
    double theta_z = atan2(T(1, 0), T(0, 0)); // rz
    double rx = theta_x * 180.0 * 1000000/ M_PI;
    double ry = theta_y * 180.0 * 1000000/ M_PI;
    double rz = theta_z * 180.0 * 1000000/ M_PI;
    Eigen::VectorXd TaskPosition = Eigen::VectorXd::Zero(6);
    TaskPosition(0) = int(x); TaskPosition(1) = int(y); TaskPosition(2) = int(z);
    TaskPosition(3) = int(rx); TaskPosition(4) = int(ry); TaskPosition(5) = int(rz);
    return TaskPosition;
}

bool ParallelPlatform::setTargetVelocity(Eigen::VectorXd V)
{
    setRobotMode(ROBOT_VELOCITY);
    this->targetVelocity_ = V;
    return true;
}

bool ParallelPlatform::setTargetPose(Eigen::VectorXd P)
{
    setRobotMode(ROBOT_POSITION);
    this->targetPose_ = P;
    return true;
}

void ParallelPlatform::setRobotMode(int mode)
{
    controlMode_ = mode;
}
bool  ParallelPlatform::moveVelocity(Eigen::VectorXd VelocityList_b, double dt)
{
    // 单位：
    // input:VelocityList位置部分采用nm,角度部分采用u°,dt为s
    // 程序运算中:VelocityList位置部分采用nm,角度部分采用弧度,dt为s
    // step1:VelocityList_b 角度部分单位转换
    // VelocityList_b(0) = VelocityList_b(0) * M_PI/(180*1000000);
    // VelocityList_b(1) = VelocityList_b(1) * M_PI/(180*1000000);
    // VelocityList_b(2) = VelocityList_b(2) * M_PI/(180*1000000);

    double k = M_PI/(180*1000000);
    Eigen::VectorXd VelocityList_temp(6);
    VelocityList_temp(0) = VelocityList_b(3) * k;
    VelocityList_temp(1) = VelocityList_b(4) * k;
    VelocityList_temp(2) = VelocityList_b(5) * k;
    VelocityList_temp(3) = VelocityList_b(0);
    VelocityList_temp(4) = VelocityList_b(1);
    VelocityList_temp(5) = VelocityList_b(2);

    Eigen::VectorXd VelocityList_e = Eigen::VectorXd::Zero(6);
    this->b_T_e_ = currentPose_Mat_;
    VelocityList_e = Adjoint(this->b_T_e_.inverse()) * VelocityList_temp;
    Eigen::Matrix4d e1_T_e2 = MatrixExp6(VecTose3(VelocityList_e) * dt);
    Eigen::Matrix4d b_T_e2 = this->b_T_e_ * e1_T_e2;
    bool res = moveAbsolute(b_T_e2);
    return res;
}

