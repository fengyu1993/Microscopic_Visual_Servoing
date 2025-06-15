#include "VisualServoingController.h"

VisualServoingController::VisualServoingController()
{
    read_vs_parameter("E:/QT/Microscopic_Visual_Servoing/resources/data/VS_Parameter.csv");

    this->m_camera = new BaslerCameraControl();

    this->m_robot = new  ParallelPlatform(1.0 / vs_parameter.control_rate);
    QThread* thread_m_robot = new QThread();
    m_robot->moveToThread(thread_m_robot);
    thread_m_robot->start();

    m_algorithm_DMVS = new Direct_Microscopic_Visual_Servoing();

    m_controlTimer = new QTimer();
    m_controlTimer->setInterval(100);
    m_controlTimer->setTimerType(Qt::PreciseTimer);

    m_isRunning = false;

    connect(m_controlTimer, &QTimer::timeout, this, &VisualServoingController::executeControlCycle);
}
VisualServoingController::~VisualServoingController()
{
    stopServoing();
    if(m_controlTimer->isActive())
    {
        m_controlTimer->stop();
    }
    delete this->m_camera;
    delete  this->m_robot;
    delete this->m_algorithm_DMVS;
    delete this->m_controlTimer;
}

void VisualServoingController::executeControlCycle()
{
    static QElapsedTimer cycleTimer;
    cycleTimer.start();

    try {
        // 获取执行器位姿
        Mat T = m_robot->getTaskMat_cv();
        // 获取最新图像
        std::shared_ptr<const cv::Mat> framePtr =  m_camera->getLatestFrameShared();
        m_algorithm_DMVS->image_gray_current_ = *framePtr;
        // 计算控制输出
        Mat velocity = m_algorithm_DMVS->get_object_velocity();
        // 存储数据
        m_algorithm_DMVS->save_all_data(T);
        // 判断是否成功
        if(m_algorithm_DMVS->is_success() || m_algorithm_DMVS->iteration_num_ > vs_parameter.max_iteration)
        {
            m_algorithm_DMVS->write_data();
            velocity = 0 * velocity;
            emit systemStatusChanged("Visual servoing success");
        }
        // 执行机器人运动
        Eigen::VectorXd velocity_eigen = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(velocity.ptr<double>(0)),velocity.rows);
        if(!m_robot-> setTargetVelocity(velocity_eigen)) {
            emit servoingError("Robot movement failed");
            return;
        }
        // 更新视觉伺服数据
        QVariantMap visData;
        visData["loop_time"] = QVariant::fromValue(cycleTimer.elapsed());
        visData["feature_error"] = m_algorithm_DMVS->cost_function_value_;
        emit updateVisualServoingData(visData);

    } catch (const std::exception &e) {
        emit servoingError(QString("Control cycle error: %1").arg(e.what()));
        stopServoing();
    }
}

bool VisualServoingController::initializeSystem()
{
    // 初始化相机
    if(!m_camera->init()) {
        emit systemStatusChanged("Camera initialization failed");
        return false;
    }
    // 连接机器人
    if(!m_robot->connect()) {
        emit systemStatusChanged("Robot connection failed");
        return false;
    }

    emit systemStatusChanged("System initialized");
    return true;
}

void VisualServoingController::startServoing()
{
    if(m_isRunning) {
        qWarning() << "Servoing is already running";
        return;
    }

    // 启动相机采集
    m_camera->StartAcquire();
    QThread::msleep(50);

    QString imagePath = vs_parameter.resource_location + vs_parameter.image_desired_name;
    Mat image_desired = imread(imagePath.toStdString());
    Mat image_initial = m_camera->img_cv.clone();
    m_algorithm_DMVS->init_VS(vs_parameter.lambda, vs_parameter.epsilon, image_desired,
                              image_initial, vs_parameter.camera_parameters, vs_parameter.pose_desired, vs_parameter.Tbc);

    // 启动控制定时器
    m_controlTimer->start();
    m_isRunning = true;

    emit systemStatusChanged("Servoing started");
}

void VisualServoingController::stopServoing()
{
    if(!m_isRunning) return;
    m_isRunning = false;
    // 停止控制定时器
    m_controlTimer->stop();
    // 停止相机采集
    m_camera->StopAcquire();
    // 停止机器人运动
    m_robot->stop();

    emit systemStatusChanged("Servoing stopped");
}

bool VisualServoingController::read_vs_parameter(QString location)
{
    vs_parameter.resolution_x = 1600;
    vs_parameter.resolution_y = 1200;
    vs_parameter.lambda = 0.03;
    vs_parameter.epsilon = 0.001;
    vs_parameter.control_rate = 10;
    vs_parameter.camera_parameters.c_u = 0.3;
    vs_parameter.camera_parameters.c_v = 0.4;
    vs_parameter.camera_parameters.Z_f = 2;
    vs_parameter.camera_parameters.R_f = 1;
    vs_parameter.camera_parameters.D_f_k_uv = 6.2;
    vs_parameter.resource_location = "E:/QT/Microscopic_Visual_Servoing/resources/data";
    vs_parameter.image_desired_name = "image_desired.png";
    vs_parameter.pose_desired = Mat::ones(4, 4, CV_64FC1);
    vs_parameter.Tbc = Mat::ones(4, 4, CV_64FC1);

    return true;
}
