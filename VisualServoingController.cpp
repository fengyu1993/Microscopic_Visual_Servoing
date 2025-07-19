#include "VisualServoingController.h"

VisualServoingController::VisualServoingController()
{
    if(read_vs_parameter(":/resources/data/VS_Parameter.csv")){
        output_vs_parameter();
        this->robotPoses.workPose = cvMatToEigenMatrix(vs_parameter.pose_work);
        this->robotPoses.desiredPose = cvMatToEigenMatrix(vs_parameter.pose_desired);
    }
    else{
        emit servoingError("Read VS parameters failed");
    }

    this->m_camera = new BaslerCameraControl(20);


    this->m_robot = new  ParallelPlatform(20);
    QThread* thread_m_robot = new QThread();
    m_robot->moveToThread(thread_m_robot);
    thread_m_robot->start();

    QString imagePath = vs_parameter.resource_location +"/" + vs_parameter.image_desired_name;
    Mat image_desired= imread(imagePath.toStdString(), cv::IMREAD_UNCHANGED);
    m_algorithm_DMVS = new Direct_Microscopic_Visual_Servoing(vs_parameter.resolution_x, vs_parameter.resolution_y);
    m_algorithm_DMVS->init_VS(vs_parameter.lambda, vs_parameter.epsilon, image_desired, vs_parameter.camera_parameters, vs_parameter.pose_desired, vs_parameter.Tbc);

    this->m_microscope_calibration = new MicroscopeCalibration(vs_parameter.resolution_x, vs_parameter.resolution_y);

    m_controlTimer = new QTimer();
    m_controlTimer->setInterval(1.0 / vs_parameter.control_rate *1000);
    m_controlTimer->setTimerType(Qt::PreciseTimer);

    m_isRunning = false;
    cnt = 0;
    mode = MODE_NULL;
    circle_du = 0;
    circle_dv = 0;
    circle_dr = 0;
    midifyCircle = cv::Mat::zeros(3, 1, CV_64FC1);
    flagRecord = false;
    flagCheckCircle = false;
    flag_first_VS = false;

    bestCircle <<  vs_parameter.resolution_x/2,  vs_parameter.resolution_y/2, vs_parameter.resolution_y / 10.0;

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
    delete this->m_microscope_calibration;
}

void VisualServoingController::executeControlCycle()
{
    switch (this->mode) {
    case MODE_VISUAL_SERVOING:
        visualServoingControl();
        break;
    case MODE_SHARPNESS:
        sharpnessControl();
        break;
    case MODE_CALIBRATION:
        calibrationControl();
        break;
    default:
        break;
    }
}

void VisualServoingController::visualServoingControl()
{
    if(m_isRunning){
        qDebug() << "cnt: " << m_algorithm_DMVS->iteration_num_;
        // qDebug() << "executeControlCycle thread ID:" << QThread::currentThreadId();

        try {
            // // 获取执行器位姿
            Mat T = m_robot->getTaskMat_cv();
            // // 更新平面参数
            Mat T_co =  vs_parameter.Tbc * T;
            Mat plame_para;
            cv::divide(T_co(cv::Rect(2, 0, 1, 3)), T_co(cv::Rect(2, 0, 1, 3)).t() * T_co(cv::Rect(3, 0, 1, 3)), plame_para);
            m_algorithm_DMVS->updeta_planar_paramters(plame_para.at<double>(0,0), plame_para.at<double>(1,0), plame_para.at<double>(2,0));
            // m_algorithm_DMVS->updeta_planar_paramters(0, 0, 1/vs_parameter.camera_parameters.Z_f);
            // 获取最新图像
            m_algorithm_DMVS->image_gray_current_  = m_camera->getLatestFrame();
            if(flag_first_VS)
            {
                m_algorithm_DMVS->set_image_initial(m_camera->getImage());
                flag_first_VS = false;
            }
            // cv::imshow("current", m_algorithm_DMVS->image_gray_current_);
            // cv::imshow("desired", m_algorithm_DMVS->image_gray_desired_);
            // cv::waitKey(1);

            // 计算控制输出
            Mat velocity = m_algorithm_DMVS->get_object_velocity();
            // 存储数据
            m_algorithm_DMVS->save_all_data(T);
            // 判断是否成功
            if(m_algorithm_DMVS->is_success() || m_algorithm_DMVS->iteration_num_ > vs_parameter.max_iteration)
            {
                qDebug() << "Visual Servoing Finish";
                QString location = vs_parameter.resource_location + "/";
                m_algorithm_DMVS->write_all_data(location.toStdString());
                qDebug() << "Write Data Finish";
                velocity = 0 * velocity;
                stopServoing();
                emit systemStatusChanged(VS_SUCCESS);
            }
            // // 执行机器人运动
            Eigen::VectorXd velocity_eigen = Eigen::Map<Eigen::VectorXd>(const_cast<double*>(velocity.ptr<double>(0)),velocity.rows);
            velocity_eigen.head(3) = velocity_eigen.head(3); // 机器人控制平台单位：nm
            velocity_eigen.tail(3) = velocity_eigen.tail(3) * 1e6; // 机器人控制平台单位：μ°
            // std::stringstream ss2;
            // ss2 << velocity_eigen;
            // qDebug() << "velocity:\n" << ss2.str().c_str();

            if(!m_robot-> setTargetVelocity(velocity_eigen)) {
                emit servoingError("Robot movement failed");
                return;
            }
            qDebug() << "error: " << m_algorithm_DMVS->cost_function_value_;
            qDebug() << "阈值: " << m_algorithm_DMVS->epsilon_;
            // 更新视觉伺服数据
            QVariantMap visData;
            visData["loop_time"] = QVariant::fromValue(cycleTimer.elapsed());
            visData["feature_error"] = m_algorithm_DMVS->cost_function_value_;
            visData["velocity_vx"] = velocity.at<double>(0);
            visData["velocity_vy"] = velocity.at<double>(1);
            visData["velocity_vz"] = velocity.at<double>(2);
            visData["velocity_wx"] = velocity.at<double>(3);
            visData["velocity_wy"] = velocity.at<double>(4);
            visData["velocity_wz"] = velocity.at<double>(5);
            emit updateVisualServoingData(visData);
        } catch (const std::exception &e) {
            emit servoingError(QString("Control cycle error: %1").arg(e.what()));
            stopServoing();
        }
    }
}

void VisualServoingController::sharpnessControl()
{
    if(m_isRunning){
        // 获取图像
        Mat image  = m_camera->getLatestFrame();
        // 计算Laplacian算子
        Mat I_u, I_v, I_uu, I_vv, Delta_I;
        m_algorithm_DMVS->get_image_gradient_u(image, I_u);
        m_algorithm_DMVS->get_image_gradient_v(image, I_v);
        m_algorithm_DMVS->get_image_gradient_u(I_u, I_uu);
        m_algorithm_DMVS->get_image_gradient_v(I_v, I_vv);
        cv::add(I_uu, I_vv, Delta_I);
        // 计算锐度指标
        cv::Scalar sum = cv::sum(cv::abs(Delta_I));
        this->sharpness = sum[0];
        // Scalar mean, stddev;
        // meanStdDev(Delta_I, mean, stddev);
        // this->sharpness = stddev.val[0] * stddev.val[0];
        // 更新锐度数据
        QVariantMap visData;
        visData["loop_time"] = QVariant::fromValue(cycleTimer.elapsed());
        visData["sharpness"] = this->sharpness;
        emit updateVisualServoingData(visData);
    }
}

void VisualServoingController::calibrationControl()
{
    if(m_isRunning)
    {
        // 获取执行器位姿
        Mat T = m_robot->getTaskMat_cv();
        // 获取图像
        Mat image  = m_camera->getLatestFrame();
        // Mat image = imread(":/resources/data/image_desired.png");
        // 检测圆
        Mat src_8u;
        image.convertTo(src_8u, CV_8U, 255.0);
        if(flagCheckCircle){
            this->bestCircle = checkBestCircle(src_8u);
            disableCheckCircle();
        }
        // 修正圆
        midifyCircle.at<double>(0, 0) = bestCircle[0] + circle_du;
        midifyCircle.at<double>(1, 0) = bestCircle[1] + circle_dv;
        midifyCircle.at<double>(2, 0) = bestCircle[2] + circle_dr;
        // qDebug() << "center_x: " << midifyCircle.at<double>(0, 0);
        // qDebug() << "center_y: " << midifyCircle.at<double>(1, 0);
        // qDebug() << "center_r: " << midifyCircle.at<double>(2, 0);
        // 显式标注后的图像
        Point center(cvRound(midifyCircle.at<double>(0, 0)), cvRound(midifyCircle.at<double>(1, 0)));
        Point center_image(cvRound(image.cols / 2), cvRound(image.rows / 2));
        int radius = cvRound(midifyCircle.at<double>(2, 0) );
        cv::cvtColor(src_8u, src_8u, cv::COLOR_GRAY2BGR);
        circle(src_8u, center, radius, Scalar(0, 255, 0), cvRound(image.cols/256));
        circle(src_8u, center_image, cvRound(image.cols/512), Scalar(255, 0, 0), cvRound(image.cols/256));
        circle(src_8u, center, cvRound(image.cols/512), Scalar(0, 0, 255), cvRound(image.cols/256));
        // 输出显示
        QImage img =  this->m_camera->cvMatToQImage(src_8u);
        emit sigCalibrationImage(img);
        // 标定
        bool flagFinish = false;
        switch (this->stepCalibration)
        {
        case 1:
            // qDebug() << "stepCalibration_1：找到焦平面";
            sharpnessControl();
            if(this->flagRecord)
            {
                m_robot->getTaskPositions(robotPoses.focusPose);
                enableflagRecord(false);
                std::stringstream ss;
                ss << robotPoses.focusPose;
                qDebug() << "robotFocusPose:\n" << ss.str().c_str();
            }
            break;
        case 2:
            // qDebug() << "stepCalibration_2：沿x移动采点";
            if(this->flagRecord)
            {
                recordPiont(m_microscope_calibration->calibrationData.pointsPuvMoveXD0, m_microscope_calibration->calibrationData.pointsPXYZMoveXD0);
                enableflagRecord(false);
                std::stringstream ss1, ss2;
                ss1 << m_microscope_calibration->calibrationData.pointsPuvMoveXD0;
                ss2 << m_microscope_calibration->calibrationData.pointsPXYZMoveXD0;
                qDebug() << "pointsPuvMoveXD0:\n" << ss1.str().c_str();
                qDebug() << "pointsPXYZMoveXD0:\n" << ss2.str().c_str();
            }
            break;
        case 3:
            // qDebug() << "stepCalibration_3：沿y移动采点";
            if(this->flagRecord)
            {
                recordPiont(m_microscope_calibration->calibrationData.pointsPuvMoveYD0, m_microscope_calibration->calibrationData.pointsPXYZMoveYD0);
                enableflagRecord(false);
                std::stringstream ss1, ss2;
                ss1 << m_microscope_calibration->calibrationData.pointsPuvMoveYD0;
                ss2 << m_microscope_calibration->calibrationData.pointsPXYZMoveYD0;
                qDebug() << "pointsPuvMoveYD0:\n" << ss1.str().c_str();
                qDebug() << "pointsPXYZMoveYD0:\n" << ss2.str().c_str();
            }
            break;
        case 4:
            // qDebug() << "stepCalibration_4：Z轴向上移动后，再沿xy移动采点";
            if(this->flagRecord)
            {
                recordPiont(m_microscope_calibration->calibrationData.pointsPuvMoveXYDn, m_microscope_calibration->calibrationData.pointsPXYZMoveXYDn);
                enableflagRecord(false);
                std::stringstream ss1, ss2;
                ss1 << m_microscope_calibration->calibrationData.pointsPuvMoveXYDn;
                ss2 << m_microscope_calibration->calibrationData.pointsPXYZMoveXYDn;
                qDebug() << "pointsPuvMoveXYDn:\n" << ss1.str().c_str();
                qDebug() << "pointsPXYZMoveXYDn:\n" << ss2.str().c_str();
            }
            break;
        case 5:
            // qDebug() << "stepCalibration_5：Z轴向下移动后，再沿xy移动采点";
            if(this->flagRecord)
            {
                recordPiont(m_microscope_calibration->calibrationData.pointsPuvMoveXYDp, m_microscope_calibration->calibrationData.pointsPXYZMoveXYDp);
                enableflagRecord(false);
                std::stringstream ss1, ss2;
                ss1 << m_microscope_calibration->calibrationData.pointsPuvMoveXYDp;
                ss2 << m_microscope_calibration->calibrationData.pointsPXYZMoveXYDp;
                qDebug() << "pointsPuvMoveXYDp:\n" << ss1.str().c_str();
                qDebug() << "pointsPXYZMoveXYDp:\n" << ss2.str().c_str();
            }
            break;
        case 6:
            // qDebug() << "stepCalibration_6：回到焦平面";
            m_robot->setTargetPose(robotPoses.focusPose);
            break;
        case 7:
            // qDebug() << "stepCalibration_7：绕Z轴转动并采点";
            if(this->flagRecord)
            {
                recordPiont(m_microscope_calibration->calibrationData.pointsPuvRotateZD0, m_microscope_calibration->calibrationData.pointsPXYZRotateZD0);
                Mat T = m_robot->getTaskMat_cv();
                m_microscope_calibration->calibrationData.posesTsRotateZD0.push_back(T.clone());
                enableflagRecord(false);
                std::stringstream ss1, ss2, ss3;
                // ss1 << m_microscope_calibration->calibrationData.pointsPuvRotateZD0;
                // ss2 << m_microscope_calibration->calibrationData.pointsPXYZRotateZD0;
                ss3 << m_microscope_calibration->calibrationData.posesTsRotateZD0.back();
                // qDebug() << "pointsPuvRotateZD0:\n" << ss1.str().c_str();
                // qDebug() << "pointsPXYZRotateZD0:\n" << ss2.str().c_str();
                qDebug() << "posesTsRotateZD:\n" << ss3.str().c_str();
            }
            break;
        case 8:
            // qDebug() << "stepCalibration_8：将圆心移至图像正中心并记录圆半径";
            if(this->flagRecord)
            {
                m_microscope_calibration->calibrationData.rf = radius;
                enableflagRecord(false);
                std::stringstream ss;
                ss << m_microscope_calibration->calibrationData.rf;
                qDebug() << "rf:\n" << ss.str().c_str();
            }
            break;
        case 9:
            // qDebug() << "stepCalibration_9：沿Z轴上下移动并记录圆半径";
            if(this->flagRecord)
            {
                Mat T = m_robot->getTaskMat_cv();
                Mat radiusZ =  (cv::Mat_<double>(2, 1) << radius, T.at<double>(2, 3));
                cv::hconcat(m_microscope_calibration->calibrationData.radiusZMoveZ, radiusZ, m_microscope_calibration->calibrationData.radiusZMoveZ);
                enableflagRecord(false);
                std::stringstream ss;
                ss << m_microscope_calibration->calibrationData.radiusZMoveZ;
                qDebug() << "radiusZMoveZ:\n" << ss.str().c_str();
            }
            break;
        case 10:
            qDebug() << "stepCalibration_10：计算参数";
            flagFinish = true;
            break;
        default:
            break;
        }
        // 标定参数计算
        if(flagFinish)
        {
            m_microscope_calibration->writeCalibrationData(m_microscope_calibration->calibrationData);
            m_microscope_calibration->calibration(m_microscope_calibration->calibrationData, m_microscope_calibration->microscopicParameter);
            // 输出数据
            m_microscope_calibration->writeCalibrationResult(m_microscope_calibration->microscopicParameter);
            flagFinish = false;
            setStepCalibration(0);
        }
    }
}

Vec3f VisualServoingController::checkBestCircle(Mat img)
{
    medianBlur(img, img, 5);
    Mat fixed_thresh;
    cv::threshold(img, fixed_thresh, 25, 255, THRESH_BINARY);
    vector<Vec3f> circles;
    HoughCircles(fixed_thresh, circles, HOUGH_GRADIENT, 1,
                 1000,  // 两个圆之间的最小距离
                 10, 10, 20, 200 // 参数可以根据需要调整
                 );

    // 选择最可靠的圆（基于累加器值）
    Vec3f bestCircle;
    if (!circles.empty())
    {
        // 按置信度排序（假设最后一个参数是置信度，实际需要根据OpenCV版本调整）
        sort(circles.begin(), circles.end(),
             [](const Vec3f& a, const Vec3f& b) {
                 return a[2] > b[2]; // 按半径排序，或使用其他标准
             });
        // 取第一个（最可靠的）圆
        bestCircle = circles[0];
    }
    else{
        bestCircle[0] = vs_parameter.resolution_x/2;
        bestCircle[1] = vs_parameter.resolution_y/2;
        bestCircle[2] = vs_parameter.resolution_y / 10.0;
        // qDebug() << "图像未检测出圆形，请调整HoughCircles函数的参数";
    }
    return bestCircle;
}

void VisualServoingController::enableCheckCircle()
{
    this->flagCheckCircle = true;
}

void VisualServoingController::disableCheckCircle()
{
    this->flagCheckCircle = false;
}

void VisualServoingController::setWorkPose()
{
    m_robot->getTaskMat(robotPoses.workPose);
}

void VisualServoingController::setDesiredPose()
{
    m_robot->getTaskMat(robotPoses.desiredPose);
}

 void VisualServoingController::getRobotPoses(RobotPoses& poses)
{
     poses = this->robotPoses;
}

void VisualServoingController::saveRobotPoses()
{
    string file_name = "robotPoses";
    string location = LOCATION;
    ofstream oFile;
    string excel_name = location + file_name + ".xls";
    oFile.open(excel_name, ios::out|ios::trunc);
    oFile << "workPose" << endl;
    write_to_excel(this->robotPoses.workPose, oFile);
    oFile << "desiredPose" << endl;
    write_to_excel(this->robotPoses.desiredPose, oFile);
    oFile.close();
}

void VisualServoingController::recordPiont(Mat& PuvList, Mat& PxyzList)
{
    Eigen::VectorXd positions;
    m_robot->getTaskPositions(positions);
    Mat Puv = (cv::Mat_<double>(2, 1) << midifyCircle.at<double>(0, 0), midifyCircle.at<double>(1, 0));
    Mat Pxyz =  (cv::Mat_<double>(3, 1) << positions[0], positions[1], positions[2]);
    cv::hconcat(PuvList, Puv, PuvList);
    cv::hconcat(PxyzList, Pxyz, PxyzList);
}

void VisualServoingController::enableflagRecord(bool flag)
{
    this->flagRecord = flag;
}

void VisualServoingController::setStepCalibration(int step)
{
    this->stepCalibration = step;
}

void VisualServoingController::setCircleDxDyDr(int dx, int dy, int dr)
{
    this->circle_du = dx;
    this->circle_dv = dy;
    this->circle_dr = dr;
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
    QThread::msleep(500);

    flag_first_VS = true;

    // 启动控制定时器
    m_controlTimer->start();
    m_isRunning = true;
    // 开始计时
    cycleTimer.restart();
    emit systemStatusChanged("Servoing started");
}

void VisualServoingController::stopServoing()
{
    if(!m_isRunning) return;
    // 停止控制定时器
    m_controlTimer->stop();
    // 停止相机采集
    m_camera->StopAcquire();
    // 停止机器人运动
    m_robot->stop();
    m_isRunning = false;
    emit systemStatusChanged("Servoing stopped");
}

void VisualServoingController::output_vs_parameter()
{

    qDebug() << "resolution_x: " << vs_parameter.resolution_x;
    qDebug() << "resolution_y: " << vs_parameter.resolution_y;
    qDebug() << "lambda: " << vs_parameter.lambda;
    qDebug() << "epsilon: " << vs_parameter.epsilon;
    qDebug() << "control_rate: " << vs_parameter.control_rate;
    qDebug() << "camera_parameters: \n" << "D_f_k_uv: "<< vs_parameter.camera_parameters.D_f_k_uv << "\n"
             << "R_f: "<< vs_parameter.camera_parameters.R_f << "\n"
             << "Z_f: "<< vs_parameter.camera_parameters.Z_f << "\n"
             << "c_u: "<< vs_parameter.camera_parameters.c_u << "\n"
             << "c_v: "<< vs_parameter.camera_parameters.c_v;
    qDebug() << "resource_location: " << vs_parameter.resource_location;
    qDebug() << "image_desired_name: " << vs_parameter.image_desired_name;
    qDebug() << "max_iteration: " << vs_parameter.max_iteration;
    std::stringstream ss1;
    ss1 << vs_parameter.Tbc;
    qDebug() << "Tbc:\n" << ss1.str().c_str();
    ss1.str("");
    ss1 << vs_parameter.Tcb;
    qDebug() << "Tcb:\n" << ss1.str().c_str();
    ss1.str("");
    ss1 << vs_parameter.pose_desired;
    qDebug() << "pose_desired:\n" << ss1.str().c_str();
    ss1.str("");
    ss1 << vs_parameter.pose_work;
    qDebug() << "pose_work:\n" << ss1.str().c_str();
}

void VisualServoingController::setMode(int mode)
{
    this->mode = mode;
    switch (this->mode){
    case MODE_VISUAL_SERVOING:
        qDebug() << "MODE_VISUAL_SERVOING";
        break;
    case MODE_SHARPNESS:
        qDebug() << "MODE_SHARPNESS";
        break;
    case MODE_CALIBRATION:
        qDebug() << "MODE_CALIBRATION";
        break;
    default:
        qDebug() << "MODE_NULL";
    }
}

int VisualServoingController::getMode()
{
    return this->mode;
}

bool VisualServoingController::read_vs_parameter(QString location) {
    // 定义默认的 camera_intrinsic 参数，使用表格中的数值

    QFile file(location);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Error: Could not open file:" << location << file.errorString();
        emit servoingError("Read VS parameters failed");
        return false;
    }

    QTextStream in(&file);
    bool overall_success = true; // 标志整体读取是否成功

    // Current parsing state to handle multi-line data
    enum ParsingState {
        ReadingGeneral,
        ReadingPoseDesired,
        ReadingWorkDesired,
        ReadingTbc
    };
    ParsingState currentState = ReadingGeneral;
    int matrixRowsRead = 0; // Counter for matrix rows

    // Use a temporary cv::Mat to build the matrix before assigning
    cv::Mat currentMatrix;

    try {
        while (!in.atEnd()) {
            QString line = in.readLine();
            line = line.trimmed(); // 移除首尾空格
            if (line.isEmpty() || line.startsWith("#")) { // 跳过空行和注释行 (以 '#' 开头的行)
                continue;
            }

            QStringList parts = line.split(',');
            // Remove empty parts from the end that might come from extra commas
            while (!parts.isEmpty() && parts.last().isEmpty()) {
                parts.removeLast();
            }

            bool ok; // For conversion checks

            if (currentState == ReadingGeneral) {
                // If the line starts with a comma or empty key, it's likely part of a matrix that
                // wasn't initiated by its header, or an error from previous parsing.
                // We'll skip such lines when in ReadingGeneral state.
                if (parts.isEmpty() || (parts.at(0).trimmed().isEmpty() && parts.size() > 1) ) {
                    qWarning() << "Warning: Skipping line with empty key or starting with comma in general reading state:" << line;
                    continue;
                }

                QString paramName = parts.at(0).trimmed();

                if (paramName == "resolution_x") {
                    if (parts.size() >= 2) {
                        vs_parameter.resolution_x = parts.at(1).trimmed().toInt(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert resolution_x to int for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid resolution_x format for line:" << line << ". Using default."; }
                } else if (paramName == "resolution_y") {
                    if (parts.size() >= 2) {
                        vs_parameter.resolution_y = parts.at(1).trimmed().toInt(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert resolution_y to int for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid resolution_y format for line:" << line << ". Using default."; }
                } else if (paramName == "lambda") {
                    if (parts.size() >= 2) {
                        vs_parameter.lambda = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert lambda to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid lambda format for line:" << line << ". Using default."; }
                } else if (paramName == "epsilon") {
                    if (parts.size() >= 2) {
                        vs_parameter.epsilon = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert epsilon to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid epsilon format for line:" << line << ". Using default."; }
                } else if (paramName == "control_rate") {
                    if (parts.size() >= 2) {
                        vs_parameter.control_rate = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert control_rate to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid control_rate format for line:" << line << ". Using default."; }
                } else if (paramName == "c_u") { // Handle individual camera params
                    if (parts.size() >= 2) {
                        vs_parameter.camera_parameters.c_u = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert c_u to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid c_u format for line:" << line << ". Using default."; }
                } else if (paramName == "c_v") {
                    if (parts.size() >= 2) {
                        vs_parameter.camera_parameters.c_v = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert c_v to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid c_v format for line:" << line << ". Using default."; }
                } else if (paramName == "Z_f") {
                    if (parts.size() >= 2) {
                        vs_parameter.camera_parameters.Z_f = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert Z_f to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid Z_f format for line:" << line << ". Using default."; }
                } else if (paramName == "D_f_k_uv") { // Updated to match exact CSV name
                    if (parts.size() >= 2) {
                        vs_parameter.camera_parameters.D_f_k_uv = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert D_f_k_uv to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid D_f_k_uv format for line:" << line << ". Using default."; }
                } else if (paramName == "R_f") {
                    if (parts.size() >= 2) {
                        vs_parameter.camera_parameters.R_f = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert R_f to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid R_f format for line:" << line << ". Using default."; }
                } else if (paramName == "resource_location") { // Matched exact key from latest CSV
                    if (parts.size() >= 2) {
                        QString value = parts.at(1).trimmed();
                        while (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                            value = value.mid(1, value.length() - 2);
                        }
                        value = value.replace("\"\"", "\"");
                        vs_parameter.resource_location = value;
                    } else { qWarning() << "Warning: Invalid resource_location format for line:" << line << ". Using default."; }
                } else if (paramName == "image_rgb_desired_name") { // Matched exact key from latest CSV
                    if (parts.size() >= 2) {
                        QString value = parts.at(1).trimmed();
                        while (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                            value = value.mid(1, value.length() - 2);
                        }
                        value = value.replace("\"\"", "\"");
                        vs_parameter.image_desired_name = value;
                    } else { qWarning() << "Warning: Invalid image_rgb_desired_name format for line:" << line << ". Using default."; }
                } else if (paramName == "pose_desired") {
                    // Start reading matrix, initialize currentMatrix
                    currentMatrix = cv::Mat::zeros(4, 4, CV_64F);
                    matrixRowsRead = 0;
                    currentState = ReadingPoseDesired;

                    // If the line "pose_desired" also contains the first row of matrix data
                    if (parts.size() > 1) { // Check if there are elements after "pose_desired"
                        QStringList matrixRowParts = parts.mid(1); // Get elements after the key
                        // Remove empty parts from the end of this sublist
                        while (!matrixRowParts.isEmpty() && matrixRowParts.last().isEmpty()) {
                            matrixRowParts.removeLast();
                        }

                        if (matrixRowParts.size() == 4) { // Expect 4 elements for the first row
                            for (int j = 0; j < 4; ++j) {
                                QString elementStr = matrixRowParts.at(j).trimmed();
                                double element = 0.0;
                                if (!elementStr.isEmpty()) { // Only try to convert if not empty
                                    element = elementStr.toDouble(&ok);
                                    if (!ok) {
                                        qWarning() << "Warning: Failed to convert pose_desired matrix element '" << elementStr << "' to double for line:" << line << ". Setting to 0.0.";
                                        element = 0.0; // Use 0.0 if conversion fails
                                    }
                                }
                                currentMatrix.at<double>(matrixRowsRead, j) = element;
                            }
                            matrixRowsRead++; // Increment row count as first row is parsed
                        } else {
                            qWarning() << "Warning: Invalid pose_desired first row format. Expected 4 elements on the key line for line:" << line << ". Using default matrix.";
                            currentState = ReadingGeneral; // Reset state
                            vs_parameter.pose_desired = cv::Mat::eye(4,4,CV_64F); // Reset to default
                            overall_success = false;
                        }
                    } // If parts.size() == 1, then it's just the key, and matrix data starts next line (handled by subsequent state)
                }  else if (paramName == "pose_work") {
                    // Start reading matrix, initialize currentMatrix
                    currentMatrix = cv::Mat::zeros(4, 4, CV_64F);
                    matrixRowsRead = 0;
                    currentState = ReadingWorkDesired;

                    // If the line "pose_work" also contains the first row of matrix data
                    if (parts.size() > 1) { // Check if there are elements after "pose_desired"
                        QStringList matrixRowParts = parts.mid(1); // Get elements after the key
                        // Remove empty parts from the end of this sublist
                        while (!matrixRowParts.isEmpty() && matrixRowParts.last().isEmpty()) {
                            matrixRowParts.removeLast();
                        }

                        if (matrixRowParts.size() == 4) { // Expect 4 elements for the first row
                            for (int j = 0; j < 4; ++j) {
                                QString elementStr = matrixRowParts.at(j).trimmed();
                                double element = 0.0;
                                if (!elementStr.isEmpty()) { // Only try to convert if not empty
                                    element = elementStr.toDouble(&ok);
                                    if (!ok) {
                                        qWarning() << "Warning: Failed to convert pose_desired matrix element '" << elementStr << "' to double for line:" << line << ". Setting to 0.0.";
                                        element = 0.0; // Use 0.0 if conversion fails
                                    }
                                }
                                currentMatrix.at<double>(matrixRowsRead, j) = element;
                            }
                            matrixRowsRead++; // Increment row count as first row is parsed
                        } else {
                            qWarning() << "Warning: Invalid pose_desired first row format. Expected 4 elements on the key line for line:" << line << ". Using default matrix.";
                            currentState = ReadingGeneral; // Reset state
                            vs_parameter.pose_work = cv::Mat::eye(4,4,CV_64F); // Reset to default
                            overall_success = false;
                        }
                    } // If parts.size() == 1, then it's just the key, and matrix data starts next line (handled by subsequent state)
                }else if (paramName == "Tbc") {
                    // Start reading matrix, initialize currentMatrix
                    currentMatrix = cv::Mat::zeros(4, 4, CV_64F);
                    matrixRowsRead = 0;
                    currentState = ReadingTbc;

                    // If the line "Tbc" also contains the first row of matrix data
                    if (parts.size() > 1) { // Check if there are elements after "Tbc"
                        QStringList matrixRowParts = parts.mid(1); // Get elements after the key
                        // Remove empty parts from the end of this sublist
                        while (!matrixRowParts.isEmpty() && matrixRowParts.last().isEmpty()) {
                            matrixRowParts.removeLast();
                        }

                        if (matrixRowParts.size() == 4) { // Expect 4 elements for the first row
                            for (int j = 0; j < 4; ++j) {
                                QString elementStr = matrixRowParts.at(j).trimmed();
                                double element = 0.0;
                                if (!elementStr.isEmpty()) { // Only try to convert if not empty
                                    element = elementStr.toDouble(&ok);
                                    if (!ok) {
                                        qWarning() << "Warning: Failed to convert Tbc matrix element '" << elementStr << "' to double for line:" << line << ". Setting to 0.0.";
                                        element = 0.0; // Use 0.0 if conversion fails
                                    }
                                }
                                currentMatrix.at<double>(matrixRowsRead, j) = element;
                            }
                            matrixRowsRead++; // Increment row count as first row is parsed
                        } else {
                            qWarning() << "Warning: Invalid Tbc first row format. Expected 4 elements on the key line for line:" << line << ". Using default matrix.";
                            currentState = ReadingGeneral; // Reset state
                            vs_parameter.Tbc = cv::Mat::eye(4,4,CV_64F); // Reset to default
                            overall_success = false;
                        }
                    } // If parts.size() == 1, then it's just the key, and matrix data starts next line (handled by subsequent state)
                }
                else if (paramName == "max_iteration") {
                    if (parts.size() >= 2) {
                        vs_parameter.max_iteration = parts.at(1).trimmed().toInt(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert resolution_x to int for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid resolution_x format for line:" << line << ". Using default."; }
                }else {
                    qWarning() << "Warning: Unknown parameter:" << paramName << "for line:" << line << ". Skipping.";
                }
            } else { // Currently parsing a multi-line matrix (ReadingPoseDesired or ReadingTbc)
                // Remove the first empty part if the line starts with a comma
                if (!parts.isEmpty() && parts.first().trimmed().isEmpty()) {
                    parts.removeFirst();
                }

                // Expect 4 parts for a 4x4 matrix row after removing potential leading empty part
                if (parts.size() != 4) {
                    qWarning() << "Warning: Invalid matrix row format. Expected 4 elements for line:" << line << " after comma handling. Matrix parsing aborted. Using default.";
                    currentState = ReadingGeneral; // Reset state
                    if (currentState == ReadingPoseDesired) vs_parameter.pose_desired = cv::Mat::eye(4,4,CV_64F); // Reset to default
                    else if (currentState == ReadingTbc) {
                        vs_parameter.Tbc = cv::Mat::eye(4,4,CV_64F); // Reset to default
                        vs_parameter.Tcb = cv::Mat::eye(4,4,CV_64F);
                    }
                    else if (currentState == ReadingWorkDesired) vs_parameter.pose_work = cv::Mat::eye(4,4,CV_64F); // Reset to default
                    overall_success = false; // Indicate a problem during parsing
                    continue; // Skip current malformed line
                }

                // Populate currentMatrix row
                for (int j = 0; j < 4; ++j) {
                    QString elementStr = parts.at(j).trimmed();
                    double element = 0.0; // Default to 0.0 if empty or conversion fails
                    if (!elementStr.isEmpty()) {
                        element = elementStr.toDouble(&ok);
                        if (!ok) {
                            qWarning() << "Warning: Failed to convert matrix element '" << elementStr << "' to double for line:" << line << ". Setting to 0.0.";
                            element = 0.0; // Set to 0.0 if conversion fails
                        }
                    }
                    currentMatrix.at<double>(matrixRowsRead, j) = element;
                }

                matrixRowsRead++;
                if (matrixRowsRead == 4) { // Finished reading all 4 rows
                    if (currentState == ReadingPoseDesired) {
                        vs_parameter.pose_desired = currentMatrix.clone(); // Assign the parsed matrix
                        qDebug() << "Successfully parsed pose_desired matrix.";
                    } else if (currentState == ReadingWorkDesired) {
                        vs_parameter.pose_work = currentMatrix.clone(); // Assign the parsed matrix
                        qDebug() << "Successfully parsed pose_work matrix.";
                    } else if (currentState == ReadingTbc) {
                        vs_parameter.Tbc = currentMatrix.clone(); // Assign the parsed matrix
                        qDebug() << "Successfully parsed Tbc matrix.";
                        vs_parameter.Tcb = cv::Mat::eye(4,4,CV_64F);
                        cv::Mat invR = vs_parameter.Tbc(cv::Rect(0, 0, 3, 3)).t();  // 旋转矩阵的逆等于其转置
                        cv::Mat invt = -invR * vs_parameter.Tbc(cv::Rect(3, 0, 1, 3));
                        invR.copyTo(vs_parameter.Tcb(cv::Rect(0, 0, 3, 3)));
                        invt.copyTo(vs_parameter.Tcb(cv::Rect(3, 0, 1, 3)));
                    }
                    currentState = ReadingGeneral; // Reset state
                }
            }
        }
    } catch (const std::exception& e) {
        qCritical() << "Critical Error: An unexpected exception occurred during CSV parsing:" << e.what();
        overall_success = false;
    } catch (...) {
        qCritical() << "Critical Error: An unknown exception occurred during CSV parsing.";
        overall_success = false;
    }

    file.close();

    if (overall_success) {
        vs_parameter.is_initialized = true;
    }

    return overall_success;
}


void VisualServoingController::write_to_excel(const Eigen::MatrixXd& data, ofstream& oFile)
{
    for (int i = 0; i < data.rows(); ++i) {
        for (int j = 0; j < data.cols(); ++j) {
            oFile << data(i, j) << '\t';
        }
        oFile << endl;
    }
}

Eigen::MatrixXd VisualServoingController::cvMatToEigenMatrix(const cv::Mat& cvMat) {
    // 确保 cvMat 是 double 类型
    CV_Assert(cvMat.type() == CV_64F);

    Eigen::MatrixXd eigenMat(cvMat.rows, cvMat.cols);
    for (int i = 0; i < cvMat.rows; ++i) {
        for (int j = 0; j < cvMat.cols; ++j) {
            eigenMat(i, j) = cvMat.at<double>(i, j);
        }
    }
    return eigenMat;
}

cv::Mat VisualServoingController::eigenMatrixToCvMat(const Eigen::MatrixXd& eigenMat) {
    cv::Mat cvMat(eigenMat.rows(), eigenMat.cols(), CV_64F);
    for (int i = 0; i < eigenMat.rows(); ++i) {
        for (int j = 0; j < eigenMat.cols(); ++j) {
            cvMat.at<double>(i, j) = eigenMat(i, j);
        }
    }
    return cvMat;
}
