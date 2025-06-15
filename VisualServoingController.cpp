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

bool VisualServoingController::read_vs_parameter(QString location) {
    // 定义默认的 camera_intrinsic 参数，使用表格中的数值
    camera_intrinsic default_camera_params = {
        0.3, // c_u
        0.4, // c_v
        2.0, // Z_f
        6.2, // D_f_k_uv
        1.0  // R_f
    };

    // 临时变量，用于存储从文件读取的信息，使用表格中的数值作为默认值
    VS_Parameter temp_param = {
        1600,  // resolution_x 默认值
        1200,  // resolution_y 默认值
        0.03,  // lambda 默认值
        0.001, // epsilon 默认值
        10.0,  // control_rate 默认值
        default_camera_params,
        "E:/QT/Microscopic_Visual_Servoing/resources/data", // resource_location 默认值
        "image_desired.png", // image_desired_name 默认值
        (cv::Mat_<double>(4,4) << 1, 0, 0, 122,
         0, 1, 0, 40,
         0, 0, 1, 30,
         0, 0, 0, 1),
        // Tbc 默认值 (4x4 单位矩阵，使用 CSV 中的数值)
        (cv::Mat_<double>(4,4) << 0, 0, 1, 0.2,
         0, -1, 0, 20,
         1, 0, 0, 20,
         0, 0, 0, 1)
    };

    QFile file(location);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Error: Could not open file:" << location << file.errorString();
        vs_parameter = temp_param;
        return false;
    }

    QTextStream in(&file);
    bool overall_success = true; // 标志整体读取是否成功

    // Current parsing state to handle multi-line data
    enum ParsingState {
        ReadingGeneral,
        ReadingPoseDesired,
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
                        temp_param.resolution_x = parts.at(1).trimmed().toInt(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert resolution_x to int for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid resolution_x format for line:" << line << ". Using default."; }
                } else if (paramName == "resolution_y") {
                    if (parts.size() >= 2) {
                        temp_param.resolution_y = parts.at(1).trimmed().toInt(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert resolution_y to int for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid resolution_y format for line:" << line << ". Using default."; }
                } else if (paramName == "lambda") {
                    if (parts.size() >= 2) {
                        temp_param.lambda = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert lambda to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid lambda format for line:" << line << ". Using default."; }
                } else if (paramName == "epsilon") {
                    if (parts.size() >= 2) {
                        temp_param.epsilon = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert epsilon to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid epsilon format for line:" << line << ". Using default."; }
                } else if (paramName == "control_rate") {
                    if (parts.size() >= 2) {
                        temp_param.control_rate = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert control_rate to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid control_rate format for line:" << line << ". Using default."; }
                } else if (paramName == "c_u") { // Handle individual camera params
                    if (parts.size() >= 2) {
                        temp_param.camera_parameters.c_u = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert c_u to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid c_u format for line:" << line << ". Using default."; }
                } else if (paramName == "c_v") {
                    if (parts.size() >= 2) {
                        temp_param.camera_parameters.c_v = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert c_v to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid c_v format for line:" << line << ". Using default."; }
                } else if (paramName == "Z_f") {
                    if (parts.size() >= 2) {
                        temp_param.camera_parameters.Z_f = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert Z_f to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid Z_f format for line:" << line << ". Using default."; }
                } else if (paramName == "D_f_k_uv") { // Updated to match exact CSV name
                    if (parts.size() >= 2) {
                        temp_param.camera_parameters.D_f_k_uv = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert D_f_k_uv to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid D_f_k_uv format for line:" << line << ". Using default."; }
                } else if (paramName == "R_f") {
                    if (parts.size() >= 2) {
                        temp_param.camera_parameters.R_f = parts.at(1).trimmed().toDouble(&ok);
                        if (!ok) { qWarning() << "Warning: Failed to convert R_f to double for line:" << line << ". Using default."; }
                    } else { qWarning() << "Warning: Invalid R_f format for line:" << line << ". Using default."; }
                } else if (paramName == "resource_location") { // Matched exact key from latest CSV
                    if (parts.size() >= 2) {
                        QString value = parts.at(1).trimmed();
                        while (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                            value = value.mid(1, value.length() - 2);
                        }
                        value = value.replace("\"\"", "\"");
                        temp_param.resource_location = value;
                    } else { qWarning() << "Warning: Invalid resource_location format for line:" << line << ". Using default."; }
                } else if (paramName == "image_rgb_desired_name") { // Matched exact key from latest CSV
                    if (parts.size() >= 2) {
                        QString value = parts.at(1).trimmed();
                        while (value.length() >= 2 && value.startsWith("\"") && value.endsWith("\"")) {
                            value = value.mid(1, value.length() - 2);
                        }
                        value = value.replace("\"\"", "\"");
                        temp_param.image_desired_name = value;
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
                            bool row_parse_ok = true;
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
                            temp_param.pose_desired = cv::Mat::eye(4,4,CV_64F); // Reset to default
                            overall_success = false;
                        }
                    } // If parts.size() == 1, then it's just the key, and matrix data starts next line (handled by subsequent state)
                } else if (paramName == "Tbc") {
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
                            bool row_parse_ok = true;
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
                            temp_param.Tbc = cv::Mat::eye(4,4,CV_64F); // Reset to default
                            overall_success = false;
                        }
                    } // If parts.size() == 1, then it's just the key, and matrix data starts next line (handled by subsequent state)
                }
                else if (paramName == "max_iteration") {
                    if (parts.size() >= 2) {
                        temp_param.max_iteration = parts.at(1).trimmed().toInt(&ok);
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
                    if (currentState == ReadingPoseDesired) temp_param.pose_desired = cv::Mat::eye(4,4,CV_64F); // Reset to default
                    else if (currentState == ReadingTbc) temp_param.Tbc = cv::Mat::eye(4,4,CV_64F); // Reset to default
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
                        temp_param.pose_desired = currentMatrix.clone(); // Assign the parsed matrix
                        qDebug() << "Successfully parsed pose_desired matrix.";
                    } else if (currentState == ReadingTbc) {
                        temp_param.Tbc = currentMatrix.clone(); // Assign the parsed matrix
                        qDebug() << "Successfully parsed Tbc matrix.";
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
        vs_parameter = temp_param;
    } else {
        vs_parameter = temp_param; // 即使有警告，也保留部分解析的数据和默认值。
    }

    return overall_success;
}

