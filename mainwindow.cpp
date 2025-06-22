#include <QDebug>
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_systemRunning(false),
    m_systemPaused(false)
{
    ui->setupUi(this);

    this->m_visualServoingController = new VisualServoingController();
    QThread* thread_visual_servoing = new QThread();
    m_visualServoingController->moveToThread(thread_visual_servoing);
    thread_visual_servoing->start();

    // qDebug() << "MainWindow thread ID:" << QThread::currentThreadId();

    // 初始化UI
    this->initUI();
    // 设置连接
    setupConnections();
    // 初始化系统
    initializeSystem();
}

MainWindow::~MainWindow()
{
    if(m_systemRunning) {
        m_visualServoingController->stopServoing();
    }
    // 安全释放相机资源
    if (this->m_visualServoingController->m_camera) {
        this->m_visualServoingController->m_camera->deleteAll(); // 关闭相机（假设存在该方法）
    }
    delete this->m_visualServoingController;
    delete ui;
}

void MainWindow::initUI()
{
    this->linearVelocityStep = 5000.0;
    this->angularVelocityStep = 5000.0;
    this->linearPositionStep = 5000.0;
    this->angularPositionStep = 5000.0;
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    this->ui->linear_velocity_step->setText(QString::number(this->linearVelocityStep));
    this->ui->angular_velocity_step->setText(QString::number(this->angularVelocityStep));
    this->ui->linear_position_step->setText(QString::number(this->linearPositionStep));
    this->ui->angular_position_step->setText(QString::number(this->angularPositionStep));
    this->ui->Robot_Control->setCurrentIndex(0);
    ui->pushButton_Start->setEnabled(true);
    ui->pushButton_Stop->setEnabled(false);
    ui->Calibration_Step_1->setEnabled(false);
    ui->Calibration_Step_2->setEnabled(false);
    ui->Calibration_Step_3->setEnabled(false);
    ui->Calibration_Step_4->setEnabled(false);
    ui->Calibration_Step_5->setEnabled(false);
    ui->Calibration_Step_6->setEnabled(false);
    ui->Calibration_Step_7->setEnabled(false);
    ui->Calibration_Step_8->setEnabled(false);
    ui->Calibration_Step_9->setEnabled(false);
    // 创建特征误差图表曲线系列
    chartFeatureError = new QChart();
    chartFeatureError->legend()->hide();
    chartFeatureError->setTitle("Feature error");
    BlueSeries_FeatureError = new QLineSeries();
    BlueSeries_FeatureError->setColor(Qt::blue);
    chartFeatureError->addSeries(BlueSeries_FeatureError);
    chartFeatureError->createDefaultAxes();
    ui->charts_feature_error->setChart(chartFeatureError);
    ui->charts_feature_error->setRenderHint(QPainter::Antialiasing); // 抗锯齿
    cost_min = 0.0;
    cost_max = 0.0;
    // 创建线速度图表曲线系列
    chartLinarVelocity = new QChart();
    chartLinarVelocity->legend()->hide();
    chartLinarVelocity->setTitle("Linar Velocity");
    RedSeries_Vx = new QLineSeries(); RedSeries_Vx->setColor(Qt::red);
    GreenSeries_Vy = new QLineSeries(); GreenSeries_Vy->setColor(Qt::green);
    BlueSeries_Vz = new QLineSeries();  BlueSeries_Vz->setColor(Qt::blue);
    chartLinarVelocity->addSeries(RedSeries_Vx);
    chartLinarVelocity->addSeries(GreenSeries_Vy);
    chartLinarVelocity->addSeries(BlueSeries_Vz);
    chartLinarVelocity->createDefaultAxes();
    ui->charts_linear_velocity->setChart(chartLinarVelocity);
    ui->charts_linear_velocity->setRenderHint(QPainter::Antialiasing); // 抗锯齿
    linear_velocity_min = 0.0;
    linear_velocity_max = 0.0;
    // 创建角速度图表曲线系列
    chartAngularVelocity = new QChart();
    chartAngularVelocity->legend()->hide();
    chartAngularVelocity->setTitle("Angular Velocity");
    CyanSeries_Wx = new QLineSeries(); CyanSeries_Wx->setColor(Qt::cyan);
    YellowSeries_Wy = new QLineSeries(); YellowSeries_Wy->setColor(Qt::yellow);
    MagentaSeries_Wz = new QLineSeries(); MagentaSeries_Wz->setColor(Qt::magenta);
    chartAngularVelocity->addSeries(CyanSeries_Wx);
    chartAngularVelocity->addSeries(YellowSeries_Wy);
    chartAngularVelocity->addSeries(MagentaSeries_Wz);
    chartAngularVelocity->createDefaultAxes();
    ui->charts_angular_velocity->setChart(chartAngularVelocity);
    ui->charts_angular_velocity->setRenderHint(QPainter::Antialiasing); // 抗锯齿
    angular_velocity_min = 0.0;
    angular_velocity_max = 0.0;
    // 创建锐度图表曲线系列
    chartSharpness = new QChart();
    chartSharpness->legend()->hide();
    chartSharpness->setTitle("Sharpness");
    BlueSeries_Sharpness = new QLineSeries();
    BlueSeries_Sharpness->setColor(Qt::blue);
    chartSharpness->addSeries(BlueSeries_Sharpness);
    chartSharpness->createDefaultAxes();
    ui->charts_sharpness->setChart(chartSharpness);
    ui->charts_sharpness->setRenderHint(QPainter::Antialiasing); // 抗锯齿
    sharpness_min = 0.0;
    sharpness_max = 0.0;
    // 模式选择
    ui->radioButton_VisualServoing->setChecked(true);
    ui->radioButton_Sharpness->setChecked(false);
    ui->radioButton_Calibration->setChecked(false);
}

void MainWindow::setupConnections()
{
    connect(ui->pushButton_Start, &QPushButton::clicked, this, &MainWindow::onSystemStart);
    connect(ui->pushButton_Stop, &QPushButton::clicked, this, &MainWindow::onSystemStop);

    connect(this-> m_visualServoingController->m_robot, &ParallelPlatform::sig_updateRobotData, this, &MainWindow::updateRobotStatus);
    connect(this-> m_visualServoingController->m_camera, &BaslerCameraControl::sigCurrentImage, this, &MainWindow::updateVisualServoingImage);
    connect(this-> m_visualServoingController, &VisualServoingController::sigCalibrationImage, this, &MainWindow::updateCalibrationImage);
    connect(this->m_visualServoingController, &VisualServoingController::systemStatusChanged, this, &MainWindow::updateSystemStatus);
    connect(this->m_visualServoingController, &VisualServoingController::updateVisualServoingData, this, &MainWindow::updateVSVisualizationData);
}

void MainWindow::getdateSeries(QLineSeries *series, double t, double pos, double&  minY, double&  maxY)
{
    // 添加新的数据点
    series->append(t, pos);

    // 增量更新最小值和最大值
    if (series->count() == 1) { // 初始化第一个点的情况
        minY = pos;
        maxY = pos;
    } else {
        minY = std::min(minY, pos);
        maxY = std::max(maxY, pos);
    }
}

void MainWindow::updateChart(QChartView *chartView, double t, double minY, double  maxY)
{
    // 更新x轴范围
    auto axesX = chartView->chart()->axes(Qt::Horizontal);
    if (!axesX.isEmpty()) {
        QValueAxis *axisX = qobject_cast<QValueAxis*>(axesX.first());
        if (axisX) {
            axisX->setRange(0, t); // 更新x轴范围以适应新数据
        }
    }

    // 更新y轴范围
    auto axesY = chartView->chart()->axes(Qt::Vertical);
    if (!axesY.isEmpty()) {
        QValueAxis *axisY = qobject_cast<QValueAxis*>(axesY.first());
        if (axisY) {
            const qreal margin = 5;
            // 设置y轴范围
            axisY->setRange(minY - margin, maxY + margin);
        }
    }
}

void MainWindow::updateSystemStatus(const QString &status)
{
    ui->label_SystemStatus->setText(status);
}

 void MainWindow::updateRobotStatus(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose)
{
          // Pose
         ui->current_pos_x->setText(QString::number(current_pose(0)));
         ui->current_pos_y->setText(QString::number(current_pose(1)));
         ui->current_pos_z->setText(QString::number(current_pose(2)));
         ui->current_pos_rx->setText(QString::number(current_pose(3)));
         ui->current_pos_ry->setText(QString::number(current_pose(4)));
         ui->current_pos_rz->setText(QString::number(current_pose(5)));
         // Velocity
         ui->current_vec_x->setText(QString::number(target_velocity(0)));
         ui->current_vec_y->setText(QString::number(target_velocity(1)));
         ui->current_vec_z->setText(QString::number(target_velocity(2)));
         ui->current_vec_rx->setText(QString::number(target_velocity(3)));
         ui->current_vec_ry->setText(QString::number(target_velocity(4)));
         ui->current_vec_rz->setText(QString::number(target_velocity(5)));
         // Position
         ui->current_joint_pos_1->setText(QString::number(current_positions(0)));
         ui->current_joint_pos_2->setText(QString::number(current_positions(1)));
         ui->current_joint_pos_3->setText(QString::number(current_positions(2)));
         ui->current_joint_pos_4->setText(QString::number(current_positions(3)));
         ui->current_joint_pos_5->setText(QString::number(current_positions(4)));
         ui->current_joint_pos_6->setText(QString::number(current_positions(5)));

}

void MainWindow::updateVisualServoingImage(QImage img)
{
    if(this->m_visualServoingController->getMode() == MODE_VISUAL_SERVOING){
        QPixmap pix = QPixmap::fromImage(img).transformed(m_matrix);
        ui->label_pic->setPixmap(pix.scaled(ui->label_pic->size(), Qt::KeepAspectRatio));
    }
}

void MainWindow::updateCalibrationImage(QImage img)
{
    if(this->m_visualServoingController->getMode() == MODE_CALIBRATION){
        QPixmap pix = QPixmap::fromImage(img).transformed(m_matrix);
        ui->label_pic->setPixmap(pix.scaled(ui->label_pic->size(), Qt::KeepAspectRatio));
    }
}

void MainWindow::updateVSVisualizationData(const QVariantMap& visData)
{
    if (visData.contains("loop_time") && visData.contains("feature_error") ) {
        double time = visData.value("loop_time").toDouble() / 1000.0;
        double cost = visData.value("feature_error").toDouble();
        double vx = visData.value("velocity_vx").toDouble();
        double vy = visData.value("velocity_vy").toDouble();
        double vz = visData.value("velocity_vz").toDouble();
        double wx = visData.value("velocity_wx").toDouble();
        double wy = visData.value("velocity_wy").toDouble();
        double wz = visData.value("velocity_wz").toDouble();

        getdateSeries(BlueSeries_FeatureError, time, cost, cost_min, cost_max);
        updateChart(ui->charts_feature_error, time, cost_min, cost_max);

        getdateSeries(RedSeries_Vx, time, vx, linear_velocity_min, linear_velocity_max);
        getdateSeries(GreenSeries_Vy, time, vy, linear_velocity_min, linear_velocity_max);
        getdateSeries(BlueSeries_Vz, time, vz, linear_velocity_min, linear_velocity_max);
        updateChart(ui->charts_linear_velocity, time, linear_velocity_min, linear_velocity_max);

        getdateSeries(CyanSeries_Wx, time, wx, angular_velocity_min, angular_velocity_max);
        getdateSeries(YellowSeries_Wy, time, wy, angular_velocity_min, angular_velocity_max);
        getdateSeries(MagentaSeries_Wz, time, wz, angular_velocity_min, angular_velocity_max);
        updateChart(ui->charts_angular_velocity, time, angular_velocity_min, angular_velocity_max);
    }

    if (visData.contains("loop_time") && visData.contains("sharpness") ) {
        double time = visData.value("loop_time").toDouble() / 1000.0;
        double sharpness = visData.value("sharpness").toDouble();
        getdateSeries(BlueSeries_Sharpness, time, sharpness, sharpness_min, sharpness_max);
        updateChart(ui->charts_sharpness, time, sharpness_min, sharpness_max);
    }
}

void MainWindow::initializeSystem()
{
    // 初始化系统组件
    if(!m_visualServoingController->initializeSystem()) {
        QMessageBox::critical(this, "错误", "系统初始化失败!");
        return;}
    else{
        displayDesiredImage();
    }
    // 更新UI状态
    ui->label_CameraStatus->setText("Connected");
    ui->label_RobotStatus->setText("Connected");
    ui->label_SystemStatus->setText("Successfully initialized");
 }

 void MainWindow::displayDesiredImage()
 {
     QImage qImage = m_visualServoingController->m_camera->cvMatToQImage
                     (m_visualServoingController->m_algorithm_DMVS->image_gray_desired_);
     QPixmap pixmap = QPixmap::fromImage(qImage);
    ui->label_pic_desired->setPixmap(pixmap.scaled(ui->label_pic_desired->size(), Qt::KeepAspectRatio));
 }

void MainWindow::onSystemStart()
{
    if(m_systemRunning) return;

    // 启动图像采集
    on_pushButton_Connect_clicked(true);
    on_pushButton_StartCamera_clicked(true);
    // 判断模式
    if (ui->radioButton_VisualServoing->isChecked()) {
        this->m_visualServoingController ->setMode(MODE_VISUAL_SERVOING);
    } else if (ui->radioButton_Sharpness->isChecked()) {
        this->m_visualServoingController ->setMode(MODE_SHARPNESS);
    } else if (ui->radioButton_Calibration->isChecked()) {
        this->m_visualServoingController ->setMode(MODE_CALIBRATION);
        ui->Calibration_Step_1->setEnabled(true);
    } else {
        qDebug() << "没有模式被选中";
    }

    // 启动机器状态更新
    this->m_visualServoingController->m_robot->startupdate();
    // 启动视觉伺服
    m_visualServoingController->startServoing();
    m_systemRunning = true;
    m_systemPaused = false;

    // 更新UI状态
    ui->pushButton_Start->setEnabled(false);
    ui->pushButton_Stop->setEnabled(true);
    ui->StartUpdate->setEnabled(false);
    ui->StopUpdate->setEnabled(false);
    ui->connect->setEnabled(false);
    ui->disconnect->setEnabled(false);
    ui->pushButton_Connect->setEnabled(false);
    ui->pushButton_StartCamera->setEnabled(false);
    ui->getState->setEnabled(false);
    ui->FindReferance->setEnabled(false);
    statusBar()->showMessage("系统已启动", 300);
}

void MainWindow::onSystemStop()
{
    if(!m_systemRunning) return;

    // 启动图像采集
    on_pushButton_StartCamera_clicked(false);
    on_pushButton_Connect_clicked(false);
    // 启动机器状态更新
    this->m_visualServoingController->m_robot->stopupdate();

    // 停止视觉伺服
    m_visualServoingController->stopServoing();
    m_systemRunning = false;
    m_systemPaused = false;

    // 更新UI状态
    ui->pushButton_Start->setEnabled(true);
    ui->pushButton_Stop->setEnabled(false);
    ui->StartUpdate->setEnabled(true);
    ui->StopUpdate->setEnabled(true);
    ui->connect->setEnabled(true);
    ui->disconnect->setEnabled(true);
    ui->pushButton_Connect->setEnabled(true);
    ui->pushButton_StartCamera->setEnabled(true);
    ui->getState->setEnabled(true);
    ui->FindReferance->setEnabled(true);
    statusBar()->showMessage("系统已停止", 300);
}


void MainWindow::on_pushButton_StartCamera_clicked(bool checked)
{
    if(checked) {// 开始采集
        this->m_visualServoingController->m_camera->StartAcquire();
        ui->pushButton_StartCamera->setText("StopCamera");// 结束采集
        qDebug() << "开始采集";
    } else {
        this->m_visualServoingController->m_camera->StopAcquire();
        ui->pushButton_StartCamera->setText("StartCamera");// 开始采集
    }
}


void MainWindow::on_pushButton_Connect_clicked(bool checked)
{
    if(checked)
    {
        const auto cameras = this->m_visualServoingController->m_camera->cameras(); // 获取相机列表

        if (!cameras.isEmpty()) { // 检查是否至少存在一个相机
            const QString& firstCameraSN = cameras.first(); // 获取第一个相机序列号
            if (!this->m_visualServoingController->m_camera->openCamera(firstCameraSN)) { // 尝试打开相机
                qDebug() << "成功打开相机: " << firstCameraSN;
                ui->label_pic->setText("成功打开相机！");
            } else {
                qCritical() << "无法打开相机: " << firstCameraSN;
                ui->label_pic->setText("无法打开相机!");
            }
        } else {
            qWarning() << "未检测到任何相机！";
            ui->label_pic->setText("未检测到相机，请连接设备！");
        }
        ui->pushButton_Connect->setText("Disconnect");// 断开连接
    }
    else{
        this->m_visualServoingController->m_camera->closeCamera();
        ui->label_pic->setText("相机已断开！");
        ui->pushButton_Connect->setText("ConnectCamera");// 连接相机
    }
}


void MainWindow::on_expose_clicked()
{
    bool ok;
    double expouse =  ui->lineEdit_expose->text().toInt(&ok);
    if(!ok){
        QMessageBox::warning(this, "Error", "input is error !!");
    }
    else{
        this->m_visualServoingController->m_camera->setExposureTime(expouse);
    }
}


void MainWindow::on_horizontalSliderExpose_sliderMoved(int position)
{
    // qDebug() << position;
    int exposeSize = 10000;
    ui->horizontalSliderExpose->setRange(0,exposeSize);
    // int exposeValue = double(position * exposeSize) / 100;
    if(position <= 10)
    {
        position = 10;
    }
    ui->lineEdit_expose->setText(QString::number(position,'f',0));
    on_expose_clicked();
}


void MainWindow::on_connect_clicked()
{
    this->m_visualServoingController->m_robot->connect();
}

void MainWindow::on_disconnect_clicked()
{
    this->m_visualServoingController->m_robot->disconnect();
}

void MainWindow::on_getState_clicked()
{
    bool result;
    Eigen::VectorXd current_positions;
    Eigen::VectorXd target_velocity;
    result = this->m_visualServoingController->m_robot->getTaskPositions(current_positions);
    result = this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    if (result)
    {
        // position
        ui->current_pos_x->setText(QString::number(current_positions(0)));
        ui->current_pos_y->setText(QString::number(current_positions(1)));
        ui->current_pos_z->setText(QString::number(current_positions(2)));
        ui->current_pos_rx->setText(QString::number(current_positions(3)));
        ui->current_pos_ry->setText(QString::number(current_positions(4)));
        ui->current_pos_rz->setText(QString::number(current_positions(5)));
        // Velocity
        ui->current_vec_x->setText(QString::number(target_velocity(0)));
        ui->current_vec_y->setText(QString::number(target_velocity(1)));
        ui->current_vec_z->setText(QString::number(target_velocity(2)));
        ui->current_vec_rx->setText(QString::number(target_velocity(3)));
        ui->current_vec_ry->setText(QString::number(target_velocity(4)));
        ui->current_vec_rz->setText(QString::number(target_velocity(5)));
    }
    else
    {
        qDebug()<< "error";
    }
}

void MainWindow::on_FindReferance_clicked()
{
    this->m_visualServoingController->m_robot->findReferance();
}



void MainWindow::on_Stop_clicked()
{
    this->m_visualServoingController->m_robot->setTargetVelocity(Eigen::VectorXd::Zero(6));
    this->m_visualServoingController->m_robot->stop();
}

void MainWindow::on_linear_velocity_step_editingFinished()
{
    QString text = this->ui->linear_velocity_step->text();
    this->linearVelocityStep = text.toDouble();

    // qDebug() << "linear_velocity_step: " << this->linearVelocityStep;
}


void MainWindow::on_angular_velocity_step_editingFinished()
{
    QString text = this->ui->angular_velocity_step->text();
    this->angularVelocityStep = text.toDouble();

    // qDebug() << "angular_velocity_step: " << this->angularVelocityStep;
}


void MainWindow::on_Vx_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(0) += this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Vx_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(0) -= this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Vy_p_clicked()
{   
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(1) += this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Vy_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(1) -= this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);
    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Vz_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(2) += this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Vz_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(2) -= this->linearVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wx_p_clicked()
{    
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(3) += this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);
    std::stringstream ss;
    ss << target_velocity;
    qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wx_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(3) -= this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    std::stringstream ss;
    ss << target_velocity;
    qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wy_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(4) += this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wy_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(4) -= this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wz_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(5) += this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Wz_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 0;
    Eigen::VectorXd target_velocity;
    this->m_visualServoingController->m_robot->getTargetVelocity(target_velocity);
    target_velocity(5) -= this->angularVelocityStep;
    this->m_visualServoingController->m_robot->setTargetVelocity(target_velocity);

    // std::stringstream ss;
    // ss << target_velocity;
    // qDebug() << "targetVelocity:\n" << ss.str().c_str();
}


void MainWindow::on_Robot_Control_tabBarClicked(int index)
{
    if(index == 0){
        this->m_visualServoingController->m_robot->controlMode_ = 0;//Velocity
    }
    else if(index == 1){
        this->m_visualServoingController->m_robot->controlMode_ = 1;//Position
    }

    // qDebug() << "controlMode_:\n" << this->m_visualServoingController->m_robot->controlMode_;
}


void MainWindow::on_linear_position_step_editingFinished()
{
    QString text = this->ui->linear_position_step->text();
    this->linearPositionStep = text.toDouble();

    qDebug() << "linear_position_step: " << this->linearPositionStep;
}


void MainWindow::on_angular_position_step_editingFinished()
{
    QString text = this->ui->angular_position_step->text();
    this->angularPositionStep = text.toDouble();

    qDebug() << "angular_position_step: " << this->angularPositionStep;
}


void MainWindow::on_Stop_2_clicked()
{
    this->m_visualServoingController->m_robot->stop();
}


void MainWindow::on_Px_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(0) += linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Px_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(0) -= linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Py_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(1) += linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Py_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(1) -= linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Pz_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(2) += linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Pz_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(2) -= linearPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rx_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(3) += angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rx_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(3) -= angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Ry_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(4) += angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Ry_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(4) -= angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rz_p_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(5) += angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}


void MainWindow::on_Rz_n_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 1;
    Eigen::VectorXd target_positions;
    bool result = this->m_visualServoingController->m_robot->getTaskPositions(target_positions);
    if(result){
        target_positions(5) -= angularPositionStep;
        this->m_visualServoingController->m_robot->setTargetPose(target_positions);
    }
}




void MainWindow::on_MoveToZero_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 3;
}


void MainWindow::on_StartUpdate_clicked()
{
    this->m_visualServoingController->m_robot->startupdate();
}


void MainWindow::on_StopUpdate_clicked()
{
    this->m_visualServoingController->m_robot->stopupdate();
}


void MainWindow::on_MoveToZero_2_clicked()
{
    this->m_visualServoingController->m_robot->controlMode_ = 3;
}



void MainWindow::on_SaveDesiredImage_clicked()
{
    cv::Mat img_64f = this->m_visualServoingController->m_camera->saveDesiredImage();
    this->m_visualServoingController->m_algorithm_DMVS->set_image_gray_desired(img_64f);
    displayDesiredImage();
}


void MainWindow::on_radioButton_VisualServoing_clicked(bool checked)
{
    if(checked){
        this->m_visualServoingController ->setMode(MODE_VISUAL_SERVOING);
    }
}


void MainWindow::on_radioButton_Sharpness_clicked(bool checked)
{
    if(checked){
        this->m_visualServoingController ->setMode(MODE_SHARPNESS);
    }
}


void MainWindow::on_radioButton_Calibration_clicked(bool checked)
{
    if(checked){
        this->m_visualServoingController ->setMode(MODE_CALIBRATION);
    }
}

void MainWindow::on_circle_du_p_clicked()
{
    this->circle_du++;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_circle_du_n_clicked()
{
    this->circle_du--;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_circle_dv_p_clicked()
{
    this->circle_dv++;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_circle_dv_n_clicked()
{
    this->circle_dv--;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_circle_dr_p_clicked()
{
    this->circle_dr++;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_circle_dr_n_clicked()
{
    this->circle_dr--;
    this->m_visualServoingController->setCircleDxDyDr(this->circle_du, this->circle_dv, this->circle_dr);
}


void MainWindow::on_Calibration_Step_1_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第一步：请找到焦平面");
        ui->Calibration_Step_1->setText("Finish");
        this->m_visualServoingController->setStepCalibration(1);
    }
    else{
        ui->Calibration_Step_1->setEnabled(false);
        ui->Calibration_Step_2->setEnabled(true);
    }
}

    // 3:   沿y移动采点
    // 4:   Z轴向上方向移动
    // 5:   沿xy移动采点
    // 6:   Z轴向下方向移动
    // 7:   xy移动采点
    // 8:   回到焦平面后,  绕Z轴转动并采点
    // 9:    计算参数

void MainWindow::on_Calibration_Step_2_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第二步：请仅沿x方向移动并采点");
        ui->Calibration_Step_2->setText("Finish");
        this->m_visualServoingController->setStepCalibration(2);
    }
    else{
        ui->Calibration_Step_2->setEnabled(false);
        ui->Calibration_Step_3->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_3_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_3->setText("Finish");
        this->m_visualServoingController->setStepCalibration(3);
    }
    else{
        ui->Calibration_Step_3->setEnabled(false);
        ui->Calibration_Step_4->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_4_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_4->setText("Finish");
        this->m_visualServoingController->setStepCalibration(4);
    }
    else{
        ui->Calibration_Step_4->setEnabled(false);
        ui->Calibration_Step_5->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_5_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_5->setText("Finish");
        this->m_visualServoingController->setStepCalibration(5);
    }
    else{
        ui->Calibration_Step_5->setEnabled(false);
        ui->Calibration_Step_6->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_6_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_6->setText("Finish");
        this->m_visualServoingController->setStepCalibration(6);
    }
    else{
        ui->Calibration_Step_6->setEnabled(false);
        ui->Calibration_Step_7->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_7_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_7->setText("Finish");
        this->m_visualServoingController->setStepCalibration(7);
    }
    else{
        ui->Calibration_Step_7->setEnabled(false);
        ui->Calibration_Step_8->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_8_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_8->setText("Finish");
        this->m_visualServoingController->setStepCalibration(8);
    }
    else{
        ui->Calibration_Step_8->setEnabled(false);
        ui->Calibration_Step_9->setEnabled(true);
    }
}


void MainWindow::on_Calibration_Step_9_clicked(bool checked)
{
    if(checked){
        QMessageBox::information(nullptr, "Calibration", "第三步：请仅沿y方向移动并采点");
        ui->Calibration_Step_9->setText("Finish");
        this->m_visualServoingController->setStepCalibration(9);
    }
    else{
        ui->Calibration_Step_9->setEnabled(false);
    }
}


void MainWindow::on_record_calibration_point_clicked()
{
    this->m_visualServoingController->enableflagRecord(true);
}

