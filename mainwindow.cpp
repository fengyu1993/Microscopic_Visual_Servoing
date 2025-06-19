#include <QDebug>
#include "mainwindow.h"
#include "ui_mainwindow.h"


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
    // 创建图表曲线系列
    chart = new QChart();
    chart->legend()->hide();
    chart->setTitle("Feature error");

    BlueSeries = new QLineSeries();
    BlueSeries->setColor(Qt::blue);
    RedSeries = new QLineSeries();
    RedSeries->setColor(Qt::red);
    GreenSeries = new QLineSeries();
    GreenSeries->setColor(Qt::green);

    chart->addSeries(BlueSeries);
    chart->createDefaultAxes();
    ui->charts_feature_error->setChart(chart);
    ui->charts_feature_error->setRenderHint(QPainter::Antialiasing); // 抗锯齿

}

void MainWindow::setupConnections()
{
    connect(ui->pushButton_Start, &QPushButton::clicked, this, &MainWindow::onSystemStart);
    connect(ui->pushButton_Stop, &QPushButton::clicked, this, &MainWindow::onSystemStop);

    connect(this-> m_visualServoingController->m_robot, &ParallelPlatform::sig_updateRobotData, this, &MainWindow::updateRobotStatus);
    connect(this->m_visualServoingController->m_camera, &BaslerCameraControl::sigCurrentImage, [=](QImage img){
        QPixmap pix = QPixmap::fromImage(img).transformed(m_matrix);
        ui->label_pic->setPixmap(pix.scaled(ui->label_pic->size(), Qt::KeepAspectRatio));
    });
    connect(this->m_visualServoingController, &VisualServoingController::systemStatusChanged, this, &MainWindow::updateSystemStatus);
    connect(this->m_visualServoingController, &VisualServoingController::updateVisualServoingData, this, &MainWindow::updateVSVisualizationData);

}

void MainWindow::getdateSeries(QLineSeries *series, double t, double pos, double&  minY, double&  maxY)
{
    // 添加新的数据点
    series->append(t, pos);

    // 增量更新最小值和最大值
    if (series->count() == 1) { // 初始化第一个点的情况
        minY = pos*0.8;
        maxY = pos*1.2;
    } else {
        minY = std::min(minY, pos)*0.8;
        maxY = std::max(maxY, pos)*1.2;
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

void MainWindow::updateVSVisualizationData(const QVariantMap& visData)
{
    if(visData.contains("loop_time")) {
        ui->LoopTimeValue->setText(QString::number(visData.value("loop_time").toDouble()));
    }
    if(visData.contains("feature_error")) {
        ui->FeatureErrorValue->setText(QString::number(visData.value("feature_error").toDouble()));
    }

    double time = visData.value("loop_time").toDouble() / 1000.0;
    double cost = visData.value("feature_error").toDouble();
    double Y_min, Y_max;
    getdateSeries(BlueSeries, time, cost, Y_min, Y_max);
    updateChart(ui->charts_feature_error, time, Y_min, Y_max);
}

void MainWindow::initializeSystem()
{
    // 初始化系统组件
    if(!m_visualServoingController->initializeSystem()) {
        QMessageBox::critical(this, "错误", "系统初始化失败!");
        return;
    }
    // 更新UI状态
    ui->label_CameraStatus->setText("Connected");
    ui->label_RobotStatus->setText("Connected");
    ui->label_SystemStatus->setText("Successfully initialized");
}

void MainWindow::onSystemStart()
{
    if(m_systemRunning) return;

    // 启动图像采集
    on_pushButton_Connect_clicked(true);
    on_pushButton_StartCamera_clicked(true);

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
    statusBar()->showMessage("系统已启动", 3000);
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
    statusBar()->showMessage("系统已停止", 3000);
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
    if(this->m_visualServoingController->m_camera->saveDesiredImage()){
        qDebug() << "saveDesiredImage Successful";
    }
    else{
        qDebug() << "saveDesiredImage fail";
    }
}

