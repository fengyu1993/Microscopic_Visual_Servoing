#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTransform>
#include <QMessageBox>
#include <QVariantMap>
#include <QtCharts>

#include "VisualServoingController.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void onSystemStart();

    void onSystemStop();

    void updateSystemStatus(const QString &status);

    void updateRobotStatus(const  Eigen::VectorXd current_positions, const Eigen::VectorXd target_velocity, const Eigen::VectorXd current_pose);

    void updateVSVisualizationData(const QVariantMap& visData);

    void on_pushButton_StartCamera_clicked(bool checked);

    void on_pushButton_Connect_clicked(bool checked);

    void on_expose_clicked();

    void on_horizontalSliderExpose_sliderMoved(int position);

    void on_connect_clicked();

    void on_disconnect_clicked();

    void on_getState_clicked();

    void on_FindReferance_clicked();

    void on_Stop_clicked();

    void on_Vx_p_clicked();

    void on_linear_velocity_step_editingFinished();

    void on_angular_velocity_step_editingFinished();

    void on_Vx_n_clicked();

    void on_Vy_p_clicked();

    void on_Vy_n_clicked();

    void on_Vz_p_clicked();

    void on_Vz_n_clicked();

    void on_Wx_p_clicked();

    void on_Wx_n_clicked();

    void on_Wy_p_clicked();

    void on_Wy_n_clicked();

    void on_Wz_p_clicked();

    void on_Wz_n_clicked();

    void on_Robot_Control_tabBarClicked(int index);

    void on_linear_position_step_editingFinished();

    void on_angular_position_step_editingFinished();

    void on_Stop_2_clicked();

    void on_Px_p_clicked();

    void on_Px_n_clicked();

    void on_Py_p_clicked();

    void on_Py_n_clicked();

    void on_Pz_p_clicked();

    void on_Pz_n_clicked();

    void on_Rx_p_clicked();

    void on_Rx_n_clicked();

    void on_Ry_p_clicked();

    void on_Ry_n_clicked();

    void on_Rz_p_clicked();

    void on_Rz_n_clicked();

    void on_MoveToZero_clicked();

    void on_StartUpdate_clicked();

    void on_StopUpdate_clicked();

    void on_MoveToZero_2_clicked();


    void on_SaveDesiredImage_clicked();

    void on_radioButton_VisualServoing_clicked(bool checked);

    void on_radioButton_Sharpness_clicked(bool checked);

    void on_radioButton_Calibration_clicked(bool checked);

private:
    Ui::MainWindow *ui;
    VisualServoingController* m_visualServoingController = Q_NULLPTR;
    QTransform  m_matrix;
    double linearVelocityStep;
    double angularVelocityStep;
    double linearPositionStep;
    double angularPositionStep;
    // 状态变量
    bool m_systemRunning;
    bool m_systemPaused;
    // 图像显示
    QChart *chartFeatureError;
    QChart *chartLinarVelocity;
    QChart *chartAngularVelocity;
    QChart *chartSharpness;
    QLineSeries *BlueSeries_FeatureError;
    QLineSeries *RedSeries_Vx;
    QLineSeries *GreenSeries_Vy;
    QLineSeries *BlueSeries_Vz;
    QLineSeries *CyanSeries_Wx;
    QLineSeries *YellowSeries_Wy;
    QLineSeries *MagentaSeries_Wz;
    QLineSeries *BlueSeries_Sharpness;
    double cost_min, cost_max, linear_velocity_min, linear_velocity_max,
                angular_velocity_min, angular_velocity_max, sharpness_min, sharpness_max;
    // 函数
    void initUI();
    void setupConnections();
    void initializeSystem();
    void updateChart(QChartView *chartView, double t, double  minY, double  maxY);
    void getdateSeries(QLineSeries *series, double t, double pos, double&  minY, double& maxY);
    void displayDesiredImage();
};
#endif // MAINWINDOW_H
