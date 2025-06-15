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

#include "BaslerCameraControl.h"
#include "ParallelPlatform.h"
#include "direct_microscopic_visual_servoing.h"

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

signals:
    void systemStatusChanged(const QString& status);
    void servoingError(const QString& error);
    void updateVisualServoingData(const QVariantMap& visData);

public:
    BaslerCameraControl* m_camera = Q_NULLPTR;
    ParallelPlatform *m_robot= Q_NULLPTR;
    Direct_Microscopic_Visual_Servoing* m_algorithm_DMVS;
private:
    QTimer* m_controlTimer;
    bool m_isRunning;
    VS_Parameter vs_parameter;

    void executeControlCycle();
};

#endif // VISUALSERVOINGCONTROLLER_H
