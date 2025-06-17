#ifndef BASLERCAMERACONTROL_H
#define BASLERCAMERACONTROL_H

#include <QObject>
#include <QImage>
#include <QTimer>
#include <QDebug>
#include <QThread>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <QMutex>

using namespace Pylon;
using namespace GenApi;

class BaslerCameraControl : public QThread
{
    Q_OBJECT
public:
    explicit BaslerCameraControl();
    ~BaslerCameraControl();

    enum BaslerCameraControl_Type{
        Type_Basler_Freerun, //设置相机的内触发
        Type_Basler_Line1, //设置相机的外触发
        Type_Basler_ExposureTimeAbs, //设置相机的曝光时间
        Type_Basler_GainRaw, //设置相机的增益
        Type_Basler_AcquisitionFrameRateAbs, //设置相机的频率
        Type_Basler_Width, //图片的宽度
        Type_Basler_Height, //图片的高度
        Type_Basler_LineSource, //灯的触发信号
    };

    bool init();
    void deleteAll();
    QStringList cameras();
    int openCamera(QString cameraName);
    int closeCamera();
    bool isOpen();
    void run();

    void setFeatureTriggerSourceType(QString type); // 设置种类
    QString getFeatureTriggerSourceType(); // 获取种类：软触发、外触发等等

    void setFeatureTriggerModeType(bool on); // 设置模式触发
    bool getFeatureTriggerModeType(); // 获取模式触发

    void SetCamera(BaslerCameraControl::BaslerCameraControl_Type index, double tmpValue = 0.0); // 设置各种参数
    double GetCamera(BaslerCameraControl::BaslerCameraControl_Type index); // 获取各种参数
    void setExposureTime(double time); // 设置曝光时间
    int getExposureTime(); // 获取曝光时间
    void setGain(double Gain);
    int getGain();
    void setFrameRate(int value);
    int getFrameRate();
    void autoExposureOnce();


    long GrabImage(QImage& image,int timeout = 2000);

    long StartAcquire(); // 开始采集
    long StopAcquire(); // 结束采集

    void qImageToCvMat(const QImage& qImage, cv::Mat & image);
    QImage cvMatToQImage(const cv::Mat& mat);
    std::shared_ptr<const cv::Mat> getLatestFrameShared() const;
    cv::Mat getLatestFrame();
    bool saveDesiredImage();

signals:
    void sigCameraUpdate(QStringList list);
    void sigSizeChange(QSize size);
    void sigCameraCount(int count);
    void sigCurrentImage(QImage img);

private:
    void UpdateCameraList();
    void CopyToImage(CGrabResultPtr pInBuffer, QImage &OutImage);

private:
    CBaslerUniversalInstantCamera m_basler;
    QStringList m_cameralist;
    QString m_currentMode;
    bool m_isOpenAcquire = false; // 是否开始采集
    bool m_isOpen = false; // 是否打开摄像头
    QSize m_size;
    mutable QMutex m_frameMutex;

public:
    cv::Mat img_cv;
    cv::Mat img_vs;
    QImage img_Q;
};


#endif // BASLERCAMERACONTROL_H
