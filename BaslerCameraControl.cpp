#include "BaslerCameraControl.h"
#include <QDateTime>
#include <QDebug>

BaslerCameraControl::BaslerCameraControl(double hz)
{
    init();
    fps = hz;
    this->m_coarseTimer = new QTimer();
    this->m_coarseTimer->setInterval(1.0 / fps * 1000);
    this-> m_coarseTimer->setTimerType(Qt::CoarseTimer);
    QObject::connect(this->m_coarseTimer, &QTimer::timeout, this, &BaslerCameraControl::grab);
}

BaslerCameraControl::~BaslerCameraControl()
{
    this->m_coarseTimer->stop();
    deleteAll();
}

bool BaslerCameraControl::init()
{
    qDebug() << "BaslerCameraControl: PylonInitialize init" ;

    Pylon::PylonInitialize();

    CTlFactory &TlFactory = CTlFactory::GetInstance();
    TlInfoList_t lstInfo;
    int transportLayerCount = TlFactory.EnumerateTls(lstInfo);
    ITransportLayer * pTl = TlFactory.CreateTl("BaslerGigE");
    DeviceInfoList_t devices;
    int cameraCount = pTl->EnumerateDevices(devices);

    if (cameraCount >= 1){
        TlInfoList_t::const_iterator it;
        for ( it = lstInfo.begin(); it != lstInfo.end(); ++it ) {
            qDebug() << "FriendlyName: " << it->GetFriendlyName() << "FullName: " << it->GetFullName();
            // qDebug() << "VendorName: " << it->GetVendorName() << "DeviceClass: " << it->GetDeviceClass() ;
        }
        UpdateCameraList();
        emit sigCameraCount(transportLayerCount);
        // qDebug() << "transportLayerCount Count: " << transportLayerCount;
        return true;
    }else{
        return false;
    }

}

void BaslerCameraControl::grab()
{
        // cv::namedWindow("window", cv::WINDOW_AUTOSIZE);
        // qDebug() << "Camera thread ID:" << QThread::currentThreadId();
        if(m_isOpenAcquire && m_isOpen)
        {
            QMutexLocker locker(&m_frameMutex);
            GrabImage(this->img_Q);
            if(!this->img_Q.isNull()) {
                emit sigCurrentImage(this->img_Q);
                qImageToCvMat(this->img_Q, img);
                if(img.channels() == 3){
                    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);
                }else{
                    img_gray = img.clone();
                }
                img_gray.convertTo(img_gray, CV_64F, 1.0/255.0);
                // qDebug() << "row: " << img_gray.rows;
                // qDebug() << "col: " << img_gray.cols;
                updateFrame(img_gray);
                // cv::imshow("window", getLatestFrame());
                // cv::waitKey(1);
            }
        }

}
void BaslerCameraControl::deleteAll()
{
    //停止采集
    if(m_isOpenAcquire) {
        StopAcquire();
    }
    //关闭摄像头
    closeCamera();
    //关闭库
    // qDebug() << "BaslerCameraControl deleteAll: PylonTerminate" ;

    Pylon::PylonTerminate();

    qDebug() << "BaslerCameraControl deleteAll: Close" ;
}

QStringList BaslerCameraControl::cameras()
{
    UpdateCameraList();
    return m_cameralist;
}

void BaslerCameraControl::UpdateCameraList()
{
    m_cameralist.clear(); // 清空旧列表

    CTlFactory& TLFactory = CTlFactory::GetInstance();
    ITransportLayer * pTl = TLFactory.CreateTl("BaslerGigE");
    DeviceInfoList_t devices;
    int cameraCount = pTl->EnumerateDevices(devices);
    qDebug() << "Basler Camera Count: " << cameraCount;

    CInstantCameraArray cameraArray(devices.size());
    if(cameraCount == 0) {
        qDebug() << "Cannot find Any camera!";
        emit sigCameraUpdate(m_cameralist); // 仍需发送空列表
        return;
    }
    for (int i=0 ; i<cameraArray.GetSize() ; i++) {
        cameraArray[i].Attach(TLFactory.CreateDevice(devices[i]));
        std::string sn = cameraArray[i].GetDeviceInfo().GetSerialNumber().c_str();
        m_cameralist << QString::fromStdString(sn);
    }
    emit sigCameraUpdate(m_cameralist);
}

void BaslerCameraControl::CopyMono8ToQImage(CGrabResultPtr pInBuffer, QImage &OutImage)
{
    uchar* buff = (uchar*)pInBuffer->GetBuffer();
    int nHeight = pInBuffer->GetHeight();
    int nWidth = pInBuffer->GetWidth();
    if(m_size != QSize(nWidth, nHeight)) {
        m_size = QSize(nWidth, nHeight);
        emit sigSizeChange(m_size);
    }
    QImage imgBuff(buff, nWidth, nHeight, QImage::Format_Indexed8);
    OutImage = imgBuff;
    if(pInBuffer->GetPixelType() == PixelType_Mono8) {
        uchar* pCursor = OutImage.bits();
        if ( OutImage.bytesPerLine() != nWidth ) {
            for ( int y=0; y<nHeight; ++y ) {
                pCursor = OutImage.scanLine( y );
                for ( int x=0; x<nWidth; ++x ) {
                    *pCursor =* buff;
                    ++pCursor;
                    ++buff;
                }
            }
        } else {
            memcpy( OutImage.bits(), buff, nWidth * nHeight );
        }
    }
}

void BaslerCameraControl::CopyBayerRG8ToQImage(CGrabResultPtr pInBuffer, QImage &OutImage)
{
    uchar* buff = (uchar*)pInBuffer->GetBuffer();
    int nHeight = pInBuffer->GetHeight();
    int nWidth = pInBuffer->GetWidth();

    if(m_size != QSize(nWidth, nHeight)) {
        m_size = QSize(nWidth, nHeight);
        emit sigSizeChange(m_size);
    }

    // 创建 OpenCV Mat 来处理 Bayer 数据
    cv::Mat bayerMat(nHeight, nWidth, CV_8UC1, buff);
    cv::Mat rgbMat;

    // 将 Bayer RG8 转换为 RGB
    cv::cvtColor(bayerMat, rgbMat, cv::COLOR_BayerRG2BGR);

    // 转换为 QImage
    OutImage = QImage(rgbMat.data,
                      rgbMat.cols,
                      rgbMat.rows,
                      rgbMat.step,
                      QImage::Format_RGB888).copy();
}

int BaslerCameraControl::openCamera(QString cameraName)
{
    try {
        CDeviceInfo cInfo;
        String_t str = String_t(cameraName.toStdString().c_str());
        cInfo.SetSerialNumber(str);
        m_basler.Attach(CTlFactory::GetInstance().CreateDevice(cInfo));
        m_basler.Open();
        //获取触发模式
        getFeatureTriggerSourceType();
        m_isOpen = true;
    } catch (GenICam::GenericException &e) {
        OutputDebugString(L"OpenCamera Error\n");
        qCritical() << "GenICam 异常: " << QString::fromLocal8Bit(e.GetDescription());
        m_isOpen = false;
        return -2;
    }
    return 0;
}

bool BaslerCameraControl::isOpen() {
    return m_isOpen;
}


int BaslerCameraControl::closeCamera() {
    // 优先检查设备实际状态，而非依赖 m_isOpen
    if (!m_basler.IsOpen()) {
        m_isOpen = false; // 同步状态
        return 0;
    }

    try {
        if (m_basler.IsOpen()) {
            m_basler.StopGrabbing();    // 停止图像采集
            m_basler.DestroyDevice();   // 显式销毁设备句柄
            m_basler.Close();           // 关闭相机连接
            m_isOpen = false;
        }
        return 0;
    }
    catch (const GenICam::GenericException& e) {
        qCritical() << "关闭相机时发生 GenICam 异常: "
                    << QString::fromLocal8Bit(e.GetDescription());
        return -2;
    }
    catch (const std::exception& e) {
        qCritical() << "关闭相机时发生标准异常: " << e.what();
        return -2;
    }
    catch (...) {
        qCritical() << "关闭相机时发生未知异常";
        return -2;
    }
}

void BaslerCameraControl::setExposureTime(double time)
{
    //相机是否开启
    if(!m_isOpen) return;
    SetCamera(Type_Basler_ExposureTimeAbs, time);
}

int BaslerCameraControl::getExposureTime()
{
    // if(!m_isOpen) return -1;
    return QString::number(GetCamera(Type_Basler_ExposureTimeAbs)).toInt();
}

void BaslerCameraControl::setGain(double Gain)
{
    // if(!m_isOpen) return;
    SetCamera(Type_Basler_GainRaw, Gain);
}

int BaslerCameraControl::getGain()
{
    // if(!m_isOpen) return -1;
    return QString::number(GetCamera(Type_Basler_GainRaw)).toInt();
}


void BaslerCameraControl::autoExposureOnce()
{
    // 相机是否开启
    // if(!m_isOpen) return;
    if ( !m_basler.ExposureAuto.IsWritable())
    {
        std::cout << "The camera does not support Exposure Auto." << std::endl << std::endl;
        return;
    }
    m_basler.ExposureAuto.SetValue(Basler_UniversalCameraParams::ExposureAuto_Once);
    int n = 0;
    while (m_basler.ExposureAuto.GetValue() != Basler_UniversalCameraParams::ExposureAuto_Off)
    {
        ++n;
        WaitObject::Sleep(100);
        if (n > 100)
        {
            throw TIMEOUT_EXCEPTION( "The adjustment of auto exposure did not finish.");
        }
    }
}

void BaslerCameraControl::setFeatureTriggerSourceType(QString type)
{
    // if(!m_isOpen) return;
    //停止采集
    if(m_isOpenAcquire) {
        StopAcquire();
    }
    if(type == "Freerun") {
        SetCamera(Type_Basler_Freerun);
    } else if(type == "Line1"){
        SetCamera(Type_Basler_Line1);
    }
}

QString BaslerCameraControl::getFeatureTriggerSourceType()
{
    // if(!m_isOpen) return "";
    INodeMap &cameraNodeMap = m_basler.GetNodeMap();
    CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
    ptrTriggerSel->FromString("FrameStart");
    CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
    ptrTrigger->SetIntValue(0);
    CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode ("TriggerSource");

    String_t str = ptrTriggerSource->ToString();
    m_currentMode = QString::fromLocal8Bit(str.c_str());
    return m_currentMode;
}

void BaslerCameraControl::setFeatureTriggerModeType(bool on)
{
    // if(!m_isOpen) return;
    INodeMap &cameraNodeMap = m_basler.GetNodeMap();
    CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
    ptrTriggerSel->FromString("FrameStart");
    CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
    ptrTrigger->SetIntValue(on?1:0);
}

bool BaslerCameraControl::getFeatureTriggerModeType()
{
    // if(!m_isOpen) return false;
    INodeMap &cameraNodeMap = m_basler.GetNodeMap();
    CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
    ptrTriggerSel->FromString("FrameStart");
    CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
    return ptrTrigger->GetIntValue() == 1;
}

void BaslerCameraControl::SetCamera(BaslerCameraControl::BaslerCameraControl_Type index, double tmpValue)
{
    //相机是否开启
    // if(!m_isOpen) return;
    INodeMap &cameraNodeMap = m_basler.GetNodeMap();
    switch (index) {
    case Type_Basler_Freerun: {
        CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
        ptrTriggerSel->FromString("FrameStart");
        CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
#ifdef Real_Freerun
        ptrTrigger->SetIntValue(0);
#else //Software
        ptrTrigger->SetIntValue(1);
        CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode ("TriggerSource");
        ptrTriggerSource->FromString("Software");
#endif
    } break;
    case Type_Basler_Line1: {
        CEnumerationPtr  ptrTriggerSel = cameraNodeMap.GetNode ("TriggerSelector");
        ptrTriggerSel->FromString("FrameStart");
        CEnumerationPtr  ptrTrigger  = cameraNodeMap.GetNode ("TriggerMode");
        ptrTrigger->SetIntValue(1);
        CEnumerationPtr  ptrTriggerSource = cameraNodeMap.GetNode ("TriggerSource");
        ptrTriggerSource->FromString("Line1");
    } break;
    case Type_Basler_ExposureTimeAbs: {
        const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
        exposureTime->SetValue(tmpValue);
    } break;
    case Type_Basler_GainRaw: {
        const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
        cameraGen->SetValue(tmpValue);
    } break;
    case Type_Basler_AcquisitionFrameRateAbs: {
        const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
        frameRate->SetValue(TRUE);
        const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
        frameRateABS->SetValue(tmpValue);
    } break;
    case Type_Basler_Width: {
        const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
        widthPic->SetValue(tmpValue);
    } break;
    case Type_Basler_Height: {
        const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
        heightPic->SetValue(tmpValue);
    } break;
    case Type_Basler_LineSource: {
        CEnumerationPtr  ptrLineSource = cameraNodeMap.GetNode ("LineSource");
        ptrLineSource->SetIntValue(2);
    } break;
    default:
        break;
    }
}

double BaslerCameraControl::GetCamera(BaslerCameraControl::BaslerCameraControl_Type index)
{
    INodeMap &cameraNodeMap = m_basler.GetNodeMap();
    switch (index) {
    case Type_Basler_ExposureTimeAbs: {
        const CFloatPtr exposureTime = cameraNodeMap.GetNode("ExposureTimeAbs");
        return exposureTime->GetValue();
    } break;
    case Type_Basler_GainRaw: {
        const CIntegerPtr cameraGen = cameraNodeMap.GetNode("GainRaw");
        return cameraGen->GetValue();
    } break;
    case Type_Basler_AcquisitionFrameRateAbs: {
        const CBooleanPtr frameRate = cameraNodeMap.GetNode("AcquisitionFrameRateEnable");
        frameRate->SetValue(TRUE);
        const CFloatPtr frameRateABS = cameraNodeMap.GetNode("AcquisitionFrameRateAbs");
        return frameRateABS->GetValue();
    } break;
    case Type_Basler_Width: {
        const CIntegerPtr widthPic = cameraNodeMap.GetNode("Width");
        return widthPic->GetValue();
    } break;
    case Type_Basler_Height: {
        const CIntegerPtr heightPic = cameraNodeMap.GetNode("Height");
        return heightPic->GetValue();
    } break;
    default:
        return -1;
        break;
    }
}

long BaslerCameraControl::StartAcquire()
{
    if(!m_isOpen) return -1;
    m_isOpenAcquire = true;
    qDebug() << "BaslerCameraControl IsGrabbing";
    try {
        qDebug() << "BaslerCameraControl StartAcquire" << m_currentMode;
        if(m_currentMode == "Freerun")  {
            m_basler.StartGrabbing(GrabStrategy_LatestImageOnly,GrabLoop_ProvidedByInstantCamera);
            this->m_coarseTimer->start();
        } else if(m_currentMode == "Software") {
            m_basler.StartGrabbing(GrabStrategy_LatestImageOnly);
            this->m_coarseTimer->start();
        } else if(m_currentMode == "Line1") {
            m_basler.StartGrabbing(GrabStrategy_OneByOne);
            this->m_coarseTimer->start();
        } else if(m_currentMode == "Line2") {
            m_basler.StartGrabbing(GrabStrategy_OneByOne);
            this->m_coarseTimer->start();
        }
    } catch (GenICam::GenericException &e) {
        qDebug() << e.what();
        return -2;
    }
    return 0;
}

long BaslerCameraControl::StopAcquire()
{
    if(!m_isOpen) return -1;
    m_isOpenAcquire = false;
    qDebug() << "BaslerCameraControl StopAcquire";
    try {
        this->m_coarseTimer->stop();
        if (m_basler.IsGrabbing()) {
            m_basler.StopGrabbing();
        }
    } catch (GenICam::GenericException &e) {
        OutputDebugString(LPCWSTR(e.GetDescription()));
        return -2;
    }
    return 0;
}

long BaslerCameraControl::GrabImage(QImage &image, int timeout)
{
    try  {
        if (!m_basler.IsGrabbing()) {
            StartAcquire();
        }
        CGrabResultPtr ptrGrabResult;
        if(m_currentMode == "Freerun")  {
        } else if(m_currentMode == "Software") {
            if (m_basler.WaitForFrameTriggerReady(1000, TimeoutHandling_Return)) {
                m_basler.ExecuteSoftwareTrigger();
                m_basler.RetrieveResult(timeout, ptrGrabResult,TimeoutHandling_Return);
            }
        } else if(m_currentMode == "Line1") {
            m_basler.RetrieveResult(timeout, ptrGrabResult, TimeoutHandling_Return);
        } else if(m_currentMode == "Line2") {
            m_basler.RetrieveResult(timeout, ptrGrabResult, TimeoutHandling_Return);
        }
        if (ptrGrabResult->GrabSucceeded()) {
            if (!ptrGrabResult.IsValid()) { OutputDebugString(L"GrabResult not Valid Error\n"); return -1; }
            EPixelType pixelType = ptrGrabResult->GetPixelType();
            switch (pixelType) {
            case PixelType_Mono8: {
                CopyMono8ToQImage(ptrGrabResult, image);
            } break;
            case PixelType_BayerRG8: {
                CopyBayerRG8ToQImage(ptrGrabResult, image);
            } break;
            default:
                qDebug() << "Unsupported pixel format:" << pixelType;
                return -4;
                break;
            }
        } else {
            // OutputDebugString(L"Grab Error!!!");
            return -3;
        }
    } catch (GenICam::GenericException &e) {
        OutputDebugString(L"GrabImage Error\n");
        qCritical() << "GenICam 异常: " << QString::fromLocal8Bit(e.GetDescription());
        return -2;
    }  catch(...)  {
        OutputDebugString(L"ZP 11 Shot GetParam Try 12 No know Error\n");
        return -1;
    }
    return 0;
}

//Qimage 与 cv mat转换
void BaslerCameraControl::qImageToCvMat(const QImage& qImage, cv::Mat & image)
{
    switch (qImage.format()) {
    case QImage::Format_RGB32:
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
    case QImage::Format_RGB888: {
        // 转换RGB888到BGR格式
        // qDebug() << "Format_RGB888";
        cv::Mat mat(qImage.height(), qImage.width(), CV_8UC3,
                    const_cast<uchar*>(qImage.bits()),
                    static_cast<size_t>(qImage.bytesPerLine()));
        cv::cvtColor(mat, image, cv::COLOR_RGB2BGR); // 修正颜色顺序
        break;
    }
    case QImage::Format_Grayscale8: {
         // qDebug() << "Format_Grayscale8";
        // 直接复制灰度图像数据
        image = cv::Mat(qImage.height(), qImage.width(), CV_8UC1,
                    const_cast<uchar*>(qImage.bits()),
                    static_cast<size_t>(qImage.bytesPerLine()));
         break;
    }
    case QImage::Format_Indexed8: {
        // qDebug() << "Format_Indexed8";
        cv::Mat mat(qImage.height(), qImage.width(), CV_8UC1,
                    const_cast<uchar*>(qImage.bits()),
                    qImage.bytesPerLine());

        if (qImage.colorTable().isEmpty()) {
            image =  mat.clone();
        }
        else{
        // 预计算调色板（BGR顺序）
        std::vector<cv::Vec3b> palette;
        for (QRgb rgb : qImage.colorTable()) {
            palette.emplace_back(cv::Vec3b(qBlue(rgb), qGreen(rgb), qRed(rgb)));
        }

        // 应用调色板
        image =cv::Mat(mat.size(), CV_8UC3);
        for (int y = 0; y < mat.rows; ++y) {
            const uchar* src = mat.ptr<uchar>(y);
            cv::Vec3b* dst = image.ptr<cv::Vec3b>(y);
            for (int x = 0; x < mat.cols; ++x) {
                dst[x] = palette[src[x]];
            }
        }
        }
        break;
    }
    default:
        qWarning() << "Unsupported QImage format:" << qImage.format();
    }
}

// cv::Mat转QImage实现
QImage BaslerCameraControl::cvMatToQImage(const cv::Mat& mat)
{
    try {
        if(mat.empty()) return QImage();

        switch(mat.type()) {
        case CV_8UC3: { // BGR格式
            cv::Mat rgbMat;
            cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);
            return QImage(rgbMat.data,
                          rgbMat.cols,
                          rgbMat.rows,
                          static_cast<int>(rgbMat.step),
                          QImage::Format_RGB888).copy();
        }
        case CV_8UC4: { // BGRA格式
            return QImage(mat.data,
                          mat.cols,
                          mat.rows,
                          static_cast<int>(mat.step),
                          QImage::Format_ARGB32).copy();
        }
        case CV_8UC1: // 灰度图
            return QImage(mat.data,
                          mat.cols,
                          mat.rows,
                          static_cast<int>(mat.step),
                          QImage::Format_Grayscale8).copy();
        case CV_64FC1: {
            cv::Mat normalized;
            // 归一化到 [0, 255] 并转换为 8UC1
            mat.convertTo(normalized, CV_8UC1, 255.0);
            return  QImage(normalized.data, normalized.cols, normalized.rows,
                         normalized.step, QImage::Format_Grayscale8).copy();
        }
        default:
            qWarning() << "Unsupported cv::Mat type:" << mat.type();
            return QImage();
        }
    } catch (...) {
        return QImage();
    }
}

// 获取帧
cv::Mat BaslerCameraControl::getLatestFrame() {
    return this->img_vs[m_readIndex.load()].clone();
}

// 更新帧（在采集线程中）
void BaslerCameraControl::updateFrame(const cv::Mat& newFrame) {
    int writeIndex = 1 - m_readIndex.load();
    this->img_vs[writeIndex] = newFrame.clone();
    m_readIndex.store(writeIndex);
}

 cv::Mat& BaslerCameraControl::getImage()
{
     return this->img;
}

int BaslerCameraControl::getFrameRate()
{
    return this->fps;
}


std::shared_ptr<const cv::Mat> BaslerCameraControl::getLatestFrameShared() const
{
    QMutexLocker locker(&m_frameMutex);

    return std::make_shared<cv::Mat>(this->img_vs[m_readIndex.load()]);
}
