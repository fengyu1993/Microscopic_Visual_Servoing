// MediaSaver.cpp
#include "MediaSaver.h"
#include <QDir>
#include <QPainter> // For potential QImage conversions
#include <QThread>  // Required for QThread::currentThreadId()

// MediaSaver constructor now simply initializes, no thread management here
MediaSaver::MediaSaver(QObject *parent) : QObject(parent) {
    // qDebug() << "MediaSaver object created in thread:" << QThread::currentThreadId();
}

// MediaSaver destructor (will be called in its QThread)
MediaSaver::~MediaSaver() {
    // qDebug() << "MediaSaver object being destroyed in thread:" << QThread::currentThreadId();
    // No need to quit/wait on m_thread here, as m_thread is managed externally
    // and this object is being deleted because its thread finished.
    qDebug() << "MediaSaver object destroyed.";
}

void MediaSaver::saveImage(QImage image, QString filePath) {
    // This slot will be executed in the thread MediaSaver lives in
    qDebug() << "Saving image in thread:" << QThread::currentThreadId();
    if (image.isNull()) {
        emit errorOccurred("Error: Attempting to save a null image.");
        emit imageSaveFinished(filePath, false);
        return;
    }
    bool success = image.save(filePath);
    if (!success) {
        emit errorOccurred("Error: Could not save image to " + filePath);
    }
    emit imageSaveFinished(filePath, success);
}

void MediaSaver::addFrame(QImage frame) {
    // This slot will be executed in the thread MediaSaver lives in
    // qDebug() << "Adding frame to buffer in thread:" << QThread::currentThreadId();
    if (!frame.isNull()) {
        m_frameBuffer.append(frame);
    } else {
        qWarning() << "Warning: Attempted to add a null frame to buffer.";
    }
}

void MediaSaver::clearFrames() {
    // This slot will be executed in the thread MediaSaver lives in
    qDebug() << "Clearing frame buffer in thread:" << QThread::currentThreadId();
    m_frameBuffer.clear();
}

// Overloaded saveVideo using the internal buffer
void MediaSaver::saveVideo(QString filePath, int fps, QString codec, bool isColor) {
    // This slot will be executed in the thread MediaSaver lives in
    qDebug() << "Saving video from internal buffer in thread:" << QThread::currentThreadId();
    bool success = internalSaveVideo(m_frameBuffer, filePath, fps, codec, isColor);
    emit videoSaveFinished(filePath, success);
    qDebug() << "Save video finish";
    // Optionally clear buffer after saving, or leave it to explicit clearFrames() call
    // m_frameBuffer.clear(); // Uncomment if you want to auto-clear after save
}

// Overloaded saveVideo taking a frames vector
void MediaSaver::save2Video(QVector<QImage> frames, QString filePath, int fps, QString codec, bool isColor) {
    // This slot will be executed in the thread MediaSaver lives in
    qDebug() << "Saving video from provided frames in thread:" << QThread::currentThreadId();
    bool success = internalSaveVideo(frames, filePath, fps, codec, isColor);
    emit videoSaveFinished(filePath, success);
}


// Private helper function for video saving logic
bool MediaSaver::internalSaveVideo(const QVector<QImage>& framesToSave, QString filePath, int fps, QString codec, bool isColor) {
    if (framesToSave.isEmpty()) {
        emit errorOccurred("Error: No frames provided for video generation.");
        return false;
    }

    if (filePath.isEmpty()) {
        emit errorOccurred("Error: Video save path is empty.");
        return false;
    }

    if (fps <= 0) {
        emit errorOccurred("Error: Frame rate (FPS) must be greater than 0.");
        return false;
    }

    const QImage& firstFrame = framesToSave.first();
    int width = firstFrame.width();
    int height = firstFrame.height();

    int fourcc = cv::VideoWriter::fourcc(codec.at(0).toLatin1(), codec.at(1).toLatin1(),
                                         codec.at(2).toLatin1(), codec.at(3).toLatin1());
    if (fourcc == 0) {
        emit errorOccurred("Error: Invalid video codec provided: " + codec + ". Please check supported codecs for your OpenCV build. Attempting to use MJPG as fallback.");
        fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G'); // Fallback
    }

    cv::VideoWriter videoWriter(filePath.toStdString(), fourcc, fps, cv::Size(width, height), isColor);

    if (!videoWriter.isOpened()) {
        emit errorOccurred("Error: Could not open video writer for file: " + filePath);
        return false;
    }

    for (int i = 0; i < framesToSave.size(); ++i) {
        const QImage& frame = framesToSave.at(i);
        if (frame.isNull()) {
            emit errorOccurred("Warning: Skipping null frame at index " + QString::number(i) + ".");
            continue;
        }

        cv::Mat cvFrame = qImageToCvMat(frame);
        if (cvFrame.empty()) {
            emit errorOccurred("Warning: Failed to convert QImage to cv::Mat for frame at index " + QString::number(i) + ". Skipping.");
            continue;
        }

        videoWriter.write(cvFrame);

        // Report progress
        int progress = (static_cast<double>(i + 1) / framesToSave.size()) * 100;
        emit videoSaveProgress(progress);
    }

    videoWriter.release();
    qDebug() << "Internal video saving logic finished for" << filePath;
    return true;
}

cv::Mat MediaSaver::qImageToCvMat(const QImage& qimage) {
    if (qimage.isNull()) {
        return cv::Mat();
    }

    // Attempt to convert indexed or unsupported formats to RGB888 first.
    // RGB888 is a common and direct mapping to OpenCV's BGR (on little-endian systems).
    QImage processedImage = qimage;
    if (qimage.format() != QImage::Format_ARGB32 &&
        qimage.format() != QImage::Format_ARGB32_Premultiplied &&
        qimage.format() != QImage::Format_RGB32 &&
        qimage.format() != QImage::Format_RGB888 &&
        qimage.format() != QImage::Format_Grayscale8)
    {
        // If it's an indexed format or other unsupported format, convert to RGB888
        // This handles QImage::Format_Indexed8 directly, removing the warning.
        processedImage = qimage.convertToFormat(QImage::Format_RGB888);
        if (processedImage.isNull()) {
            qWarning() << "Error: Failed to convert QImage format" << qimage.format() << "to Format_RGB888.";
            return cv::Mat();
        }
    }

    switch (processedImage.format()) {
    case QImage::Format_ARGB32:
    case QImage::Format_ARGB32_Premultiplied:
        // Convert to 8-bit, 4-channel BGRA for OpenCV
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC4, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case QImage::Format_RGB32: // In QImage, this is BGRX (on little-endian) or RGBX
        // For OpenCV, CV_8UC4 with QImage::Format_RGB32 often means BGRA or RGBA.
        // It's generally safe to map RGB32 directly to CV_8UC4.
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC4, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case QImage::Format_RGB888: {// In QImage, this is BGR (on little-endian systems)
        cv::Mat mat(processedImage.height(), processedImage.width(), CV_8UC3,
                    const_cast<uchar*>(processedImage.bits()),
                    static_cast<size_t>(processedImage.bytesPerLine()));
        cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
        return mat.clone();
    }
    case QImage::Format_Grayscale8:
        return cv::Mat(processedImage.height(), processedImage.width(), CV_8UC1, (void*)processedImage.constBits(), processedImage.bytesPerLine()).clone();
    case  QImage::Format_Indexed8: {
        cv::Mat mat(processedImage.height(), processedImage.width(), CV_8UC1,
                    const_cast<uchar*>(processedImage.bits()),
                    processedImage.bytesPerLine());
        cv::Mat image;
        if (processedImage.colorTable().isEmpty()) {
            image =  mat.clone();
        }
        else{
            // 预计算调色板（BGR顺序）
            std::vector<cv::Vec3b> palette;
            for (QRgb rgb : processedImage.colorTable()) {
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
        return image.clone();
    }
    default:
        // This case should ideally not be reached after the initial conversion attempt
        qWarning() << "Error: Unhandled QImage format" << processedImage.format() << "after conversion attempt.";
        return cv::Mat();
    }
}
