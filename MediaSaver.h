// media_saver.h
#ifndef MEDIA_SAVER_H
#define MEDIA_SAVER_H

#include <QObject>
#include <QImage>
#include <QString>
#include <QVector>
#include <QDebug> // For debugging output

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp> // For cv::VideoWriter

/**
 * @brief MediaSaver class for saving QImage pictures and generating video from a QImage sequence.
 *
 * This class acts as the worker object that will be moved to a separate QThread.
 * Its public slots are designed to be executed in the thread it lives in.
 * Video saving uses the OpenCV library.
 */
class MediaSaver : public QObject {
    Q_OBJECT // Required for signals and slots

public:
    /**
     * @brief Constructor. This object will be moved to a QThread externally.
     * @param parent Optional QObject parent.
     */
    explicit MediaSaver(QObject *parent = nullptr);

    /**
     * @brief Destructor. This will be called in the thread that MediaSaver lives in.
     */
    ~MediaSaver();

public slots: // These methods will be executed in the thread MediaSaver lives in
    /**
     * @brief Slot to perform image saving.
     * @param image The QImage object to save. (QImage is implicitly shared, efficient to pass by value)
     * @param filePath The complete file path (including filename and extension).
     */
    void saveImage(QImage image, QString filePath);

    /**
     * @brief Adds a QImage frame to the internal buffer for video generation.
     * @param frame The QImage frame to add.
     */
    void addFrame(QImage frame);

    /**
     * @brief Clears the internal frame buffer.
     * It's recommended to call this after a video is saved or if frames are no longer needed.
     */
    void clearFrames();

    /**
     * @brief Slot to perform video saving, using the internal frame buffer.
     * @param filePath The complete file path for the video output.
     * @param fps The frames per second (FPS) for the video.
     * @param codec The four character code for the video codec (e.g., "MP4V", "MJPG").
     * @param isColor True if the frames are color frames, false for grayscale.
     */
    void saveVideo(QString filePath, int fps, QString codec = "MJPG", bool isColor = true);

    /**
     * @brief Slot to perform video saving, using an explicitly provided frame vector.
     * @param frames A QVector containing all video frames as QImage objects.
     * @param filePath The complete file path for the video output.
     * @param fps The frames per second (FPS) for the video.
     * @param codec The four character code for the video codec (e.g., "MP4V", "MJPG").
     * @param isColor True if the frames are color frames, false for grayscale.
     */
    void save2Video(QVector<QImage> frames, QString filePath, int fps, QString codec = "MJPG", bool isColor = true);

signals:
    /**
     * @brief Signal emitted when an image saving operation is finished.
     * @param filePath The path of the saved image.
     * @param success True if the image was successfully saved, false otherwise.
     */
    void imageSaveFinished(QString filePath, bool success);

    /**
     * @brief Signal emitted when a video saving operation is finished.
     * @param filePath The path of the saved video.
     * @param success True if the video was successfully saved, false otherwise.
     */
    void videoSaveFinished(QString filePath, bool success);

    /**
     * @brief Signal emitted to report video saving progress (0-100%).
     * @param progress The current progress percentage.
     */
    void videoSaveProgress(int progress);

    /**
     * @brief Signal emitted when an error occurs during saving.
     * @param message A descriptive error message.
     */
    void errorOccurred(QString message);

private:
    QVector<QImage> m_frameBuffer; /**< Internal buffer to store video frames. */

    /**
     * @brief Helper function to perform the actual video saving logic.
     * This function is used by both overloaded saveVideo slots.
     * @param framesToSave The QVector of QImage objects to save.
     * @param filePath The complete file path for the video output.
     * @param fps The frames per second (FPS) for the video.
     * @param codec The four character code for the video codec.
     * @param isColor True if the frames are color frames, false for grayscale.
     * @return true if the video was successfully generated, false otherwise.
     */
    bool internalSaveVideo(const QVector<QImage>& framesToSave, QString filePath, int fps, QString codec, bool isColor);

    /**
     * @brief Converts a QImage to an OpenCV cv::Mat.
     * This function handles various QImage formats and converts them to a suitable cv::Mat format.
     * @param qimage The QImage to convert.
     * @return The converted cv::Mat. Returns an empty cv::Mat if conversion fails or qimage is null.
     */
    cv::Mat qImageToCvMat(const QImage& qimage);
};

#endif // MEDIA_SAVER_H
