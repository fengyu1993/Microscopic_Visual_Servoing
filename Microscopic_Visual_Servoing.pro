QT += core gui
QT += charts

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    VisualServoingController.cpp \
    main.cpp \
    mainwindow.cpp\
    BaslerCameraControl.cpp\
    ParallelPlatform.cpp\
    modern_robotic_lib.cpp\
    microscopic_visual_servoing.cpp\
    direct_microscopic_visual_servoing.cpp\
    defocus_microscopic_visual_servoing.cpp\
    MicroscopeCalibration.cpp\
    MediaSaver.cpp

HEADERS += \
    VisualServoingController.h \
    mainwindow.h\
    BaslerCameraControl.h\
    ParallelPlatform.h\
    modern_robotic_lib.h\
    microscopic_visual_servoing.h\
    direct_microscopic_visual_servoing.h\
    defocus_microscopic_visual_servoing.h\
    MicroscopeCalibration.h\
    MediaSaver.h


FORMS += \
    mainwindow.ui

# eigen
INCLUDEPATH += $$PWD/dependence/eigen-3.4.0


# pylon
INCLUDEPATH += $$PWD/dependence/pylon/include
INCLUDEPATH += $$PWD/dependence/pylon/lib/x64
DEPENDPATH += $$PWD/dependence/pylon/lib/x64
win32: LIBS += -L$$PWD/dependence/pylon/lib/x64/ -lGCBase_MD_VC141_v3_1_Basler_pylon -lGenApi_MD_VC141_v3_1_Basler_pylon -lgxapi_v15 -lPylonBase_v9 -lPylonC_v9 -lPylonDataProcessing_v3 -lPylonGUI_v9 -lPylonUtility_v9 -luxapi_v14 -luxtopapi_v9

# NarPod
INCLUDEPATH += $$PWD/dependence/NarpodSDK-1.0.12
win32: LIBS += -L$$PWD/dependence/NarpodSDK-1.0.12/64/ -lNarpodControl
INCLUDEPATH += $$PWD/dependence/NarpodSDK-1.0.12/64
DEPENDPATH += $$PWD/dependence/NarpodSDK-1.0.12/64

# opencv
win32:CONFIG(release, debug|release): LIBS += -L$$PWD/dependence/opencv/x64/vc15/lib/ -lopencv_world451
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/dependence/opencv/x64/vc15/lib/ -lopencv_world451d
INCLUDEPATH += $$PWD/dependence/opencv/include
DEPENDPATH += $$PWD/dependence/opencv/include

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
