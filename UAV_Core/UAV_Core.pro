
QT += core gui network

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = UAV_Core
TEMPLATE = app

SOURCES += main.cpp \
    myserver.cpp \
    gps.cpp \
    gsm.cpp \
    EEPROM_I2C.cpp \
    i2c_ports.cpp \
    pca9685.cpp \
    lidar_tfmini.cpp \
    BNO080.cpp \
    ms5611.cpp \
    timer.cpp \
    mainwindow.cpp \
    GpsImuFusionKalmon.cpp \
    Matrix.cpp \
    Kalman.cpp

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

LIBS += -L/usr/local/lib -lwiringPi

HEADERS += \
    myserver.h \
    gps.h \
    Data.h \
    gsm.h \
    EEPROM_I2C.h \
    i2c_ports.h \
    utilitiy/tools.h \
    pca9685.h \
    lidar_tfmini.h \
    BNO080.h \
    ms5611.h \
    timer.h \
    mainwindow.h \
    Kalman.h \
    Matrix.h \
    Commons.h \
    GpsImuFusionKalmon.h

unix:!symbian: LIBS += -L$$OUT_PWD/../SLAM/ -lSLAM

INCLUDEPATH += $$PWD/../SLAM
DEPENDPATH += $$PWD/../SLAM

FORMS += \
    mainwindow.ui
