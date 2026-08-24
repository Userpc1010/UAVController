QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Voxels
TEMPLATE = lib

SOURCES += \
        oglwidget.cpp \
        simpleobject3d.cpp \
    transformational.cpp \
    camera_3d.cpp \
    cube.cpp

HEADERS += \
        oglwidget.h \
        simpleobject3d.h \
    transformational.h \
    camera_3d.h \
    cube.h

INCLUDEPATH += $$PWD/../UAV_Core
DEPENDPATH += $$PWD/../UAV_Core

# Default rules for deployment.
#qnx: target.path = /tmp/$${TARGET}/bin
#else: unix:!android: target.path = /opt/$${TARGET}/bin
#!isEmpty(target.path): INSTALLS += target

RESOURCES += \
    Recurce/shaders.qrc \
    Recurce/textures.qrc \
    Recurce/model.qrc

DISTFILES += \
    Recurce/Cube.png \
    Recurce/cube1.png \
    Recurce/cube2.png \
    Recurce/cube3.png \
    Recurce/fragshader.fsh \
    Recurce/vertshader.vsh
