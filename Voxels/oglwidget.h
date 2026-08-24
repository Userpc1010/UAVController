#ifndef OGLWIDGET_H
#define OGLWIDGET_H

#include <QOpenGLWidget>
//#include <QtOpenGL>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QOpenGLFunctions_ES2>
#include <Data.h>

class SimpleObject3D;
class Camera_3D;
class Cube;

class OGLWidget : public QOpenGLWidget
{
    Q_OBJECT

public:
    OGLWidget(QWidget *QOpenGLWidget = nullptr);
    ~OGLWidget();

protected:
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();

    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);

    void initShaders();
    bool initObj(const QString &path, const QImage &img);

public slots:

    void I2C_to_SLAM (QByteArray data);

private:
    QMatrix4x4 m_PojectionMatrix;
    QOpenGLShaderProgram m_Program;
    QVector2D m_MousePosition;
    QQuaternion m_Rotation;

    GPS_to__SLAM SLAM;

    SimpleObject3D *Drone;
    Camera_3D *m_camera;
    Cube  *m_cube;

    float s_w, s_h;

    bool camera;

    double lastX = 0.0, lastY = 0.0;

    //float camera_offset_z;
};

#endif // OGLWIDGET_H
