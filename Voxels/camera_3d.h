#ifndef CAMERA_3D_H
#define CAMERA_3D_H

#include "simpleobject3d.h"
#include <QQuaternion>
#include <QVector3D>
#include <QMatrix4x4>

class Camera_3D

{

public:
    Camera_3D();

    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions*functions = nullptr);

    void rotate_camera (double x, double y);
    void camera_zoom   (bool zoom);

    void position_camera (QVector3D pos);

    void Front_move ();
    void Back_move  ();
    void right_move ();
    void left_move  ();
    void Up_move    ();
    void Down_move  ();

private:

    void updateViewMatrix();

private:
    QVector3D m_Translate =  QVector3D( 0.0f, 0.0f, -15.0f );
    QVector3D m_camera_up =  QVector3D( 0.0f, 0.25f, 0.0f );
    QVector3D front;

    QMatrix4x4 m_GlobalTransform;
    QMatrix4x4 m_ViewMatrix;

    qreal pitch = 0.0;
    qreal yaw = -90.0050;

    double sensitivity_x = 0.00005;
    double sensitivity_y = 0.00005;
};

#endif // CAMERA_3D_H
