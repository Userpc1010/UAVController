#include "camera_3d.h"
#include <QOpenGLShaderProgram>
#include <QtMath>

Camera_3D::Camera_3D()
{
}

void Camera_3D::draw(QOpenGLShaderProgram *program, QOpenGLFunctions *functions)
{
    if(functions != nullptr) return;

    program->setUniformValue("u_viewMatrix", m_ViewMatrix);

    updateViewMatrix();
}

void Camera_3D::rotate_camera(double x, double y)
{
  yaw += (x * sensitivity_x); pitch += (y * sensitivity_y);

  if(pitch > 89.99)  pitch = 89.99;
  if(pitch < -89.99) pitch = -89.99;

  front.setX(qCos(qRadiansToDegrees(yaw)) * qCos(qRadiansToDegrees(pitch)));
  front.setY(qSin(qRadiansToDegrees(pitch)));
  front.setZ(qSin(qRadiansToDegrees(yaw)) * qCos(qRadiansToDegrees(pitch)));
  front.normalize();
}

void Camera_3D::camera_zoom(bool zoom)
{
   if(zoom)m_Translate += front;
   else m_Translate -= front;
}

void Camera_3D::position_camera(QVector3D pos)
{
  m_Translate += pos;
}

void Camera_3D::Front_move()
{
  m_Translate += front;
}

void Camera_3D::Back_move()
{
  m_Translate -= front;
}

void Camera_3D::right_move()
{
  m_Translate -=  QVector3D::crossProduct(front, m_camera_up).normalized();
}

void Camera_3D::left_move()
{
 m_Translate += QVector3D::crossProduct(front, m_camera_up).normalized();

}

void Camera_3D::Up_move()
{
  m_Translate += m_camera_up;
}

void Camera_3D::Down_move()
{
  m_Translate -= m_camera_up;
}

void Camera_3D::updateViewMatrix()
{
    m_ViewMatrix.setToIdentity();
    m_ViewMatrix.lookAt( m_Translate,  m_Translate + front, m_camera_up );
    m_ViewMatrix = m_ViewMatrix * m_GlobalTransform.inverted();
}
