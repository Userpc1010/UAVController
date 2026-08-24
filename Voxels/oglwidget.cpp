#include "oglwidget.h"
#include "simpleobject3d.h"
#include "camera_3d.h"
#include "cube.h"

#include <QMouseEvent>
#include <QOpenGLContext>
#include <QtMath>


OGLWidget::OGLWidget(QWidget *parent)
    : QOpenGLWidget(parent)
{
   m_camera = new Camera_3D;
   m_cube = new Cube;
}

OGLWidget::~OGLWidget()
{
   makeCurrent();
   delete Drone;
   delete m_camera;
   delete m_cube;
}

void OGLWidget::initializeGL()
{
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);

    initShaders();

    if(initObj(":/UAV/UAV.obj", QImage(":/UAV/UAV.png")))
    {
      Drone->translate(QVector3D(0.0f, 0.0f, 0.0f));
    }
}

void OGLWidget::resizeGL(int w, int h)
{
    float aspect = w / (h? static_cast<float>(h) : 1);
    m_PojectionMatrix.setToIdentity();
    m_PojectionMatrix.perspective(88, aspect, 0.01f, 1000.0f);
}

void OGLWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    m_Program.bind();
    m_Program.setUniformValue("u_projectionMatrix", m_PojectionMatrix);
    m_cube->draw(&m_Program,context()->functions());
    Drone->draw(&m_Program, context()->functions());
    m_camera->draw(&m_Program);
    m_Program.release();
}

void OGLWidget::I2C_to_SLAM(QByteArray data)
{
  uint8_t ee = 0; uint8_t* p = (uint8_t*)(void*)&SLAM; for( int count = data.length() - ee; count ; --count ) *p++ = data[ee++];

  Drone->rotate(SLAM.rotation);

  Drone->translate(SLAM.positional);

  update();
}

void OGLWidget::mousePressEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::LeftButton)
    {
     m_cube->add_cube();
    }

    if(event->buttons() == Qt::MidButton)
    {
     m_cube->delete_cube();
    }

    if(event->buttons() == Qt::RightButton)
    {

     lastX =  event->localPos().x();
     lastY =  event->localPos().y();

    }

    event->accept();
}

void OGLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    if(event->buttons() == Qt::RightButton)
    {
     camera = true;
    }
    event->accept();
}


void OGLWidget::mouseMoveEvent(QMouseEvent *event)
{

    if(event->buttons() == Qt::RightButton)
    {
        if (camera){

          lastX = event->localPos().x();
          lastY = event->localPos().y();
          camera = false;
        }
         double xoffset = event->localPos().x() - lastX;
         double yoffset = lastY - event->localPos().y();

         lastX = event->localPos().x();
         lastY = event->localPos().y();

         m_camera->rotate_camera(xoffset, yoffset);
    }

    event->accept();
}

void OGLWidget::wheelEvent(QWheelEvent *event)
{
  if ( event->delta() > 0 ) m_camera->camera_zoom(true);
  if ( event->delta() < 0 ) m_camera->camera_zoom(false);

  event->accept();
}

void OGLWidget::keyPressEvent(QKeyEvent *event)
{
     qDebug()<< event->key();
     if (event->key() == 1062) m_camera->Front_move();
     if (event->key() == 1067) m_camera->Back_move();
     if (event->key() == 1042) m_camera->right_move();
     if (event->key() == 1060) m_camera->left_move();
     if (event->key() == 32)m_camera->Up_move();
     if (event->key() == 1071) m_camera->Down_move();

     event->accept();
}

void OGLWidget::initShaders()
{
    if(! m_Program.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/vertshader.vsh"))
        close();
    if(! m_Program.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/fragshader.fsh"))
        close();

    if(! m_Program.link())
        close();
}

bool OGLWidget::initObj(const QString &path, const QImage &img)
{
    QFile objfile(path);

    if(! objfile.exists()) { qCritical() << "File not exist:" << path; return false; }
    if(! objfile.open(QFile::ReadOnly))  { qCritical() << "File not opened:" << path; return false; }

    QTextStream input(&objfile);
    QVector<QVector3D> coords;
    QVector<QVector2D> texturcoords;
    QVector<QVector3D> normals;

    QVector<VertexData> vertexes;
    QVector<GLuint> indexes;

    qDebug() << "Reading" << path << "...";

    bool ok = true;
    while(!input.atEnd() && ok)
    {
        auto str = input.readLine(); if(str.isEmpty()) continue;
        auto strlist = str.split(' '); strlist.removeAll("");
        auto key = strlist.at(0);

        if (key == "#") { qDebug() << str; }
        else if(key == "mtllib")
        {
            qDebug() << str;
            // ìàòåðèàë
        }
        else if(key.toLower() == "o")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "g")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "s")
        {
            qDebug() << str;
        }
        else if(key.toLower() == "v")
        {
            if(strlist.size() > 3)
            {
                coords.append(QVector3D(strlist.at(1).toFloat(&ok),
                                        strlist.at(2).toFloat(&ok),
                                        strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vt")
        {
            if(strlist.size() > 2)
            {
                texturcoords.append(QVector2D(strlist.at(1).toFloat(&ok),
                                              strlist.at(2).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "vn")
        {
            if(strlist.size() == 4)
            {
                normals.append(QVector3D(strlist.at(1).toFloat(&ok),
                                         strlist.at(2).toFloat(&ok),
                                         strlist.at(3).toFloat(&ok)));
                if(!ok) { qCritical() << "Error at line (format):" << str; }
            }
            else { qCritical() << "Error at line (count):" << str; ok = false; }
        }
        else if(key.toLower() == "f")
        {
            for(int i = 1; i < strlist.size(); i++)
            {
                auto v = strlist.at(i).split('/');
                if(v.size() == 3 && !v.at(1).isEmpty() && !v.at(2).isEmpty())
                {
                    vertexes.append(VertexData(coords.at(v.at(0).toInt(&ok, 10) - 1),
                                               texturcoords.at(v.at(1).toInt(&ok, 10) - 1),
                                               normals.at(v.at(2).toInt(&ok, 10) - 1)));
                    indexes.append(static_cast<GLuint>(indexes.size()));
                }
                else
                {
                    qCritical() << "Unsupported OBJ data format:" << strlist.at(i);
                    ok = false; break;
                }
            }
            if(!ok) { qCritical() << "Error at line (format):" << str; }
        }
    }

    objfile.close();
    qDebug() <<  "... done";
    if(!ok) return false;

    Drone = new SimpleObject3D(vertexes, indexes, img);
    return true;
}

