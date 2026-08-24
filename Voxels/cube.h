#ifndef CUBE_H
#define CUBE_H
#include "transformational.h"
#include "simpleobject3d.h"
#include <QQuaternion>
#include <QVector3D>
#include <QMatrix4x4>

class Cube
{
public:
    Cube();

    void draw(QOpenGLShaderProgram* program, QOpenGLFunctions* functions);
    void initCube(float width, const QImage &img);

    void add_cube ();
    void delete_cube ();
    void offset_cube();



private:

float i = 5.0f;
int ii = 0; // ×èñëî èíäåêñîâ íóæíî îãðàíè÷èòü ðàäèóñ èíäåêñîâ.

QVector<SimpleObject3D*> m_Objects_Cube;


};

#endif // CUBE_H
