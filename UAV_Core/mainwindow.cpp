#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "myserver.h"
#include "gps.h"
#include "gsm.h"
#include "i2c_ports.h"
#include "lidar_tfmini.h"
#include "oglwidget.h"

#include <QApplication>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  QSurfaceFormat format;
  format.setSamples(16);
  format.setDepthBufferSize(24);
  QSurfaceFormat::setDefaultFormat(format);

  OGLWidget * widget = new OGLWidget(this);

  this->setCentralWidget(widget);

  widget->setFocusPolicy(Qt::StrongFocus);

  GPS * gps = new GPS(this);
  GSM * gsm = new GSM(this);
  I2C_Ports * I2C = new I2C_Ports(this);
  TcpServer * Server = new TcpServer(this);
  Lidar_TFmini * Lidar = new Lidar_TFmini(this);



  connect (Server, SIGNAL(Server_to_GPS(QByteArray)),              gps, SLOT(Server_to_GPS(QByteArray)));

  connect (gps, SIGNAL(GPS_to_Server(QByteArray)),                 Server, SLOT(GPS_to_Server(QByteArray)));

  connect (gsm, SIGNAL(GSM_to_Server(QByteArray)),                 Server, SLOT(GSM_to_Server(QByteArray)));

  connect (gsm, SIGNAL(GSM_to_GPS()),                              gps, SLOT(GSM_to_GPS()));

  connect (gps, SIGNAL(GPS_to_GSM(double, double)),                gsm, SLOT(GPS_to_GSM(double, double)));

  connect (Server, SIGNAL(Server_to_I2C(QByteArray)),              I2C, SLOT(Server_to_I2C(QByteArray)));

  connect (I2C, SIGNAL(I2C_to_Server(QByteArray)),                 Server, SLOT(I2C_to_Server(QByteArray)));

  connect (gps, SIGNAL(GPS_to_I2C(QByteArray)),                    I2C, SLOT(GPS_to_I2C(QByteArray)));

  connect (gps, SIGNAL(GPS_to_SLAM(QByteArray)),                   widget, SLOT(I2C_to_SLAM(QByteArray)));

  connect (I2C, SIGNAL(I2C_to_GPS(QByteArray)),                    gps, SLOT(I2C_to_GPS(QByteArray)));

  connect (Lidar, SIGNAL(Lidar_to_I2C(int)),                       I2C, SLOT(Lidar_to_I2C(int)));

}

MainWindow::~MainWindow()
{
    delete ui;
}
