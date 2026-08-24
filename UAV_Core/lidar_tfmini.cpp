#include "lidar_tfmini.h"
#include <QTimer>
#include <wiringSerial.h>
#include <errno.h>

Lidar_TFmini::Lidar_TFmini(QObject *parent) : QObject(parent)
{
    QTimer *timer_liadar = new QTimer(this);

    connect(timer_liadar, SIGNAL(timeout()), this, SLOT (lidar_timer()));

    start_lidar();

    timer_liadar->start(10);
}

Lidar_TFmini::~Lidar_TFmini()
{
  serialClose(fd1);
}

void Lidar_TFmini::start_lidar()
{
 fd1 = serialOpen ("/dev/ttyS1", 115200);

  qDebug()<<"Лидар запущен";
}

void Lidar_TFmini::lidar_timer()
{

   getTFminiData(&distance, &strength, &complete);
   if (complete) Lidar_to_I2C(distance);

}

void Lidar_TFmini::getTFminiData (int* distance, int* strength, bool* complete) {

  uint8_t i = 0;
  uint8_t checksum = 0;

  while(serialDataAvail(fd1)) {
    rx[i] = serialGetchar(fd1);
    if(rx[0] != 0x59) {
      i = 0;
    } else if(i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if(i == 8) {
      for(int j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if(rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
        *complete = true;
      }
      i = 0;
    } else {
      i++;
    }
  }
}
