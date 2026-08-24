#include "myserver.h"
#include <QMessageBox>

TcpServer::TcpServer(QObject *parent): QObject(parent)
{
 server = new QTcpServer(this);

 if (server->listen(QHostAddress::Any,8981))qDebug()<<"Сервер запущен"; else{ QMessageBox::critical(0, "Ошибка сервера", "Старт не возможен:" + server->errorString()); server->close();}

 connect(server, SIGNAL(newConnection()), this, SLOT(new_Connection()));

}

TcpServer::~TcpServer()
{
   lock = false;
   socket->close();
   server->close();
   socket->deleteLater();
   server->deleteLater();
}

void TcpServer::new_Connection()
{

  socket = server->nextPendingConnection();

  connect(socket, SIGNAL(disconnected()), this,SLOT(disconect()));

  connect(socket, SIGNAL(readyRead()), this, SLOT(read()));

  socket->write("Connection");

  socket->waitForBytesWritten(5000);

  qDebug()<<"Соединение установлено";

  lock = true;
}

void TcpServer::disconect()
{
   lock = false;
   socket ->close();
   socket ->deleteLater();
}

void TcpServer::read()
{      
     arr = socket->readAll();

      if ( arr == "wu" )Server_to_I2C("wu");
      if ( arr == "su" )Server_to_I2C("su");
      if ( arr == "au" )Server_to_I2C("au");
      if ( arr == "du" )Server_to_I2C("du");
      if ( arr == "qu" )Server_to_I2C("qu");
      if ( arr == "eu" )Server_to_I2C("eu");
      if ( arr == "wd" )Server_to_I2C("wd");
      if ( arr == "sd" )Server_to_I2C("sd");
      if ( arr == "ad" )Server_to_I2C("ad");
      if ( arr == "dd" )Server_to_I2C("dd");
      if ( arr == "qd" )Server_to_I2C("qd");
      if ( arr == "ed" )Server_to_I2C("ed");

      if ( arr == "cs" )Server_to_I2C("cs");
      if ( arr == "cp" )Server_to_I2C("cp");
      if ( arr == "st" )Server_to_GPS("st");
      if ( arr == "rt" )Server_to_GPS("rt");
      if ( arr == "dr" )Server_to_GPS("dr");

      if ( arr == "MN0")Server_to_I2C("MN0");
      if ( arr == "MN1")Server_to_I2C("MN1");
      if ( arr == "PID0")Server_to_I2C("PID0");
      if ( arr == "LIM0")Server_to_I2C("LIM0");
      if ( arr == "TRIM0")Server_to_I2C("TRIM0");

      if ( arr == "dp")Server_to_I2C("dp");
      if ( arr == "cr")Server_to_I2C("cr");

      if (arr[0] == 'M' && arr[1] == 'O' && arr[2] == 'V' && arr[3] == 'E' ) Server_to_GPS(arr);

      if (arr[0] == 'R' && arr[1] == 'O' && arr[2] == 'U' && arr[3] == 'T' ) Server_to_GPS(arr);

      if (arr[0] == 'P' && arr[1] == 'I' && arr[2] == 'D' && arr[3] == '1' ) Server_to_I2C(arr);

      if (arr[0] == 'L' && arr[1] == 'I' && arr[2] == 'M' && arr[3] == '1' ) Server_to_I2C(arr);

      if (arr[0] == 'T' && arr[1] == 'R' && arr[2] == 'I' && arr[3] == 'M' && arr[4] == '1' ) Server_to_I2C(arr);


      if( arr[0] == 't' && arr[1] == 'r') Server_to_I2C(arr);

}


void TcpServer::Write_to_Client(QByteArray data)
{
  if(lock){socket->write(data); socket->waitForBytesWritten(5000); socket->flush();}
}

