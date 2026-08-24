#ifndef MYSERVER_H
#define MYSERVER_H

#include <QTcpServer>
#include <QTcpSocket>
#include <QByteArray>

class TcpServer : public QObject
{
    Q_OBJECT

public:
      TcpServer(QObject *parent = nullptr);
      ~TcpServer();

private slots:

    void new_Connection();
    void disconect();
    void read();

public slots:

    void GPS_to_Server(QByteArray data) {Write_to_Client(data);}
    void GSM_to_Server(QByteArray data) {Write_to_Client(data);}                                    // запись в порт
    void I2C_to_Server(QByteArray data) {Write_to_Client(data);}

private:

    void Write_to_Client (QByteArray data);


    QTcpServer * server;
    QTcpSocket * socket;
    QByteArray arr;
    bool lock =  false;

signals:

   void Server_to_GPS (QByteArray point);
   void Server_to_I2C (QByteArray data);
};
#endif // MYSERVER_H
