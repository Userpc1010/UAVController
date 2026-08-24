#ifndef GSM_H
#define GSM_H

#include <QDebug>

class GSM : public QObject
{
    Q_OBJECT

public:

    GSM(QObject *parent = nullptr);
      ~GSM();

private:

    void Start_GSM ();

    void GSM_();

    void Initializing_GSM (const uint8_t step );

    void reset (bool res);

    void send_cordinat_SMS (const uint8_t step );


signals:

    void GSM_to_Server (QByteArray point);

    void GSM_to_GPS ();


private slots:

    void gsm_timer();


public slots:

    void GPS_to_GSM (double lat_, double lon_);

private:

uint8_t fd4; // UART 3/4

bool ini = false;  /* Готовность gsm модема к отправке */ bool sim_error = false; int8_t last = 0; /* Сброс модема */ uint8_t type_client = 0;

double lat, lon;

uint16_t count;

QByteArray data;

};

#endif // GSM_H
