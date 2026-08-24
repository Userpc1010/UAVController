#ifndef UART_GPS_GSM_H
#define UART_GPS_GSM_H

#include "Data.h"
#include <utilitiy/tools.h>
#include "GpsImuFusionKalmon.h"
#include "timer.h"

class GPS : public QObject
{

    Q_OBJECT

public:

    GPS(QObject *parent = nullptr);
      ~GPS();

    void Start_GPS ();

private:

NAV_PVT pvt; /* UBX NAV_PVT */ uint8_t gps_reset_count = 0; /* Счётчик сброса GPS */ uint8_t gps_reset_count_2 = 0; bool first_start = true;

MOVE _move; ROUT _rout;  /* Полётные точки */ Move _move_; /* Курс для DUE */ SI si; /* Перевод данных UBX в СИ */

Current_pos current;

Telemetry_wlan Tele_w; //Телеметрия для MapGraphics

Current_attitude attitude; //Текущее положение БПЛА

GPS_to__SLAM SLAM;

GPSAccKalmanFilter_t * kf2;

Tools tools;

Timer timer;

uint8_t fd3; // UART 3/4

QVector <ROUT> array_rout; //Запись маршрута полёта

double last_axis_lon = 0.0;
double last_axis_lat = 0.0;

double lat_ [6];
double lon_ [6];

float Distance_to_point = 0.0f;

uint8_t circle_stage_1 = 0;

static const uint8_t size = 15;
float lin_x [size];
float lin_y [size];

float position_x [12];
float position_y [12];

uint64_t last_time_micros = 0;
uint64_t Time = 0;
float time_charg_rk4 = 0.0f;

int fpos = 0;
unsigned char checksum[2];
int payloadSize = sizeof(NAV_PVT);


    void translit_to_SI ();

    void GoToNavPoint ();

    int UpdateNavPiont (int type);

    float LOS_guidance_law (double curent_lat, double curent_lon, double next_lat, double next_lon, double previus_lat, double previus_lon, uint8_t counter);

    float circle (double axis_lat, double axis_lon, double curent_lat, double curent_lon, double radius_circle, double radius_point, float heading);

    void Move_cirle();

    void Move_line(uint8_t fly_point);

    void Positional ();

    void calcChecksum(unsigned char* CK);


protected slots:

     void gps_timer();


public slots:

     void Server_to_GPS (QByteArray point);

     void GSM_to_GPS ();

     void I2C_to_GPS (QByteArray data);


signals:

     void GPS_to_SLAM (QByteArray point);

     void GPS_to_Server (QByteArray point);

     void GPS_to_I2C ( QByteArray data );

     void GPS_to_GSM ( double lat_, double lon_ );

private:

    void telemetry();

};
#endif // UART_GPS_H
