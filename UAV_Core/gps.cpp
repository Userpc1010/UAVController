#include <QTimer>
#include "gps.h"
#include <errno.h>
#include <math.h>
#include <unistd.h>
#include <wiringSerial.h>

#define scale_c 10000000.0f
#define scale_h 100000.0f
#define scale_v 1000.0f
#define LOS_cross_angle_error 10000.0f
#define Acc_accuracy 0.00016197093
#define Gps_accuracy 1.49010849

//Можно попробовать использовать координаты ECEF из UBX-NAV-SOL тому потверждение https://github.com/gdlow/pixhawk-android-gps-follower
//Возможно это увеличит точность фильтра но пока так.

GPS::GPS(QObject *parent) : QObject(parent)
{
  QTimer * timer_gps = new QTimer(this);

  connect (timer_gps, SIGNAL( timeout()), this, SLOT (gps_timer()));

  Start_GPS();

  timer_gps->start(200);
}

void GPS::translit_to_SI()
{
    si.raw_lat = pvt.lat / scale_c; si.raw_lon = pvt.lon / scale_c; si.heading = pvt.heading / scale_h; si.headingAcc = pvt.headingAcc/ scale_h; si.g_speed = pvt.gSpeed/ 1000.0f;
    si.N_vel = pvt.velN / scale_v; si.E_vel = pvt.velE / scale_v; si.D_vel = pvt.velD / scale_v; si.h_acc = pvt.hAcc / scale_v;
}

GPS::~GPS()
{
 serialClose(fd3);
}

void GPS::Start_GPS()
{
  fd3 = serialOpen ("/dev/ttyS3", 9600); //fd3 = serialOpen ("/dev/ttyACM0", 115200);

  for(uint16_t i = 0; i < sizeof(UBLOX_INIT); i++) { serialPutchar(fd3, UBLOX_INIT[i] );}

  serialClose(fd3);

  fd3 = serialOpen ("/dev/ttyS3", 115200);

  qDebug()<<"GPS запущен";
}

float GPS::LOS_guidance_law (double curent_lat, double curent_lon, double next_lat, double next_lon, double previus_lat, double previus_lon, uint8_t counter)
{
   //qDebug()<<"curent_lat"<<QString("%1").arg(curent_lat, 0, 'f',  7)<<"curent_lon"<<QString("%1").arg(curent_lon, 0, 'f',  7)<<"next_lat "<<QString("%1").arg(next_lat, 0, 'f',  7)<<"next_lon"<<QString("%1").arg(next_lon, 0, 'f',  7)<<"previus_lat"<<QString("%1").arg(previus_lat, 0, 'f',  7)<<"previus_lon"<<QString("%1").arg(previus_lon, 0, 'f',  7);

   if (counter > 0) {

   float LOS = atan2(next_lon - previus_lon, next_lat - previus_lat);

   float LOP = (LOS - atan2(next_lon - curent_lon, next_lat - curent_lat));

   float S0 = tools.geoDistance (previus_lat, previus_lon, curent_lat, curent_lon) * sin(LOS - atan2(curent_lon - previus_lon, curent_lat - previus_lat));

   float cross_error_angle = asin( S0 / sqrt(LOS_cross_angle_error + S0 * S0));

   if (LOP > 1.570796326794 || LOP < -1.570796326794) return tools.geoBearing(next_lat, next_lon, curent_lat, curent_lon);

   else {  LOS = cross_error_angle + LOS;

   if (LOS > 3.141592653589) LOS -= 6.283185307179;
   if (LOS < -3.141592653589) LOS += 6.283185307179;

   return LOS * RADTODEG;
   }
   }
   else return tools.geoBearing(next_lat, next_lon, curent_lat, curent_lon);
}

float GPS::circle (double axis_lat, double axis_lon, double curent_lat, double curent_lon, double radius_circle, double radius_point, float heading)
{
   vec_2 point;

   heading += 90;

  if (heading > 180.0f) heading-= 360.0f;
  if (heading < -180.0f) heading += 360.0f;

  if (first_start) {axis_lat = si.lat; axis_lon = si.lon; current.lat = si.lat; current.lon = si.lon; } //Циркуляризация на месте пуска

 if (axis_lon != last_axis_lon || axis_lat != last_axis_lat ) {
 for (int i = 0; i <= 5; i++ ){

     if ( radius_circle < 5 ) radius_circle = 5;

     float direction =  heading + i * 60;

     if (direction > 180.0f) direction -= 360.0f;
     if (direction < -180.0f) direction += 360.0f;

     point = tools.Point_Distance_Direct_to_Point( axis_lon, /* Долгота*/ axis_lat, /* Широта */ radius_circle, direction);

     lat_[i] = point.lat; lon_[i] = point.lon;

     //qDebug()<<" bering: "<<direction<<" Lat: "<<QString("%1").arg(lat_[i], 0, 'f',  7)<<" Lon: "<<QString("%1").arg(lon_[i], 0, 'f',  7);
 }
     last_axis_lat = axis_lat; last_axis_lon = axis_lon; qDebug()<<"heading: "<<heading<<"axis: "<<QString("%1").arg(axis_lat, 0, 'f',  7)<<" : "<<QString("%1").arg(axis_lon, 0, 'f',  7);
 }
     if (tools.geoDistance (lat_[circle_stage_1], lon_[circle_stage_1], curent_lat, curent_lon) < radius_point) circle_stage_1++;

     if(circle_stage_1 >= 6) circle_stage_1 -= 6;

     float Bering = tools.geoBearing(lat_[circle_stage_1], lon_[circle_stage_1], curent_lat, curent_lon );

     //qDebug()<<" distance: "<<QString("%1").arg(geoDistance (lat_[circle_stage_1], lon_[circle_stage_1], curent_lat, curent_lon), 0, 'f',  7)<<" circle stage: "<<circle_stage_1<<" Lat/Lon: "<<QString("%1").arg(lat_[circle_stage_1], 0, 'f',  7)<<" : "<<QString("%1").arg(lon_[circle_stage_1], 0, 'f',  7);

     //qDebug()<<" Bering: "<< Bering;

     return Bering;
}

void GPS::calcChecksum(unsigned char* CK) {
  memset(CK, 0, 2);
  for (int i = 0; i < (int)sizeof(NAV_PVT); i++) {
    CK[0] += ((unsigned char*)(&pvt))[i];
    CK[1] += CK[0];
  }
}

void GPS::gps_timer()
{
    while ( serialDataAvail(fd3) ) {
       uint8_t c = serialGetchar(fd3);
         if ( fpos < 2 ) {
           if ( c == UBX_HEADER[fpos] )
             fpos++;
           else
             fpos = 0;
         }
         else {
           if ( (fpos-2) < payloadSize )
             ((unsigned char*)(&pvt))[fpos-2] = c;

           fpos++;

           if ( fpos == (payloadSize+2) ) {
               calcChecksum(checksum);
           }
           else if ( fpos == (payloadSize+3) ) {
             if ( c != checksum[0] )
               fpos = 0;
           }
           else if ( fpos == (payloadSize+4) ) {
             fpos = 0;
             if ( c == checksum[1] )
             {
                 translit_to_SI();
                 if(first_start) kf2 = GPSAccKalmanAlloc(tools.CoordLongitudeToMeters(si.raw_lon), tools.CoordLatitudeToMeters(si.raw_lat), si.E_vel, si.N_vel, 0.32, si.h_acc , timer.Time_microseconds_d());
                 if(!first_start) GPSAccKalmanUpdate(kf2, timer.Time_microseconds_d(), tools.CoordLongitudeToMeters(si.raw_lon), tools.CoordLatitudeToMeters(si.raw_lat), si.E_vel, si.N_vel, si.h_acc);
                 gps_reset_count = 0; gps_reset_count_2 = 0; first_start = false;              
             }
           }
           else if ( fpos > (payloadSize+4) ) {
             fpos = 0;
           }
         }
       }


    gps_reset_count++;
    if (gps_reset_count > 5){ gps_reset_count = 0; gps_reset_count_2++; for(uint16_t i = 0; i < sizeof(UBLOX_INIT); i++) { serialPutchar(fd3, UBLOX_INIT[i] );} if(gps_reset_count_2 > 2){ Start_GPS(); gps_reset_count_2 = 0;}}
    telemetry();
}

void GPS::Server_to_GPS(QByteArray point) //Приём маршрута из GUI карты
{
    if ( point == "dr" ) { if(!array_rout.isEmpty())array_rout.clear(); current.rout_count = 0; }

    if ( point == "st" ) UpdateNavPiont(current.fly_point = 0);

    if ( point == "rt" ) UpdateNavPiont(current.fly_point = 2);

    if (point [0] == 'M' && point[1] == 'O' && point[2] == 'V' && point[3] == 'E' )

    { uint8_t ee = 4; uint8_t* p = (uint8_t*)(void*)&_move; for( int count = point.length(); count ; --count ) *p++ = point[ee++]; UpdateNavPiont(current.fly_point = 1); current.move_count = 0; }

    if (point[0] == 'R' && point[1] == 'O' && point[2] == 'U' && point[3] == 'T' )

     { current.rout_length = (uint16_t)(point[5] << 8 | point[4]);  uint8_t ee = 6; uint8_t* p = (uint8_t*)(void*)&_rout; for( int count = point.length(); count ; --count ) *p++ = point[ee++]; array_rout.push_back(_rout);

     point.clear(); point += "Reciv";  uint16_t bof = array_rout.length(); point += static_cast<uint8_t >(bof & 0xFF); point += static_cast<uint8_t >(bof >> 8); GPS_to_Server(point); if(current.rout_length == array_rout.length()) UpdateNavPiont(current.fly_point = 2); }
}

void GPS::GoToNavPoint()
{
   Distance_to_point = tools.geoDistance( current.lat, current.lon, si.lat, si.lon );

   if ( current.fly_point == 1 && ( Distance_to_point < current.radius_point )) {current.move_count++; UpdateNavPiont(current.fly_point);}// Если расстояние меньше 6м и можно вызывать следующую точку то вызываем, нет циркуляризируемся

   if (current.fly_point == 2 && ( Distance_to_point < current.radius_point )) {current.rout_count++; UpdateNavPiont(current.fly_point);}

   if (current.fly_point != 0)  Move_line(current.fly_point); //qDebug()<<"Движение"; }

   if (current.fly_point == 0)  Move_cirle(); //qDebug()<<"Циркуляризация"; }
}

void GPS::Move_line(uint8_t fly_point)
{
    _move_.alt = current.alt;
    if (fly_point == 1)_move_.azimut = tools.geoBearing(current.lat, current.lon, si.lat, si.lon );
    if (fly_point == 2)_move_.azimut = LOS_guidance_law(si.lat, si.lon, current.lat, current.lon, current.previus_lat, current.previus_lon, current.rout_count);
    _move_.climb = current.climb;
    _move_.type_point = current.type_point;

    QByteArray data;

    uint8_t* p = (uint8_t*)(void*)&_move_; for( int count = sizeof(_move_); count ; --count ) data += *p++;

    GPS_to_I2C (data);
}

void GPS::Move_cirle() //Вращения самолёта вокруг точки
{
    _move_.alt = current.alt;   // Высота для DUE
    _move_.azimut = circle(current.lat, current.lon, si.lat, si.lon, current.radius_circl, current.radius_point, attitude.yaw); //Радиус вращения вокруг точки
    _move_.climb = current.climb;  //Угол атаки
    _move_.type_point = current.type_point; //Тип точки

    QByteArray data;

    uint8_t* p = (uint8_t*)(void*)&_move_; for( int count = sizeof(_move_); count ; --count ) data += *p++;

    GPS_to_I2C (data);

}

int GPS::UpdateNavPiont(int type ) //Получение точек из парсера
{
  if (type == 1 && _move.lon != 0 && _move.lat != 0 && 0 == current.move_count ) // Идти к точке
  {
     current.lon =           _move.lon;
     current.lat =           _move.lat;
     current.alt =           _move.alt;
     current.climb =         _move.climb;
     current.type_point =    _move.type_point;
     current.radius_point =  _move.radius_point;
     current.radius_circl =  _move.radius_circl;

     return current.fly_point = 1;
  }

  if (type == 2 && (array_rout.length() > current.rout_count)) // Идти по маршруту
  {
      qDebug()<<"Точек достигнуто: "<< current.rout_count << " точек заргружено: " << array_rout.length()<< " точек доступно: "<< current.rout_length;

      uint8_t* p = (uint8_t*)(void*)&array_rout[current.rout_count]; uint8_t* b = (uint8_t*)(void*)&_rout; for( int count = sizeof(_rout); count ; --count ) *b++ = *p++;

      current.previus_lon =    current.lon;
      current.previus_lat =    current.lat;
      current.lon =            _rout.lon;
      current.lat =            _rout.lat;
      current.climb =          _rout.climb;
      current.alt =            _rout.alt;
      current.type_point =     _rout.type_point;
      current.radius_point =   _rout.radius_point;
      current.radius_circl =   _rout.radius_circl;

      return current.fly_point = 2;
    }

     qDebug()<<"Конец маршрута достигнута точка: "<<current.rout_count;

     return current.fly_point = 0;
  }

void GPS::telemetry()
{
    QByteArray data;

    data += "TELE";

    float turn_2  = tools.geoBearing(current.lat, current.lon, si.lat, si.lon ) - si.heading;

    if (turn_2 < -180) turn_2 += 360.0f;
    if (turn_2 >  180) turn_2 -= 360.0f;

    float turn_1  = _move_.azimut - attitude.yaw;

    if (turn_1 < -180) turn_1 += 360.0f;
    if (turn_1 >  180) turn_1 -= 360.0f;

    Tele_w.lon = si.lon;
    Tele_w.lat = si.lat;
    Tele_w.altitude = attitude.altitude_compensated_lidar_bar;
    Tele_w.g_speed = si.g_speed;
    Tele_w.dist_to_point = Distance_to_point;
    Tele_w.mismatch = turn_1;
    Tele_w.pitch = attitude.pitch;
    Tele_w.roll = attitude.roll;
    Tele_w.yaw = attitude.yaw;
    Tele_w.gps_mismatch = turn_2;
    Tele_w.cal_system = attitude.QuatAccuracy;
    Tele_w.cal_gyro = attitude.QuatRadianAccuracy;
    Tele_w.cal_accel = attitude.LinAccelAccuracy;
    Tele_w.cal_mag = attitude.calibrationComplete;

    uint8_t* b = (uint8_t*)(void*)&Tele_w; for( int count = sizeof(Tele_w); count ; --count ) data += *b++;

    GPS_to_Server (data);
}

void GPS::GSM_to_GPS()
{
    emit GPS_to_GSM( si.lat, si.lon);
}

void GPS::I2C_to_GPS(QByteArray data)
{
    uint8_t ee = 0; uint8_t* p = (uint8_t*)(void*)&attitude; for( int count = data.length() - ee; count ; --count ) *p++ = data[ee++];

    Positional();
}

void GPS::Positional()
{
  QByteArray data;

  QVector3D Body_vector;

  //Трансформация вектора ускорений систему координат NED

  Body_vector = tools.Quaternion_rotate(SLAM.rotation, attitude.linear);

  GPSAccKalmanPredict(kf2, timer.Time_microseconds_d(), Body_vector.y(), Body_vector.x());

  vec_2 pp = tools.CoordMetersToGeopoint(GPSAccKalmanGetX(kf2),GPSAccKalmanGetY(kf2));

  si.lat = pp.lat; si.lon = pp.lon; GoToNavPoint(); // Обновление координат движения

  //qDebug()<<"Vec: "<<attitude.linear[0]<<"  :  "<<attitude.linear[1]<<"  :  "<<attitude.linear[2];

  //Пересчёт координат широты долготы в декартовые координаты SLAM

  //float x = (1000000.0f * cos(si.lat * DEGTORAD) * cos(si.lon * DEGTORAD)) - 524786.0f;

  //float y = (1000000.0f * cos(si.lat * DEGTORAD) * sin(si.lon * DEGTORAD)) - 471426.0f;

  //SLAM.positional = Body_vector;

 // qDebug()<<"Kalman "<<GPSAccKalmanGetX(kf2)<<"  "<<GPSAccKalmanGetY(kf2);

  SLAM.rotation = attitude.rotation;

  uint8_t* b = (uint8_t*)(void*)&SLAM; for( int count = sizeof(SLAM); count ; --count ) data += *b++;

  GPS_to_SLAM (data);
}
