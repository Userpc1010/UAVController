#ifndef I2C_PORT_1_H
#define I2C_PORT_1_H

#include <timer.h>
#include <Data.h>
#include <ms5611.h>
#include <EEPROM_I2C.h>
#include <pca9685.h>
#include <BNO080.h>
#include <utilitiy/tools.h>


class I2C_Ports : public QObject
{

   Q_OBJECT

public:

    I2C_Ports(QObject *parent = nullptr);
     ~I2C_Ports();

private:

    void Start_I2C_1();

    void Start_I2C_4();

private slots:

     void port_1_timer();

public slots:

    void Server_to_I2C(QByteArray data);

    void GPS_to_I2C(QByteArray data);

    void Lidar_to_I2C (int distance);

signals:

    void I2C_to_Server (QByteArray data);

    void I2C_to_SLAM (QByteArray data);

    void I2C_to_GPS (QByteArray data);

private:

 void Start_calibration();
 void Stop_calibration();

 void save_pid();
 void load_pid();
 void Del_PID_Data ();

 void save_limit();
 void load_limit();

 void save_trimm();
 void load_trimm();

 void trottle_init();

 void I2C_Send();

 template< typename T > T & send_struct (const char * mask, const T &struc)
 {
     buffdata.clear();

     buffdata += mask;

     uint8_t * b = (uint8_t*) &struc; for(size_t i = 0; i < sizeof(struc); i++)  buffdata += *b++;

     I2C_to_Server(buffdata);
 }

 void Bank_pid (float yaw );
 void Alt_pid (float alt );
 void PIFF ();
 void Fly_metod ();

 QByteArray buffdata;

 Timer timer;
 uint64_t Time = 0;

 BNO080 bno;
 EEPROM_I2C EEPROM;
 PCA9685 PWM;
 Tools tools;
 MS5611 ms5611;
 Current_attitude attitude;
 Move move;

 PID_ PID;
 Trimm_ Trimm;
 Limit_ Limit;

 uint8_t Cheksumm = 0;
 uint8_t PWM_init_count = 251;

 const char sp_PID [4] = "PID";
 const char sp_Lim [4] = "LIM";
 const char sp_Trimm [5] = "TRIM";

 bool calibration_start = false, calibration_stop = false;

 float Pitch_pid_out = 0; //Выходы ПИД регуляторов
 float Roll_pid_out = 0;
 float Yaw_pid_out = 0;

 //Время после последих вычислений
 uint64_t Last_Timer = 0;

 uint16_t time_charg_Timer = 0;

 int16_t set_point_pitch = 0; //Заданное значение ПИД
 int16_t set_point_roll =  0;
 int16_t set_point_yaw =  0;
 float   set_point_alt =  0;
 int16_t set_point_bank =  0;

 uint8_t climb_angle = 0;  //Угол атаки
 int8_t vifish_angle = 0; //Скрость (угол) поворота в ручном режиме

 bool Manual = false;  //Ручной режим упреавления

 float shiftBuf_pitch [fir_filterLength] = {0, 0, 0, 0, 0};
 float shiftBuf_roll [fir_filterLength] =  {0, 0, 0, 0, 0};
 float shiftBuf_yaw  [fir_filterLength] =  {0, 0, 0, 0, 0};
 float shiftBuf_alt  [fir_filterLength] =  {0, 0, 0, 0, 0};
 float shiftBuf_bank [fir_filterLength] =  {0, 0, 0, 0, 0};

 int8_t baro_counter = -1;

};

#endif // I2C_PORT_1_H
