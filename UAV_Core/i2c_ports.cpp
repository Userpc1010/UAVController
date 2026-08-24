#include "i2c_ports.h"
#include <QTimer>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <iostream>

#define I2C_SLAVE	0x0703	// Включаем режим мастер
#define EE24LC16MAXBYTES 16384/8 //Кол-во банок памяти

#define Left_Vtail 1
#define Right_Vtail 2
#define Left_Aileron 3
#define Right_Aileron 4
#define sq(x) ((x)*(x))
#define Proximity_prop_fuse 10

// Chips 16Kbit (2048KB) or smaller only have one-word addresses.
// Also try to guess page size from device size (going by Microchip 24LCXX datasheets here). èç áèáëèîòåêè
#define DEVICEADDRESS (0x50)

I2C_Ports::I2C_Ports(QObject *parent) : QObject(parent)
{
  nice(-10);

  QTimer * timer = new QTimer(this);

  timer->setTimerType(Qt::PreciseTimer);

  timer->setInterval(11);

  connect (timer, SIGNAL(timeout()),this, SLOT (port_1_timer()));

  Start_I2C_4();

  Start_I2C_1();

  timer->start();
}

I2C_Ports::~I2C_Ports()
{
  close(PWM.fd);
  close(bno.fd);
}

void I2C_Ports::Start_I2C_1()
{
  bno.fd = ms5611.fd = open("/dev/i2c-1", O_RDWR);

  ms5611.begin();

  while(!bno.begin()) { qDebug()<<"Неполадки при загрузке BNO080"; usleep(11111);}

  qDebug()<<"BNO080 запущен";
}

void I2C_Ports::Start_I2C_4()
{
    EEPROM.fd = PWM.fd = open("/dev/i2c-4", O_RDWR);

    EEPROM.begin(DEVICEADDRESS, EE24LC16MAXBYTES );

    load_pid(); load_limit(); load_trimm();

    PWM.begin();
}

void I2C_Ports::Start_calibration()
{  
    calibration_start = true;

    qDebug()<<"Start calibration";
}

void I2C_Ports::Stop_calibration()
{
   calibration_stop = true;

   qDebug()<<"Stop calibration";

}

void I2C_Ports::save_pid()
{
    EEPROM.put(25, PID);
}

void I2C_Ports::load_pid()
{
    EEPROM.get(25, PID);
}

void I2C_Ports::Del_PID_Data()
{
  for (int i = 25; i < 65; i++ ) EEPROM.writeByte( i, 0 );
}

void I2C_Ports::save_limit()
{
    EEPROM.put(71, Limit);
}

void I2C_Ports::load_limit()
{
    EEPROM.get(71, Limit);

    bno.Offset(Trimm.Offset_roll, Trimm.Offset_pitch, Trimm.Magnetick_decclination);
}

void I2C_Ports::save_trimm()
{
    EEPROM.put(131, Trimm);
}

void I2C_Ports::load_trimm()
{
    EEPROM.get(131, Trimm);
}

void I2C_Ports::trottle_init()
{
  if (PWM_init_count <  250) {PWM.write_to_deg_Trottle(255); PWM_init_count++;}
  if (PWM_init_count == 250) {PWM.write_to_deg_Trottle(0);   PWM_init_count++;}
}

void I2C_Ports::I2C_Send()
{
  buffdata.clear();

  uint8_t* p = (uint8_t*)(void*)&attitude; for( int count = sizeof(attitude); count ; --count ) buffdata += *p++;

  I2C_to_GPS(buffdata);
}

void I2C_Ports::port_1_timer()
{
  trottle_init(); Time = timer.Time_microseconds(); time_charg_Timer = Time - Last_Timer; Last_Timer = Time;

  if (time_charg_Timer > 8000) { //SHTP не приемлит преждевременное чтение регистров а не стабильность таймера +/-5мс

  baro_counter++; if(baro_counter > 2) baro_counter = 1;

  attitude.altitude_compensated_lidar_bar = ms5611.getAltitude(baro_counter);

  if (bno.dataAvailable()) {

  //attitude.rotation.setScalar(bno.getQuatReal()); attitude.rotation.setX(bno.getQuatI()); attitude.rotation.setY(bno.getQuatJ()); attitude.rotation.setZ(bno.getQuatK());

  attitude.rotation.setScalar(-bno.getQuatReal()); attitude.rotation.setX(bno.getQuatI()); attitude.rotation.setY(-bno.getQuatK()); attitude.rotation.setZ(-bno.getQuatJ());

  attitude.pitch = bno.getPitch(); attitude.roll = bno.getRoll(); attitude.yaw = bno.getYaw();

  attitude.linear[0] = -bno.getLinAccelX(); attitude.linear[1] = -bno.getLinAccelY(); attitude.linear[2] = bno.getLinAccelZ();

  attitude.QuatAccuracy = bno.getQuatAccuracy();

  attitude.QuatRadianAccuracy = bno.getQuatRadianAccuracy();

  attitude.LinAccelAccuracy  =  bno.getLinAccelAccuracy();

  attitude.calibrationComplete = bno.calibrationComplete();

  I2C_Send();

  if(!Manual){Alt_pid(attitude.altitude_compensated_lidar_bar); Bank_pid(attitude.yaw);}

  else { set_point_yaw = attitude.yaw + vifish_angle;

  if (set_point_yaw < -180.0f) set_point_yaw += 360.0f;
  if (set_point_yaw >  180.0f) set_point_yaw -= 360.0f;
  }

  PIFF();

  if(calibration_start){ bno.calibrateAll(); calibration_start = false;}

  if(calibration_stop) {  bno.saveCalibration(); bno.requestCalibrationStatus(); bno.endCalibration(); calibration_stop = false;}

 }
 }
}

void I2C_Ports::Server_to_I2C(QByteArray data)
{
    if ( data == "wd" ) set_point_pitch = 0;
    if ( data == "sd" ) set_point_pitch = 0;
    if ( data == "ad" ) set_point_roll = 0;
    if ( data == "dd" ) set_point_roll = 0;
    if ( data == "qd" ) vifish_angle  = 0;
    if ( data == "ed" ) vifish_angle  = 0;
    if ( data == "su" ) set_point_pitch = 30;
    if ( data == "wu" ) set_point_pitch = -30;
    if ( data == "au" ) set_point_roll = 45;
    if ( data == "du" ) set_point_roll = -45;
    if ( data == "qu" ) vifish_angle  = 30;
    if ( data == "eu" ) vifish_angle  = -30;

    if (data == "cs" ) Start_calibration();
    if (data == "cp" ) Stop_calibration();
    if (data == "dp" ) Del_PID_Data ();
    if (data == "cr" ) PWM_init_count = 0;

    if ( data[0] == 'P' && data[1] == 'I' && data[2] == 'D' && data[3] == '1')
    {
    uint8_t ee = 4; uint8_t* p = (uint8_t*)(void*)&PID; for( int count = data.length() - ee; count ; --count ) *p++ = data[ee++]; save_pid();
    }

    if ( data[0] == 'P' && data[1] == 'I' && data[2] == 'D' && data[3] == '0')
    {
    load_pid(); send_struct(sp_PID, PID);
    }

    if ( data[0] == 'T' && data[1] == 'R' && data[2] == 'I' && data[3] == 'M' && data[4] == '1')
    {
    uint8_t ee = 5; uint8_t* p = (uint8_t*)(void*)&Trimm; for( int count = data.length() - ee; count ; --count ) *p++ = data[ee++];

    save_trimm(); bno.Offset(Trimm.Offset_roll, Trimm.Offset_pitch, Trimm.Magnetick_decclination);

    }

    if ( data[0] == 'T' && data[1] == 'R' && data[2] == 'I' && data[3] == 'M' && data[4] == '0' )
    {
    load_trimm(); send_struct(sp_Trimm, Trimm);
    }

    if ( data[0] == 'L' && data[1] == 'I' && data[2] == 'M' && data[3] == '1' )
    {
    uint8_t ee = 4; uint8_t* p = (uint8_t*)(void*)&Limit; for( int count = data.length() - ee; count ; --count ) *p++ = data[ee++]; save_limit();
    }

    if ( data[0] == 'L' && data[1] == 'I' && data[2] == 'M' && data[3] == '0' )
    {
    load_limit(); send_struct(sp_Lim, Limit);
    }

    if (data == "MN0") Manual = false;
    if (data == "MN1") Manual = true;

    if ( data[0] == 't' && data[1] == 'r') PWM.write_to_deg_Trottle(data[2]);
}

void I2C_Ports::GPS_to_I2C(QByteArray data)
{
    uint8_t ee = 0; uint8_t* p = (uint8_t*)(void*) &move; for( int count = data.length(); count ; --count ) *p++ = data[ee++]; data.clear();

    set_point_alt = move.alt;

    set_point_bank = move.azimut;

    set_point_yaw = move.azimut;

    climb_angle = move.climb;
}

void I2C_Ports::Lidar_to_I2C(int distance)
{
    if ((distance < Proximity_prop_fuse) && move.type_point == 2) PWM.write_to_deg_Trottle(0); //Защита пропелера от падения 10см

    if ((distance > 1)) {

    float roll = abs(attitude.roll);

    float pitch = abs(attitude.pitch);

    if ((roll > 1 || pitch > 1) && ( roll < 40 && pitch < 40 )) {

    float alt = (distance * (1.0f - 2.0f * sq(pitch / 100.0f) - 2.0f * sq(roll / 100.0f))) / 100.0f;

    ms5611.altitude_offset(alt);
    }
  }
}

void I2C_Ports::Bank_pid(float yaw)
{
     float Error = set_point_bank - yaw;

     if (Error < -180.0f) Error += 360.0f; // Делаем результат вычетания вида -180 +180
     if (Error >  180.0f) Error -= 360.0f;

     tools.filterUpdateFIR( shiftBuf_bank, yaw);

     set_point_roll = -(PID.P_Bank * Error + tools.filterApplyFIR ( shiftBuf_bank, yaw, -PID.D_Bank/ (8 * time_charg_Timer)));

     if ( set_point_roll > Limit.Bank_Max ) set_point_roll = Limit.Bank_Max;
     if ( set_point_roll < Limit.Bank_Min ) set_point_roll = Limit.Bank_Min;    
}

void I2C_Ports::Alt_pid(float alt)
{     
     float Error = set_point_alt - alt;

     tools.filterUpdateFIR( shiftBuf_alt, alt);

     set_point_pitch =  PID.P_Alt * Error + tools.filterApplyFIR ( shiftBuf_alt, alt, -PID.D_Alt/ (8 * time_charg_Timer));

     if ( set_point_pitch > climb_angle) set_point_pitch = climb_angle;
     if ( set_point_pitch < -climb_angle) set_point_pitch = -climb_angle;
}

void I2C_Ports::PIFF() //Пид регулятор с прямой обратной связью производной
{
     float Error_Pitch = set_point_pitch - attitude.pitch;
     float Error_Roll = set_point_roll - attitude.roll;
     float Error_Yaw = set_point_yaw - attitude.yaw;

     if (Error_Yaw < -180.0f) Error_Yaw += 360.0f; // Делаем результат вычетания вида -180 +180
     if (Error_Yaw >  180.0f) Error_Yaw -= 360.0f;

     if (Error_Yaw < Limit.Error_Yaw_Min) Error_Yaw = Limit.Error_Yaw_Min;
     if (Error_Roll < Limit.Error_Roll_Min) Error_Roll = Limit.Error_Roll_Min;
     if (Error_Pitch <  Limit.Error_Pitch_Min) Error_Pitch =  Limit.Error_Pitch_Min;

     if (Error_Yaw >  Limit.Error_Yaw_Max) Error_Yaw =  Limit.Error_Yaw_Max;
     if (Error_Roll >  Limit.Error_Roll_Max) Error_Roll =  Limit.Error_Roll_Max;
     if (Error_Pitch >  Limit.Error_Pitch_Max) Error_Pitch =  Limit.Error_Pitch_Max;

     tools.filterUpdateFIR( shiftBuf_pitch, attitude.pitch);
     tools.filterUpdateFIR( shiftBuf_roll, attitude.roll);
     tools.filterUpdateFIR( shiftBuf_yaw, attitude.yaw);

     Pitch_pid_out = PID.P_Pitch * Error_Pitch + tools.filterApplyFIR ( shiftBuf_pitch, attitude.pitch, -PID.D_Pitch / (8 * time_charg_Timer));
     Roll_pid_out =  PID.P_Roll * Error_Roll + tools.filterApplyFIR ( shiftBuf_roll, attitude.roll, -PID.D_Roll / (8 * time_charg_Timer));
     Yaw_pid_out = PID.P_Yaw * Error_Yaw + tools.filterApplyFIR ( shiftBuf_yaw, attitude.yaw, -PID.D_Yaw / (8 * time_charg_Timer));

     if ( Yaw_pid_out > Limit.Yaw_Max )  Yaw_pid_out = Limit.Yaw_Max;
     if ( Roll_pid_out > Limit.Roll_Max )  Roll_pid_out = Limit.Roll_Max;
     if ( Pitch_pid_out > Limit.Pitch_Max )  Pitch_pid_out = Limit.Pitch_Max;

     if ( Yaw_pid_out < Limit.Yaw_Min )  Yaw_pid_out = Limit.Yaw_Min;
     if ( Roll_pid_out < Limit.Roll_Min )  Roll_pid_out = Limit.Roll_Min;
     if ( Pitch_pid_out < Limit.Pitch_Min )  Pitch_pid_out = Limit.Pitch_Min;

     Fly_metod();
}

void I2C_Ports::Fly_metod()
{
//  qDebug()<<"Left_Vtail: "<<  (Yaw_pid_out - Pitch_pid_out)  *  0.5f + Trimm.Left_V_Tail;

//  qDebug()<<"Right_Vtail: "<<  (Yaw_pid_out + Pitch_pid_out) *  0.5f + Trimm.Right_V_Tail;

//  qDebug()<<"Left_Aileron: "<< Roll_pid_out  +  Trimm.Left_Eleron;

//  qDebug()<<"Right_Aileron: "<< Roll_pid_out  +  Trimm.Right_Eleron;

  PWM.write_to_deg_Servo (Left_Vtail, (Yaw_pid_out - Pitch_pid_out)  *  0.5f + Trimm.Left_V_Tail);

  PWM.write_to_deg_Servo (Right_Vtail, (Yaw_pid_out + Pitch_pid_out) *  0.5f + Trimm.Right_V_Tail);

  PWM.write_to_deg_Servo (Left_Aileron, Roll_pid_out  +  Trimm.Left_Eleron);

  PWM.write_to_deg_Servo (Right_Aileron, Roll_pid_out  +  Trimm.Right_Eleron);
}
