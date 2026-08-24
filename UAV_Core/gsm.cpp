#include <QTimer>
#include "gsm.h"
#include <wiringSerial.h>
#include <wiringPi.h>
#include <errno.h>

#define boot 6 // Контакт для упревления пином загрузки GSM под номером 22 GP5C3

 GSM::GSM(QObject *parent) : QObject(parent)
{
    QTimer *timer_gsm = new QTimer(this);

    connect(timer_gsm, SIGNAL(timeout()), this, SLOT (gsm_timer()));

//    Start_GSM();

//    timer_gsm->start(500);
}

 GSM::~GSM()
 {
  serialClose(fd4);
 }

 void GSM::Start_GSM()
{

 wiringPiSetup();

 fd4 = serialOpen ("/dev/ttyS4", 115200);

 pinMode( boot , OUTPUT);

 serialPuts(fd4, "AT+CPAS\r"); //Проверка состояния готовности

 qDebug()<<"GSM запущен";

}


void GSM::GSM_() //Приём сообщений от модема
{
     data.clear();

     while(serialDataAvail(fd4)) data += serialGetchar(fd4);

    if ( data == "\r\nMODEM:STARTUP\r\n") { last = -2; type_client = 0; ini = false; sim_error = false; qDebug()<< "Загрузка GSM...";  GSM_to_Server("GSM_load"); }

    if ( data == "\r\n+PBREADY\r\n") Initializing_GSM(1);

    if ( data == "AT+CLIP=1\r\r\nOK\r\n") Initializing_GSM(2);

    if ( data == "AT+CMGF=1\r\r\nOK\r\n") Initializing_GSM(3);

    if ( data == "AT+CSCS=\"GSM\"\r\r\nOK\r\n") Initializing_GSM(4);

    if ( (data ==  "\r\nRING\r\n\r\n+CLIP: \"79187910029\",145,,,\"\",0\r\n") && ini ){ type_client = 1; send_cordinat_SMS(1);} // Звонок

    if ( (data ==  "\r\nRING\r\n\r\n+CLIP: \"79187675957\",145,,,\"\",0\r\n") && ini ){ type_client = 2; send_cordinat_SMS(1);} // Звонок

    if ( data == "\r\nNO CARRIER\r\n") {  GSM_to_Server("Call");   qDebug()<< "Вызов окончен"; }

    if ( data == "\r\nRING\r\n" ) { last++;  GSM_to_Server("Un_call");  qDebug()<< "Неизвесный вызов"; };

    if ( data == "ATH\r\r\nOK\r\n") send_cordinat_SMS(2);

    if ( data == "AT+CMGS=\"+79187910029\"\r\r\n> ") send_cordinat_SMS(3);

    if ( data == "AT+CMGS=\"+79187675957\"\r\r\n> ") send_cordinat_SMS(3);

    if ( data == "AT+CPAS\r\r\n+CPAS: 0\r\n\r\nOK\r\n" || data == "AT+CPAS\r\r\n+CPAS: 3\r\n\r\nOK\r\n" || data == "AT+CPAS\r\r\n+CPAS: 4\r\n\r\nOK\r\n") last = 0;

    if ( data == "AT+CPAS\r\r\n+CPAS: 1\r\n\r\nOK\r\n" ) { last = 0; sim_error = true; GSM_to_Server("GSM_no_sim");  qDebug()<< "Симкарта не определена";} //Могут быть проблемы т.к. когда модем не готов нужен сброс

    if (count >= 60) { count = 0;

    serialPuts(fd4, "AT+CPAS\r"); last++;

    if (!ini && !sim_error) last = 3;

    }
}


void GSM::Initializing_GSM (const uint8_t step) // Подготовка модема к отправке сообщений и АОН
{

    if ( step == 1 ) serialPuts(fd4, "AT+CLIP=1\r");

    if ( step == 2 ) serialPuts(fd4, "AT+CMGF=1\r");

    if ( step == 3 ) serialPuts(fd4, "AT+CSCS=\"GSM\"\r" );

    if ( step == 4 ) { ini = true; GSM_to_Server("GSM_init"); emit GSM_to_GPS(); qDebug()<< "Модем в режиме ожидания";}

    last = 0;

}

void GSM::reset(bool res) // Сброс модема
{    
     if(res){ qDebug()<< "Сброс"; sim_error = false; digitalWrite(boot, HIGH); } //Элемент сброса GSM модема, что-бы не тормозить поток на 0.5с завязан на таймер

     else digitalWrite (boot, LOW);
}

void GSM::send_cordinat_SMS ( const uint8_t step ) //Отправка координат
{

   if (step == 1 && type_client > 0) { serialPuts(fd4, "ATH\r");

     emit GSM_to_GPS(); //Получаем координаты

     }

   if (step == 2 && type_client == 1) {

       QString hed = "AT+CMGS=\"+79187910029\"""\r";

       serialPuts(fd4, hed.toUtf8());
   }

   if (step == 2 && type_client == 2) {

       QString hed = "AT+CMGS=\"+79187675957\"""\r";

       serialPuts(fd4, hed.toUtf8());
   }

   if (step == 3 && type_client > 0) {

       QString sms = "lon: "+QString("%1").arg( lon, 0, 'f', 7 )+" lat: "+ QString("%1").arg( lat, 0, 'f', 7 );

       serialPuts(fd4, sms.toUtf8());

       const unsigned char c = 26;

       serialPutchar(fd4, c);

       GSM_to_Server("GSM_send");

       send_cordinat_SMS(4);

       type_client = 0;

       serialPuts(fd4, "AT+CMGD=1,3\r"); //Стирание всех сообщений

       qDebug()<< "Координаты отправлены";
   }

   last = 0;
}

void GSM::gsm_timer()
{  
    count++;

    GSM_(); // Приём очередных данных GSM

    reset(false);

    if (last > 3) { last = 0; reset(true);} //Сброс модема если спит или не инициализирован
}

void GSM::GPS_to_GSM(double lat_, double lon_)
{
   lat = lat_; lon = lon_;
}
