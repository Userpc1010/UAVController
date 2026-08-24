#include "EEPROM_I2C.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/fs.h>

#define I2C_SLAVE	0x0703	//Регистр I2C мастер

void EEPROM_I2C::begin(uint8_t deviceAddress, unsigned int deviceSize) {

   this->deviceAddress = deviceAddress;
   
  // Чипы 16Kbit (2048KB) или меньше имеют адреса только из одного байта.
  //Также можно попытайтесь угадать размер страницы по размеру устройства (см. Таблицы Microchip 24LCXX здесь).
  if (deviceSize > 256 * 8) {
    this->isAddressSizeTwoWords = true;
    this->pageSize = 32;
  }
  else {
    this->isAddressSizeTwoWords = false;

    if (deviceSize <= 256) {
      this->pageSize = 8;
    }
    else {
      this->pageSize = 16;
    }
  }

}

void EEPROM_I2C::writeByte(uint8_t eeaddress, uint8_t data){

    if(!this->isAddressSizeTwoWords) {

        uint8_t buf[2] = { eeaddress & 0xff, data };

            write(fd, &buf, 2);


    } if (this->isAddressSizeTwoWords ) {

        uint8_t buf[2] =  { eeaddress >> 8 , data };

            write(fd, &buf, 2);
    }

}

uint8_t EEPROM_I2C::readByte(uint8_t eeaddress){

  uint8_t rdata = 0;

  set_adress(eeaddress);

  read( fd, &rdata, 1);

  return rdata;
}

void EEPROM_I2C::set_adress(uint8_t eeaddress){

  if (this->isAddressSizeTwoWords) {

   uint16_t High = eeaddress >> 8;

   write(fd, &High, 1); // Адресс со сташим байтом
   }

  if (!this->isAddressSizeTwoWords) {
   uint16_t Low = eeaddress & 0xFF;

   write(fd, &Low, 1);  // Адресс с младшим байтом (только для чипов 16K или менее т.к. адресс состоит из одного байта)

  }
}

void EEPROM_I2C::start()
{
  ioctl(fd, I2C_SLAVE, this->deviceAddress);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------//
//=================================================== Функция сохранения чисел с плавающей точкой ===========================================================//
//----------------------------------------------------------------------------------------------------------------------------------------------------------//
void EEPROM_I2C::writeFloat (uint8_t adress, float data)
{
 start(); uint8_t *x = (uint8_t *)&data;

 for (uint8_t i =0; i<4; i++)
 {
   writeByte( i + adress, x[i]);

   usleep(2000);
 }
}

float EEPROM_I2C::readFloat (uint8_t adress)
{
  start(); uint8_t x[4];

  for (uint8_t i = 0; i < 4; i++)x[i] = readByte(i + adress);
  float *y = (float *) &x;

  return y[0];
}
