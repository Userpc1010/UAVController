#include "ms5611.h"
#include <sys/ioctl.h>
#include <unistd.h>
#include <math.h>
#include <QDebug>

#define MS5611_ADDRESS                0x77

#define MS5611_CMD_ADC_READ           0x00
#define MS5611_CMD_RESET              0x1E
#define MS5611_CMD_CONV_D1            0x40
#define MS5611_CMD_CONV_D2            0x50
#define MS5611_CMD_READ_PROM          0xA2

// check daily sea level pressure at 2021.01.31.02:00
// http://www.kma.go.kr/weather/observation/currentweather.jsp
#define SEA_LEVEL_PRESSURE 102865

#define I2C_SLAVE	0x0703	/* Use this slave address */

MS5611::MS5611()
{

}

bool MS5611::begin(ms5611_osr_t osr)
{
    ioctl(fd, I2C_SLAVE, MS5611_ADDRESS);

    reset();

    setOversampling(osr);

    readEPROM();

    return true;
}

// Set oversampling value
void MS5611::setOversampling(ms5611_osr_t osr)
{
    switch (osr)
    {
    case MS5611_ULTRA_LOW_POWER:
        ct = 1;
        break;
    case MS5611_LOW_POWER:
        ct = 2;
        break;
    case MS5611_STANDARD:
        ct = 3;
        break;
    case MS5611_HIGH_RES:
        ct = 5;
        break;
    case MS5611_ULTRA_HIGH_RES:
        ct = 10;
        break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t MS5611::getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void MS5611::reset()
{

  writeByte(MS5611_CMD_RESET);

  usleep(10000);
}

void MS5611::readEPROM(void)
{
    for (uint8_t offset = 0; offset < 6; offset++)
    {
    fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}

uint32_t MS5611::readRawPressure(int8_t counter)
{
    if(counter == 0) writeByte( MS5611_CMD_CONV_D1 + uosr);

    if(counter == 1) pressure = readRegister24(MS5611_CMD_ADC_READ);

    return pressure;
}

uint32_t MS5611::readRawTemperature(int8_t counter)
{

   if(counter == 1) writeByte( MS5611_CMD_CONV_D2 + uosr);

   if(counter == 2) temperature = readRegister24(MS5611_CMD_ADC_READ);

   if(counter == 2) writeByte( MS5611_CMD_CONV_D1 + uosr);

   return temperature;
}

int32_t MS5611::readPressure(int8_t counter)
{
    ioctl(fd, I2C_SLAVE, MS5611_ADDRESS);

    readRawPressure(counter);

    readRawTemperature(counter);

    if(counter == 1) {

    int32_t dT = temperature - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    OFF2 = 0;
    SENS2 = 0;

    if (TEMP < 2000)
    {
        OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
        SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
    }

    if (TEMP < -1500)
    {
        OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
        SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
    }

    OFF = OFF - OFF2;
    SENS = SENS - SENS2;

    P = (pressure * SENS / 2097152 - OFF) / 32768;
}

    return P;
}

float MS5611::readTemperature(int8_t counter)
{
    ioctl(fd, I2C_SLAVE, MS5611_ADDRESS);

    uint32_t D2 = readRawTemperature(counter);

    if(counter == 2){

    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (TEMP < 2000)
    {
        TEMP2 = (dT * dT) / (2 << 30);
    }

    TEMP = TEMP - TEMP2;

    }

    return ((double)TEMP/100);
}



// Calculate altitude from Pressure & Sea level pressure
float MS5611::getAltitude(int8_t counter)
{
   altitude = (44330.0f * (1.0f - powf((double)readPressure(counter)/SEA_LEVEL_PRESSURE, 0.1902949f)));

   return tools.moving_average(altitude - (altitude - altitude_lidar), buffer, size);
}

void MS5611::altitude_offset(float alt)
{
  altitude_lidar = alt;
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t MS5611::readRegister16(uint8_t reg)
{
    uint8_t rawData[2];

    readBytes(reg, 2, &rawData[0]);

    return (rawData[0] << 8 | rawData[1]);
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t MS5611::readRegister24(uint8_t reg)
{
    uint8_t rawData[3];

    readBytes(reg, 3, &rawData[0]);

    return ((int32_t)rawData[0] << 16 | ((int32_t)rawData[1] << 8) | rawData[2]);
}

void MS5611::writeByte(uint8_t data)
{
   if(write(fd, &data, 1) != 1) qDebug()<<"Ошибка записи в регистр I2C Метод writeByte MS5611";
}

void MS5611::readBytes( uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    if(write(fd, &subAddress, 1) != 1) qDebug()<<"Ошибка записи в регистр I2C: "<<subAddress << " Метод readBytes MS5611";

    if(read(fd, dest, count) != count) qDebug()<<"Ошибка чтения из регистра I2C: "<<subAddress << " Метод readBytes MS5611";
}
