#include <inttypes.h>
#include <utilitiy/tools.h>

#ifndef MS5611_H
#define MS5611_H

typedef enum
{
    MS5611_ULTRA_HIGH_RES   = 0x08,
    MS5611_HIGH_RES         = 0x06,
    MS5611_STANDARD         = 0x04,
    MS5611_LOW_POWER        = 0x02,
    MS5611_ULTRA_LOW_POWER  = 0x00
} ms5611_osr_t;


class MS5611
{

public:

MS5611();

    bool begin(ms5611_osr_t osr = MS5611_ULTRA_HIGH_RES);
    uint32_t readRawTemperature(int8_t counter);
    uint32_t readRawPressure(int8_t counter);
    float readTemperature(int8_t counter);
    int32_t readPressure(int8_t counter);
    float getAltitude(int8_t counter);
    void setOversampling(ms5611_osr_t osr);
    ms5611_osr_t getOversampling(void);
    void altitude_offset(float alt);

    uint8_t fd;

    private:

    uint8_t ct;
    uint16_t fc[6];
    uint8_t uosr;
    int32_t TEMP2;
    int64_t OFF2, SENS2;

    uint32_t temperature = 0;
    uint32_t pressure = 0;
    float altitude = 0.0f;
    float altitude_lidar = 0.0f;
    uint32_t P = 0;
    uint32_t TEMP = 0;

    static const uint8_t size = 5;
    float buffer[size];

    void reset();
    void readEPROM(void);

    Tools tools;

    uint16_t readRegister16(uint8_t reg);
    uint32_t readRegister24(uint8_t reg);

    void writeByte (uint8_t data);
    void readBytes(uint8_t subAddress, uint8_t count, uint8_t * dest);
};

#endif // MS5611_H
