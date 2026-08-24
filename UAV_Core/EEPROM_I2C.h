#ifndef EEPROM_I2C_h
#define EEPROM_I2C_h

#include <inttypes.h>
#include <unistd.h>

struct EEPROM_I2C{

public:
		/**
		 * Begins the Wire interface to the given device.
		 *
		 * It will try to guess page size and address word size based on the size of the device.
		 * 
		 * @param deviceAddress Byte address of the device.
		 * @param deviceSize    Max size in bytes of the device (divide your device size in Kbits by 8)
		 */
        void begin(uint8_t deviceAddress, unsigned int deviceSize);
		
        void writeByte(uint8_t eeaddresspage, uint8_t data);
        uint8_t readByte(uint8_t eeaddresspage);

        void writeFloat (uint8_t adress, float data);
        float readFloat (uint8_t adress);

                template< typename T > T &get (int ee, T &value)
                 {
                     start(); uint8_t* p = (uint8_t*)(void*) &value;

                     for( int count = sizeof(T) ; count ; --count ) *p++ = readByte(ee++);

                     return value;
                 }

                template< typename T > const T &put (int ee, const T &value)
                {

                    start(); const uint8_t* p = (const uint8_t*)(const void*) &value;

                    for( int count = sizeof(T) ; count; --count ) {

                    writeByte(ee++, *p++); usleep(2000); }

                    return value;
                }

       uint8_t fd;

   protected:

        void set_adress(uint8_t eeaddress);
        void start();

	private:

        uint8_t deviceAddress;
        uint8_t pageSize;
		bool isAddressSizeTwoWords;
};

#endif
