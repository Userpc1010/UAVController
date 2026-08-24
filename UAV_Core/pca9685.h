#ifndef PCA9685_H
#define PCA9685_H

#include <inttypes.h>
#include <unistd.h>

class PCA9685
{
  public:

  uint8_t fd;

  PCA9685();

  void begin();

  void setPWMFreq(float freq);

  uint8_t readPrescale(void);

  void reset();

  void start();

  void setOscillatorFrequency(uint32_t freq);

  void setPWM(uint8_t num, uint16_t on, uint16_t off);

  void writeMicroseconds(uint8_t num, uint16_t Microseconds);

  void write_to_deg_Servo (uint8_t servo, uint8_t deg);

  void write_to_deg_Trottle(uint8_t deg);

  uint16_t map (uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);

private:

uint32_t _oscillator_freq;

uint8_t read8(uint8_t addr);

void write8(uint8_t addr, uint8_t d);

};

#endif // PCA9685_H
