#include "pca9685.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/fs.h>
#include <QDebug>

#define I2C_SLAVE	0x0703	//Регистр I2C мастер

// REGISTER ADDRESSES
#define PCA9685_MODE1 0x00      /**< Mode Register 1 */
#define PCA9685_MODE2 0x01      /**< Mode Register 2 */
#define PCA9685_SUBADR1 0x02    /**< I2C-bus subaddress 1 */
#define PCA9685_SUBADR2 0x03    /**< I2C-bus subaddress 2 */
#define PCA9685_SUBADR3 0x04    /**< I2C-bus subaddress 3 */
#define PCA9685_ALLCALLADR 0x05 /**< LED All Call I2C-bus address */
#define PCA9685_LED0_ON_L 0x06  /**< LED0 on tick, low byte*/
#define PCA9685_LED0_ON_H 0x07  /**< LED0 on tick, high byte*/
#define PCA9685_LED0_OFF_L 0x08 /**< LED0 off tick, low byte */
#define PCA9685_LED0_OFF_H 0x09 /**< LED0 off tick, high byte */
// etc all 16:  LED15_OFF_H 0x45
#define PCA9685_ALLLED_ON_L 0xFA  /**< load all the LEDn_ON registers, low */
#define PCA9685_ALLLED_ON_H 0xFB  /**< load all the LEDn_ON registers, high */
#define PCA9685_ALLLED_OFF_L 0xFC /**< load all the LEDn_OFF registers, low */
#define PCA9685_ALLLED_OFF_H 0xFD /**< load all the LEDn_OFF registers,high */
#define PCA9685_PRESCALE 0xFE     /**< Prescaler for PWM output frequency */
#define PCA9685_TESTMODE 0xFF     /**< defines the test mode to be entered */

// MODE1 bits
#define MODE1_ALLCAL 0x01  /**< respond to LED All Call I2C-bus address */
#define MODE1_SUB3 0x02    /**< respond to I2C-bus subaddress 3 */
#define MODE1_SUB2 0x04    /**< respond to I2C-bus subaddress 2 */
#define MODE1_SUB1 0x08    /**< respond to I2C-bus subaddress 1 */
#define MODE1_SLEEP 0x10   /**< Low power mode. Oscillator off */
#define MODE1_AI 0x20      /**< Auto-Increment enabled */
#define MODE1_EXTCLK 0x40  /**< Use EXTCLK pin clock */
#define MODE1_RESTART 0x80 /**< Restart enabled */
// MODE2 bits
#define MODE2_OUTNE_0 0x01 /**< Active LOW output enable input */
#define MODE2_OUTNE_1                                                          \
  0x02 /**< Active LOW output enable input - high impedience */
#define MODE2_OUTDRV 0x04 /**< totem pole structure vs open-drain */
#define MODE2_OCH 0x08    /**< Outputs change on ACK vs STOP */
#define MODE2_INVRT 0x10  /**< Output logic state inverted */

#define PCA9685_I2C_ADDRESS 0x40      /**< Default PCA9685 I2C Slave Address */
#define FREQUENCY_OSCILLATOR 25000000 /**< Int. osc. frequency in datasheet */

#define PCA9685_PRESCALE_MIN 3   /**< minimum prescale value */
#define PCA9685_PRESCALE_MAX 255 /**< maximum prescale value */

PCA9685::PCA9685()
{
}

void PCA9685::begin()
{
 start();

 reset();

 // set the default internal frequency
 setOscillatorFrequency(27000000);

 // set a default frequency
 setPWMFreq(90);
}

void PCA9685::reset() {
  write8(PCA9685_MODE1, MODE1_RESTART);
  usleep(20000);
}


void PCA9685::setPWMFreq(float freq) {

  // Range output modulation frequency is dependant on oscillator
  if (freq < 1)
    freq = 1;
  if (freq > 3500)
    freq = 3500; // Datasheet limit is 3052=50MHz/(4*4096)

  float prescaleval = ((_oscillator_freq / (freq * 4096.0)) + 0.5) - 1;
  if (prescaleval < PCA9685_PRESCALE_MIN)
    prescaleval = PCA9685_PRESCALE_MIN;
  if (prescaleval > PCA9685_PRESCALE_MAX)
    prescaleval = PCA9685_PRESCALE_MAX;
  uint8_t prescale = (uint8_t)prescaleval;


  uint8_t oldmode = read8(PCA9685_MODE1);
  uint8_t newmode = (oldmode & ~MODE1_RESTART) | MODE1_SLEEP; // sleep
  write8(PCA9685_MODE1, newmode);                             // go to sleep
  write8(PCA9685_PRESCALE, prescale); // set the prescaler
  write8(PCA9685_MODE1, oldmode);
  usleep(5000);
  // This sets the MODE1 register to turn on auto increment.
  write8(PCA9685_MODE1, oldmode | MODE1_RESTART | MODE1_AI);
}


void PCA9685::setOscillatorFrequency(uint32_t freq) {
  _oscillator_freq = freq;
}


void PCA9685::writeMicroseconds(uint8_t num, uint16_t Microseconds)
{

double pulse = Microseconds;
double pulselength;
pulselength = 1000000; // 1,000,000 us per second

// Read prescale
uint16_t prescale = readPrescale();

// Calculate the pulse for PWM based on Equation 1 from the datasheet section
// 7.3.5
prescale += 1;
pulselength *= prescale;
pulselength /= _oscillator_freq;

pulse /= pulselength;

setPWM(num, 0, pulse);
}

void PCA9685::write_to_deg_Servo(uint8_t servo, uint8_t deg)
{
  start();  writeMicroseconds(servo, map (deg, 0, 180, 500, 2400 ));
}

void PCA9685::write_to_deg_Trottle(uint8_t deg)
{
  start();  writeMicroseconds(0, map (deg, 0, 255, 800, 2200 ));
}

void PCA9685::setPWM(uint8_t num, uint16_t on, uint16_t off) {

  uint8_t buf[5] = { PCA9685_LED0_ON_L + 4 * num, on, on >> 8, off, off >> 8 };

  if(write(fd, &buf, 5) != 5) {

      qDebug()<<"Ошибка записи I2C в регистр: "<<PCA9685_LED0_ON_L + 4 * num << " Метод setPWM PCA9685";
   }
}

uint8_t PCA9685::readPrescale(void) {

  return read8(PCA9685_PRESCALE);
}

uint16_t PCA9685::map(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/******************* Low level I2C interface */

void PCA9685::start()
{
  ioctl(fd, I2C_SLAVE, PCA9685_I2C_ADDRESS);
}

uint8_t PCA9685::read8(uint8_t addr) {


 if(write(fd, &addr, 1) != 1) qDebug()<<"Ошибка записи I2C в регистр: "<<addr<< " Метод read8_write PCA9685";

 uint8_t value;

 if(read(fd, &value, 1) != 1) {

  qDebug()<<"Ошибка чтения из регистра I2C: "<<addr << "Метод read8_read PCA9685";
 }

  return value;
}

void PCA9685::write8(uint8_t addr, uint8_t d) {

  uint8_t buf[2] = { addr, d };

  if(write(fd, &buf, 2) != 2) {

      qDebug()<<"Ошибка записи I2C в регистр: "<<addr << " Метод write8 PCA9685";
   }
}

