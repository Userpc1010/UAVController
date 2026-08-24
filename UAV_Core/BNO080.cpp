#include "BNO080.h"
#include <limits.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include "utilitiy/tools.h"
#include <QDebug>

#define I2C_SLAVE	0x0703

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
bool BNO080::begin()
{         
    ioctl(fd, I2C_SLAVE, BNO080_DEFAULT_ADDRESS);

    if(get_shtp_errors() > 0) bno_reset();

    //Check communication with device
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0;							  //Reserved

    //Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    if (receivePacket())
    {
    if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
    {

    enableARVRStabilizedRotationVector(10);

    enableLinearAccelerometer(10);

    start_bno080();

    if(receivePacket()) return (true);

    }
    }

    return (false); //Something went wrong
}

void BNO080::start_bno080() //хз почему но датчик сразу не выходит на режим
{
    for (;bno080_counter_1 <= 100; bno080_counter_1++)
    {
           bno080_counter_2++;

        if (bno080_counter_2 >= 10)
        {
            bno080_counter_2 = 0;

            dataAvailable();
        }

        usleep(10000);
    }
}


int BNO080::get_shtp_errors() {
   // Test code: Below line simulates an SHTP error for incomplete
   // header data. SH2 will add the code 2 entry to the error list
   // sensor reset clears the error list.
   // char data[3] = { 2, 3, 4}; write(i2cfd, data, 3);

   /* --------------------------------------------------------- *
    * SHTP get error list from sensor                           *
    * --------------------------------------------------------- */
   shtpData[0] = 0x01;                // CMD 0x01 gets error list
   sendPacket(CHANNEL_COMMAND, 1);    // Write 1 byte to chan CMD
   usleep(100);                  // 100 msecs before next I2C

   /* --------------------------------------------------------- *
    * Get the SHTP error list, after reset it should be clean   *
    *  RX   5 bytes HEAD 05 80 00 03 CARGO 01 ST [0]            *
    * --------------------------------------------------------- */
   // Wait for answer packet
   int datalen;
   while ((datalen = receivePacket()) != 0) {
      if(shtpHeader[2] == CHANNEL_COMMAND && shtpData[0] == 0x01) break;
      usleep(100);              // Delay 100 microsecs before next I2C
   }

   if(shtpHeader[2] != CHANNEL_COMMAND || shtpData[0] != 1) {
      qDebug()<<"Error: can't get SHTP error list";
      return(-1);
   }

   /* --------------------------------------------------------- *
    * Calculate the error counter                               *
    * --------------------------------------------------------- */
   int errcount = datalen - 1; // datalen minus 1 report byte

   return(errcount);
}
/* ------------------------------------------------------------ *
 * bno_reset() resets the sensor.                               *
 * ------------------------------------------------------------ */
void BNO080::bno_reset() {   
   /* --------------------------------------------------------- *
    * Send the "reset" command and watch the response packets   *
    * --------------------------------------------------------- */
   shtpData[0] = 1;                   // CMD1 = reset
   sendPacket(CHANNEL_EXECUTABLE, 1); // Write 1 byte to chan EXE
   usleep(200000);                    // 700 millisecs for reboot

   /* --------------------------------------------------------- *
    * After reset,  we get 3 packets:                           *
    * 1. packet: unsolicited advertising packet (chan 0)        *
    * --------------------------------------------------------- */
   receivePacket();
   if(shtpHeader[2] != CHANNEL_COMMAND || shtpHeader[3] != 1) {
      qDebug()<<"Error: can't get SHTP advertising.";
   }
   usleep(100);            // Delay 100 microsecs before next I2C

   /* --------------------------------------------------------- *
    * 2. packet: "reset complete" (chan 1, response code 1)     *
    * --------------------------------------------------------- */
   receivePacket();
   if(shtpHeader[2] != CHANNEL_EXECUTABLE || shtpHeader[3] != 1
      || shtpData[0] != 1) {
      qDebug()<<"Error: can't get 'reset complete' status.";
   }
   usleep(100);            // Delay 100 microsecs before next I2C
   /* --------------------------------------------------------- *
    * 3. packet: SH-2 sensor hub SW init (chan 2)               *
    * --------------------------------------------------------- */
   receivePacket();
   if(shtpHeader[2] != CHANNEL_CONTROL || shtpHeader[3] != 1) {
      qDebug()<<"Error: can't get SH2 initialization.";
   }
}

//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
    ioctl(fd, I2C_SLAVE, BNO080_DEFAULT_ADDRESS);

    if (receivePacket())
    {

        //Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
            return (true);
        }
        else if (shtpHeader[2] == CHANNEL_CONTROL)
        {
            parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
            return (true);
        }
    else if(shtpHeader[2] == CHANNEL_GYRO)
    {
      parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
      return (true);
    }
    }

    return (false);
}

void BNO080::parseCommandReport(void)
{
    if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
        }
    }
}

void BNO080::parseInputReport(void)
{
    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
    //Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; //Remove the header bytes from the data count

    timeStamp = ((uint32_t)shtpData[4] << (8 * 3)) | ((uint32_t)shtpData[3] << (8 * 2)) | ((uint32_t)shtpData[2] << (8 * 1)) | ((uint32_t)shtpData[1] << (8 * 0));

    // The gyro-integrated input reports are sent via the special gyro channel and do no include the usual ID, sequence, and status fields
    if(shtpHeader[2] == CHANNEL_GYRO) {
        QuatI = rotationVector_Q1_to_float((uint16_t)shtpData[1] << 8 | shtpData[0]);
        QuatJ = rotationVector_Q1_to_float((uint16_t)shtpData[3] << 8 | shtpData[2]);
        QuatK = rotationVector_Q1_to_float((uint16_t)shtpData[5] << 8 | shtpData[4]);
        QuatReal = rotationVector_Q1_to_float((uint16_t)shtpData[7] << 8 | shtpData[6]);
        FastGyroX = (uint16_t)shtpData[9] << 8 | shtpData[8];
        FastGyroY = (uint16_t)shtpData[11] << 8 | shtpData[10];
        FastGyroZ = (uint16_t)shtpData[13] << 8 | shtpData[12];

        return;
    }

    uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
    int16_t data1 = (int16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    int16_t data2 = (int16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    int16_t data3 = (int16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    int16_t data4 = (int16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
    int16_t data5 = (int16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12]; //We would need to change this to uin32_t to capture time stamp value on Raw Accel/Gyro/Mag reports

    //Store these generic values to their proper global variable
    if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
    {
        accelAccuracy = status;
        AccelX = accelerometer_Q1_to_float(data1);
        AccelY = accelerometer_Q1_to_float(data2);
        AccelZ = accelerometer_Q1_to_float(data3);
    }
    else if (shtpData[5] == SENSOR_REPORTID_LINEAR_ACCELERATION)
    {
        accelLinAccuracy = status;
        LinAccelX = linear_acceleration_Q1_to_float(data1);
        LinAccelY = linear_acceleration_Q1_to_float(data2);
        LinAccelZ = linear_acceleration_Q1_to_float(data3);
    }
    else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
    {
        gyroAccuracy = status;
        GyroX = gyro_Q1_to_float(data1);
        GyroY = gyro_Q1_to_float(data2);
        GyroZ = gyro_Q1_to_float(data3);
    }
    else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
    {
        magAccuracy = status;
        MagX = magnetometer_Q1_to_float(data1);
        MagY = magnetometer_Q1_to_float(data2);
        MagZ = magnetometer_Q1_to_float(data3);
    }
    else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR ||
        shtpData[5] == SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR)
    {
        quatAccuracy = status;
        QuatI = rotationVector_Q1_to_float(data1);
        QuatJ = rotationVector_Q1_to_float(data2);
        QuatK = rotationVector_Q1_to_float(data3);
        QuatReal = rotationVector_Q1_to_float(data4);

        //Only available on rotation vector and ar/vr stabilized rotation vector,
        // not game rot vector and not ar/vr stabilized rotation vector
        QuatRadianAccuracy = rotationVectorAccuracy_Q1_to_float(data5);

        float norm = sqrtf(QuatReal * QuatReal + QuatI * QuatI + QuatJ * QuatJ + QuatK * QuatK);
        if (norm != 0.0f) {
        norm = 1.0f / norm;
        QuatReal = QuatReal * norm;
        QuatI = QuatI * norm;
        QuatJ = QuatJ * norm;
        QuatK = QuatK * norm;

        getRPY(); //Quat to Eualer
    }}
    else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
    {
        stepCount = data3; //Bytes 8/9
    }
    else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
    {
        stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
    }
    else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
    {
        activityClassifier = shtpData[5 + 5]; //Most likely state

        //Load activity classification confidences into the array
        for (uint8_t x = 0; x < 9; x++)					   //Hardcoded to max of 9. TODO - bring in array size
            _activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_ACCELEROMETER)
    {
        memsRawAccelX = data1;
        memsRawAccelY = data2;
        memsRawAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_GYROSCOPE)
    {
        memsRawGyroX = data1;
        memsRawGyroY = data2;
        memsRawGyroZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_RAW_MAGNETOMETER)
    {
        memsRawMagX = data1;
        memsRawMagY = data2;
        memsRawMagZ = data3;
    }
    else if (shtpData[5] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[5 + 2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibrationStatus = shtpData[5 + 5]; //R0 - Status (0 = success, non-zero = fail)
        }
    }
}

void BNO080::getRPY()
{
    float q0q0 = QuatReal * QuatReal;

    //qDebug()<<"Quat: "<<QuatReal<<" "<<QuatI<<" "<<QuatJ<<" "<<QuatK;

    //x-y-z
    CBn[0] = 2.0f * (q0q0 + QuatI * QuatI) - 1.0f;
    CBn[1] = 2.0f * (QuatI * QuatJ + QuatReal * QuatK);
    CBn[2] = 2.0f * (QuatI * QuatK - QuatReal * QuatJ);
    CBn[3] = 2.0f * (QuatJ * QuatK + QuatReal * QuatI);
    CBn[4] = 2.0f * (q0q0 + QuatK * QuatK) - 1.0f;

    //roll
    roll = atan2f(-CBn[2], CBn[4]);

    roll += Roll_offset;

    if (roll > Quat_PI ) roll -= Quat_TWOPI;
    if (roll < -Quat_PI) roll += Quat_TWOPI;

    //pitch
    pitch = atan2f(-CBn[3], CBn[4]);

    pitch += Pitch_offset;

    if (pitch >  Quat_PI) pitch -= Quat_TWOPI;
    if (pitch < -Quat_PI) pitch += Quat_TWOPI;

    //yaw
    yaw = atan2f(CBn[1], -CBn[0]);

    yaw += Yaw_offset;

    if (yaw >  Quat_PI) yaw -= Quat_TWOPI;
    if (yaw < -Quat_PI) yaw += Quat_TWOPI;

    roll = Quat_TODEG(roll);
    pitch = Quat_TODEG(pitch);
    yaw = Quat_TODEG(yaw);

    GravX = CBn[2];
    GravY = CBn[3];
    GravZ = CBn[4];

   // qDebug()<<"Grav: "<<GravX<<" : "<<GravY<<" : "<<GravZ;
}


float BNO080::getRoll()
{
        return roll;
}

// Return the pitch (rotation around the y-axis) in Radians
float BNO080::getPitch()
{
        return pitch;
}

// Return the yaw / heading (rotation around the z-axis) in Radians
float BNO080::getYaw()
{
    return yaw;
}

float BNO080::getGravX()
{
  return GravX;
}

float BNO080::getGravY()
{
 return GravY;
}

float BNO080::getGravZ()
{
    return GravZ;
}

void BNO080::Offset(float initialRoll, float initialPitch, float initialYaw)
{
  Pitch_offset = initialPitch; Roll_offset = initialRoll; Yaw_offset = initialYaw;
}

float BNO080::getQuatReal()
{
    return QuatReal;
}

float BNO080::getQuatI()
{
  return QuatI;
}

float BNO080::getQuatJ()
{
  return QuatJ;
}

float BNO080::getQuatK()
{
 return QuatK;
}
float BNO080::getQuatRadianAccuracy()
{
 return QuatRadianAccuracy;
}

//Return the acceleration component
uint8_t BNO080::getQuatAccuracy()
{
    return (quatAccuracy);
}

//Return the acceleration component
float BNO080::getAccelX()
{
    return AccelX;
}

//Return the acceleration component
float BNO080::getAccelY()
{ 
    return AccelY;
}

//Return the acceleration component
float BNO080::getAccelZ()
{
    return AccelZ;
}

//Return the acceleration component
uint8_t BNO080::getAccelAccuracy()
{
    return (accelAccuracy);
}

// linear acceleration, i.e. minus gravity

//Return the acceleration component
float BNO080::getLinAccelX()
{
    return LinAccelX; // qToFloat(rawLinAccelX, linear_accelerometer_Q1);
}

//Return the acceleration component
float BNO080::getLinAccelY()
{
    return LinAccelY; //qToFloat(rawLinAccelY, linear_accelerometer_Q1);
}

//Return the acceleration component
float BNO080::getLinAccelZ()
{ 
    return LinAccelZ; //qToFloat(rawLinAccelZ, linear_accelerometer_Q1);
}

//Return the acceleration component
uint8_t BNO080::getLinAccelAccuracy()
{
    return (accelLinAccuracy);
}

//Return the gyro component
float BNO080::getGyroX()
{ 
    return GyroX;
}

//Return the gyro component
float BNO080::getGyroY()
{
    return GyroY;
}

//Return the gyro component
float BNO080::getGyroZ()
{
    return GyroZ;
}

//Return the gyro component
uint8_t BNO080::getGyroAccuracy()
{
    return (gyroAccuracy);
}

//Return the magnetometer component
float BNO080::getMagX()
{

    return MagX;
}

//Return the magnetometer component
float BNO080::getMagY()
{ 
    return MagY;
}

//Return the magnetometer component
float BNO080::getMagZ()
{
    return MagZ;
}

//Return the mag component
uint8_t BNO080::getMagAccuracy()
{
    return (magAccuracy);
}

// Return the high refresh rate gyro component
float BNO080::getFastGyroX()
{
    return FastGyroX;
}

// Return the high refresh rate gyro component
float BNO080::getFastGyroY()
{
    return FastGyroY;
}

// Return the high refresh rate gyro component
float BNO080::getFastGyroZ()
{
    return FastGyroZ;
}

//Return the step count
uint16_t BNO080::getStepCount()
{
    return (stepCount);
}

//Return the stability classifier
uint8_t BNO080::getStabilityClassifier()
{
    return (stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO080::getActivityClassifier()
{
    return (activityClassifier);
}

//Return the time stamp
uint32_t BNO080::getTimeStamp()
{
    return (timeStamp);
}

//Return raw mems value for the accel
int16_t BNO080::getRawAccelX()
{
    return (memsRawAccelX);
}
//Return raw mems value for the accel
int16_t BNO080::getRawAccelY()
{
    return (memsRawAccelY);
}
//Return raw mems value for the accel
int16_t BNO080::getRawAccelZ()
{
    return (memsRawAccelZ);
}

//Return raw mems value for the gyro
int16_t BNO080::getRawGyroX()
{
    return (memsRawGyroX);
}
int16_t BNO080::getRawGyroY()
{
    return (memsRawGyroY);
}
int16_t BNO080::getRawGyroZ()
{
    return (memsRawGyroZ);
}

//Return raw mems value for the mag
int16_t BNO080::getRawMagX()
{
    return (memsRawMagX);
}
int16_t BNO080::getRawMagY()
{
    return (memsRawMagY);
}
int16_t BNO080::getRawMagZ()
{
    return (memsRawMagZ);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::rotationVector_Q1_to_float(int16_t fixedPointValue)
{
  return ((float)fixedPointValue) * 0.00006103515625f;
}

float BNO080::rotationVectorAccuracy_Q1_to_float(int16_t fixedPointValue)
{
  return ((float)fixedPointValue) * 0.000244140625f;
}

float BNO080::linear_acceleration_Q1_to_float(int16_t fixedPointValue)
{
  return ((float)fixedPointValue) * 0.00390625f;
}

float BNO080::accelerometer_Q1_to_float(int16_t fixedPointValue)
{
   return ((float)fixedPointValue) * 0.00390625f;
}

float BNO080::gyro_Q1_to_float(int16_t fixedPointValue)
{
    return ((float)fixedPointValue) * 0.001953125f;
}

float BNO080::magnetometer_Q1_to_float(int16_t fixedPointValue)
{
    return ((float)fixedPointValue) * 0.0625f;
}

float BNO080::angular_velocity_Q1_to_float(int16_t fixedPointValue)
{
    return ((float)fixedPointValue) * 0.0009765625f;
}

//Sends the packet to enable the rotation vector
void BNO080::enableRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO080::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
void BNO080::enableGameRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
void BNO080::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableAccelerometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO080::enableMagnetometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the high refresh-rate gyro-integrated rotation vector
void BNO080::enableGyroIntegratedRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080::enableStepCounter(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO080::enableStabilityClassifier(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawAccelerometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawGyro(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
void BNO080::enableRawMagnetometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);
}

//Sends the packet to enable the various activity classifiers
void BNO080::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9])
{
    _activityConfidences = activityConfidences; //Store pointer to array

    setFeatureCommand(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

//Sends the commands to begin calibration of the accelerometer
void BNO080::calibrateAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO080::calibrateGyro()
{
    sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO080::calibrateMagnetometer()
{
    sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO080::calibratePlanarAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO080::calibrateAll()
{
    sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080::endCalibration()
{
    sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}

//See page 51 of reference manual - ME Calibration Response
//Byte 5 is parsed during the readPacket and stored in calibrationStatus
bool BNO080::calibrationComplete()
{
    if (calibrationStatus == 0)
        return (true);
    return (false);
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
    setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    long microsBetweenReports = (long)timeBetweenReports * 1000L;

    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND;	 //Set feature command. Reference page 55
    shtpData[1] = reportID;							   //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0;								   //Feature flags
    shtpData[3] = 0;								   //Change sensitivity (LSB)
    shtpData[4] = 0;								   //Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF;  //Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF;  //Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
    shtpData[9] = 0;								   //Batch Interval (LSB)
    shtpData[10] = 0;								   //Batch Interval
    shtpData[11] = 0;								   //Batch Interval
    shtpData[12] = 0;								   //Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF;	   //Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF;	   //Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF;	  //Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF;	  //Sensor-specific config (MSB)

    //Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
    shtpData[1] = commandSequenceNumber++;	 //Increments automatically each function call
    shtpData[2] = command;					   //Command

    //Transmit packet on channel 2, 12 bytes
    sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO080::sendCalibrateCommand(uint8_t thingToCalibrate)
{
    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    if (thingToCalibrate == CALIBRATE_ACCEL)
        shtpData[3] = 1;
    else if (thingToCalibrate == CALIBRATE_GYRO)
        shtpData[4] = 1;
    else if (thingToCalibrate == CALIBRATE_MAG)
        shtpData[5] = 1;
    else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL)
        shtpData[7] = 1;
    else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
    {
        shtpData[3] = 1;
        shtpData[4] = 1;
        shtpData[5] = 1;
    }
    //else if (thingToCalibrate == CALIBRATE_STOP); //Do nothing, bytes are set to zero

    //Make the internal calStatus variable non-zero (operation failed) so that user can test while we wait
    calibrationStatus = 1;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//Request ME Calibration Status from BNO080
//See page 51 of reference manual
void BNO080::requestCalibrationStatus()
{
    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    shtpData[6] = 0x01; //P3 - 0x01 - Subcommand: Get ME Calibration

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO080::saveCalibration()
{
    for (uint8_t x = 3; x < 12; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_DCD); //Save DCD command
}

/* ------------------------------------------------------------ *
 * Check to see if there is any new data available. Read the    *
 * contents of the incoming packet into the shtpData array.     *
 * Returns cargo data size in bytes, or 0 for errors.           *
 * ------------------------------------------------------------ */
bool BNO080::receivePacket(void) {

   int rbytes;                // Received bytes uffer
   int err;                   // error code buffer

   // 1st Read to get the 4-byte SHTP header with the cargo size
   rbytes = read(fd, shtpHeader, 4);

   err = errno;

   if(rbytes != 4) {
      qDebug()<<"Error: I2C SHTP header read failure: "<<rbytes;
      qDebug()<<"Error: "<<strerror(err);
      return(false);
   }

   // Calculate the number of data bytes to be received
   short packetlen = ((short) shtpHeader[1] << 8 | shtpHeader[0]);
   packetlen &= ~(1 << 15);       // Clear the MSbit.
   short datalen = packetlen - 4; // Remove the 4 header byte

   // 2nd Read the remaining cargo data
   if(datalen == 0) {             // Cargo data is to be received
      return(false);
   }

   uint8_t data[packetlen];    // Buffer for cargo data

   rbytes = read(fd, data, packetlen);
   err = errno;

   if(rbytes < packetlen) {
      qDebug()<<"Error: I2C SHTP data read failure: got "<<rbytes<<'/'<<datalen<<" bytes.";
      qDebug()<<"Error: "<<strerror(err);
      return(false);
   }

   // update the sequence counter for the channel
   sequenceNumber[data[2]] = data[3];

   // clear global data array, and write data buffer to it
   memset(shtpData, 0, sizeof shtpData);

   for (int i = 0; i < packetlen; i++) {
      if(i < 4) shtpHeader[i] = data[i]; // Store data into the shtpData array
      if(i > 3) shtpData[i-4] = data[i]; // Store data into the shtpData array
   }

   return true;
}

/* ------------------------------------------------------------ *
 * Given the data packet, send the header and then the data.    *
 * ------------------------------------------------------------ */
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength) {

   uint8_t packetlen = dataLength + 4;        // Add four bytes for the header
   uint8_t data[packetlen];                  // local buffer for I2C write data

   data[0] = packetlen & 0xFF;               // packet length LSB
   data[1] = packetlen >> 8;                 // packet length MSB
   data[2] = channelNumber;                  // channel number
   data[3] = sequenceNumber[channelNumber]++; // packet sequence num, increment seq for each packet

   // Copy the payload data from shtpData to the I2C data buffer
   for (short i = 0 ; i < dataLength; i++) {
      data[4+i] = shtpData[i];
   }

   if(write(fd, &data, packetlen) != packetlen) {
      qDebug()<<"Error: I2C write failure "<<packetlen<<" data\n";
      return (false);
   }

   return (true);
}
