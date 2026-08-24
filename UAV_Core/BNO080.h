#ifndef BNO080_H
#define BNO080_H

#include <inttypes.h>

//The default I2C address for the BNO080 on the SparkX breakout is 0x4B. 0x4A is also possible.
#define BNO080_DEFAULT_ADDRESS 0x4B

//Platform specific configurations

//Registers
const uint8_t CHANNEL_COMMAND = 0;
const uint8_t CHANNEL_EXECUTABLE = 1;
const uint8_t CHANNEL_CONTROL = 2;
const uint8_t CHANNEL_REPORTS = 3;
const uint8_t CHANNEL_WAKE_REPORTS = 4;
const uint8_t CHANNEL_GYRO = 5;

//All the ways we can configure or talk to the BNO080, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER 0x01
#define SENSOR_REPORTID_GYROSCOPE 0x02
#define SENSOR_REPORTID_MAGNETIC_FIELD 0x03
#define SENSOR_REPORTID_LINEAR_ACCELERATION 0x04
#define SENSOR_REPORTID_ROTATION_VECTOR 0x05
#define SENSOR_REPORTID_GRAVITY 0x06
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR 0x2A
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER 0x11
#define SENSOR_REPORTID_STABILITY_CLASSIFIER 0x13
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER 0x1E
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

//Record IDs from figure 29, page 29 reference manual
//These are used to read the metadata for each sensor type
#define FRS_RECORDID_ACCELEROMETER 0xE302
#define FRS_RECORDID_GYROSCOPE_CALIBRATED 0xE306
#define FRS_RECORDID_MAGNETIC_FIELD_CALIBRATED 0xE309
#define FRS_RECORDID_ROTATION_VECTOR 0xE30B

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define MAX_PACKET_SIZE 128 //Packets can be up to 32k but we don't have that much RAM.
#define MAX_METADATA_SIZE 9 //This is in words. There can be many but we mostly only care about the first 9 (Qs, range, etc)

//#define rotationVector_Q1(x) ((x) * 0.00006103515625f)
//#define rotationVectorAccuracy_Q1(x) ((x) * 0.000244140625f) //Heading accuracy estimate in radians. The Q point is 12.
//#define accelerometer_Q1(x) ((x) * 0.00390625f)
//#define linear_accelerometer_Q1(x) ((x) * 0.00390625f)
//#define gyro_Q1(x) ((x) * 0.001953125f)
//#define magnetometer_Q1(x) ((x) * 0.0625f)
//#define angular_velocity_Q1(x) ((x) * 0.0009765625f)

#define Quat_HALFPI 1.5707963267948966192313216916398f
#define Quat_PI 3.1415926535897932384626433832795f
#define Quat_TWOPI 6.283185307179586476925286766559f
#define Quat_TODEG(x) ((x) * 57.2957796f)
#define Quat_TORAD(x) ((x) *  0.0174533f)

class BNO080
{
public:

    bool begin(); //By default use the default I2C addres, and use Wire port, and don't declare an INT pin

    uint8_t resetReason(); //Query the IMU for the reason it last reset
    bool receivePacket(void);
    bool getData(uint16_t bytesRemaining); //Given a number of bytes, send the requests in I2C_BUFFER_LENGTH chunks
    bool sendPacket(uint8_t channelNumber, uint8_t dataLength);
    int get_shtp_errors(void);
    void bno_reset(void);

    void enableRotationVector(uint16_t timeBetweenReports);
    void enableGameRotationVector(uint16_t timeBetweenReports);
    void enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
    void enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
    void enableAccelerometer(uint16_t timeBetweenReports);
    void enableLinearAccelerometer(uint16_t timeBetweenReports);
    void enableGyro(uint16_t timeBetweenReports);
    void enableMagnetometer(uint16_t timeBetweenReports);
    void enableStepCounter(uint16_t timeBetweenReports);
    void enableStabilityClassifier(uint16_t timeBetweenReports);
    void enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9]);
    void enableRawAccelerometer(uint16_t timeBetweenReports);
    void enableRawGyro(uint16_t timeBetweenReports);
    void enableRawMagnetometer(uint16_t timeBetweenReports);
    void enableGyroIntegratedRotationVector(uint16_t timeBetweenReports);

    bool dataAvailable(void);
    void parseInputReport(void);   //Parse sensor readings out of report
    void parseCommandReport(void); //Parse command responses out of report

    float getQuatI();
    float getQuatJ();
    float getQuatK();
    float getQuatReal();
    float getQuatRadianAccuracy();
    uint8_t getQuatAccuracy();

    float getAccelX();
    float getAccelY();
    float getAccelZ();
    uint8_t getAccelAccuracy();

    float getLinAccelX();
    float getLinAccelY();
    float getLinAccelZ();
    uint8_t getLinAccelAccuracy();

    float getGyroX();
    float getGyroY();
    float getGyroZ();
    uint8_t getGyroAccuracy();

    float getFastGyroX();
    float getFastGyroY();
    float getFastGyroZ();

    float getMagX();
    float getMagY();
    float getMagZ();
    uint8_t getMagAccuracy();

    void calibrateAccelerometer();
    void calibrateGyro();
    void calibrateMagnetometer();
    void calibratePlanarAccelerometer();
    void calibrateAll();
    void endCalibration();
    void saveCalibration();
    void requestCalibrationStatus(); //Sends command to get status
    bool calibrationComplete();   //Checks ME Cal response for byte 5, R0 - Status

    uint32_t getTimeStamp();
    uint16_t getStepCount();
    uint8_t getStabilityClassifier();
    uint8_t getActivityClassifier();

    int16_t getRawAccelX();
    int16_t getRawAccelY();
    int16_t getRawAccelZ();

    int16_t getRawGyroX();
    int16_t getRawGyroY();
    int16_t getRawGyroZ();

    int16_t getRawMagX();
    int16_t getRawMagY();
    int16_t getRawMagZ();

    //В радианах
    float  Pitch_offset = 0.0f;
    float  Roll_offset = 0.0f;
    float  Yaw_offset = 0.0f;

    uint8_t fd = 0;

    float getRoll();
    float getPitch();
    float getYaw();

    float getGravX ();
    float getGravY ();
    float getGravZ ();

    void Offset(float initialRoll, float initialPitch, float initialYaw);

    void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports);
    void setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig);
    void sendCommand(uint8_t command);
    void sendCalibrateCommand(uint8_t thingToCalibrate);

    //Global Variables
    uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
    uint8_t shtpData[MAX_PACKET_SIZE];
    uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
    uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
    uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

private:

    //Given a Q value, converts fixed point floating to regular floating point number
    float rotationVector_Q1_to_float (int16_t fixedPointValue);
    float rotationVectorAccuracy_Q1_to_float (int16_t fixedPointValue);
    float linear_acceleration_Q1_to_float (int16_t fixedPointValue);
    float accelerometer_Q1_to_float (int16_t fixedPointValue);
    float gyro_Q1_to_float (int16_t fixedPointValue);
    float magnetometer_Q1_to_float (int16_t fixedPointValue);
    float angular_velocity_Q1_to_float (int16_t fixedPointValue);

    void start_bno080 ();

    void getRPY();

    //Variables
    float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;

    float CBn[5];

    uint8_t bno080_counter_1 = 0;
    uint8_t bno080_counter_2 = 0;

    //These are the raw sensor values (without Q applied) pulled from the user requested Input Report
    float AccelX, AccelY, AccelZ, accelAccuracy;
    float LinAccelX, LinAccelY, LinAccelZ, accelLinAccuracy;
    float GyroX, GyroY, GyroZ, gyroAccuracy;
    float GravX, GravY, GravZ;
    float MagX, MagY, MagZ, magAccuracy;
    float QuatI, QuatJ, QuatK, QuatReal, QuatRadianAccuracy, quatAccuracy;
    float FastGyroX, FastGyroY, FastGyroZ;
    uint16_t stepCount;
    uint32_t timeStamp;
    uint8_t stabilityClassifier;
    uint8_t activityClassifier;
    uint8_t *_activityConfidences;						  //Array that store the confidences of the 9 possible activities
    uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
    uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
    uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
    uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

};
#endif // BNO080_H
