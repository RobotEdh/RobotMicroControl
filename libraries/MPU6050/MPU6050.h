#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <Wire.h>


#define LED 13    // LED 

#define ADDRESS_DEFAULT 0X69 //AD0 logic level set to high
#define MPU6050_ID 0x68

#define WIRE_TRANSMIT_SUCESS          0x00 // Wire.endTransmission()- 0:success
#define WIRE_ERROR_TRANSMIT_TOOLONG   0x01 // Wire.endTransmission()- 1:data too long to fit in transmit buffer
#define WIRE_ERROR_TRANSMIT_ADR_NACK  0x02 // Wire.endTransmission()- 2:received NACK on transmit of address
#define WIRE_ERROR_TRANSMIT_DATA_NACK 0x03 // Wire.endTransmission()- 3:received NACK on transmit of data
#define WIRE_TRANSMIT_ERROR_OTHER     0x04 // Wire.endTransmission()- 4:other error
#define WIRE_REQUEST_ERROR            0x80 // Wire.requestFrom()- the number of bytes returned from the slave device != the number of bytes to request

const double FE_ACCEL = 4096.0; // scale factor accelerometer for AFS_SEL = 2 
const double FE_GYRO = 65.5;    // scale factor gyro  for FS_SEL = 1



class MPU6050Class
{
  public:
    // register addresses from API MPU6050_device.h (ordered as listed there)
    enum regAddr
    {
      SMPRT_DIV                                   = 0x19,
      CONFIG                                      = 0x1A,
      GYRO_CONFIG                                 = 0x1B,
      ACCEL_CONFIG                                = 0x1C,
      ACCEL_XOUT                                  = 0x3B,
      ACCEL_YOUT                                  = 0x3D,
      ACCEL_ZOUT                                  = 0x3F,
      TEMP_OUT                                    = 0x41,
      GYRO_XOUT                                   = 0x43,
      GYRO_YOUT                                   = 0x45,
      GYRO_ZOUT                                   = 0x47,            
      PWR_MGMT_1                                  = 0x6B,
      WHO_AM_I                                    = 0x75
    };

    MPU6050Class();
    
     void MPU6050_writeReg(uint8_t reg, uint8_t value);
    uint8_t  MPU6050_readReg(uint8_t reg);
    int16_t MPU6050_readReg16BitHL(uint8_t reg);

    uint8_t MPU6050_init(void);
    uint8_t MPU6050_checkDeviceID(void);
    uint8_t MPU6050_calibrate(void);
    
    uint8_t MPU6050_read_raw_roll_pitch_yaw();
    uint8_t MPU6050_compute_accel_gyro(double accel[3],double gyro[3]);
    uint8_t MPU6050_compute_angle_gyro(uint32_t dt, double angle[2], double gyro[3]);
       
    uint8_t MPU6050_getStatus(void);
    uint8_t MPU6050_getAddress(void);
    void    MPU6050_getZero(double accel[3],double gyro[3]);
    double  MPU6050_getTemperature(void);
  
    // OLD functions
    uint8_t MPU6050_get_roll_pitch_yaw(double angle[3]);

    double MPU6050_getAccel_x(void);
    double MPU6050_getAccel_y(void);
    double MPU6050_getAccel_z(void);
    double MPU6050_getGyro_x(void);
    double MPU6050_getGyro_y(void);
    double MPU6050_getGyro_z(void);
 
    
  private:
    uint8_t _address;
    uint8_t _last_status;
    uint8_t _last_nb_receive; 
    uint32_t _previousTime, _currentTime;
    int16_t _accel_raw_x, _accel_raw_y, _accel_raw_z;
    int16_t _gyro_raw_roll, _gyro_raw_pitch, _gyro_raw_yaw;
    int16_t _temperature;
    double _accel_zero_x, _accel_zero_y, _accel_zero_z;
    double _gyro_zero_roll, _gyro_zero_pitch, _gyro_zero_yaw; 
    double _accel_filter_x, _accel_filter_y, _accel_filter_z;
    double _gyro_filter_roll, _gyro_filter_pitch, _gyro_filter_yaw; 
    double _previous_roll, _previous_pitch;
};

#endif


