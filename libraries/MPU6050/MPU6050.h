#ifndef MPU6050_h
#define MPU6050_h

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ID 0x68

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
    
    uint8_t MPU6050_init(void);
    uint8_t MPU6050_CheckDeviceID(void);
            
    void MPU6050_writeReg(uint8_t reg, uint8_t value);

    uint8_t  MPU6050_readReg(uint8_t reg);
    int16_t MPU6050_readReg16Bit(uint8_t reg);
    
    double MPU6050_getTemperature(void);
    double MPU6050_getAccel_x(void);
    double MPU6050_getAccel_y(void);
    double MPU6050_getAccel_z(void);
    double MPU6050_getGyro_x(void);
    double MPU6050_getGyro_y(void);
    double MPU6050_getGyro_z(void);
    
    uint8_t MPU6050_calibrate(void);
    
    uint8_t MPU6050_get_roll_pitch_yaw(double angle[3]);
    
    uint8_t MPU6050_getStatus(void);
    uint8_t MPU6050_getAddress(void);
    
  private:
    uint8_t _address;
    uint8_t _last_status;
    long _previousTime, _currentTime;
    double _ax_zero, _ay_zero, _az_zero;
    double _gyrox_zero, _gyroy_zero, _gyroz_zero;
    double _previous_roll, _previous_pitch;
    double _a;
};

#endif


