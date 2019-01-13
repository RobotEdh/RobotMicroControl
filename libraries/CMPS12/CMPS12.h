#ifndef CMPS12_h
#define CMPS12_h

#include <Arduino.h>
#include <Wire.h>

#define CMPS12_ADDRESS   0x60

#define WIRE_TRANSMIT_SUCESS          0x00 // Wire.endTransmission()- 0:success
#define WIRE_ERROR_TRANSMIT_TOOLONG   0x01 // Wire.endTransmission()- 1:data too long to fit in transmit buffer
#define WIRE_ERROR_TRANSMIT_ADR_NACK  0x02 // Wire.endTransmission()- 2:received NACK on transmit of address
#define WIRE_ERROR_TRANSMIT_DATA_NACK 0x03 // Wire.endTransmission()- 3:received NACK on transmit of data
#define WIRE_TRANSMIT_ERROR_OTHER     0x04 // Wire.endTransmission()- 4:other error
#define WIRE_REQUEST_ERROR            0x80 // Wire.requestFrom()- the number of bytes returned from the slave device != the number of bytes to request

class CMPS12Class
{
  public:

    // register addresses
    enum regAddr
    {
      CMD_REVISION      = 0x00,

      OUT_COMPASS       = 0x01,
      OUT_COMPASS_H     = 0x02,
      OUT_PITCH         = 0x04,
      OUT_ROLL          = 0x05,

      OUT_MAG_X_H       = 0x08,
      OUT_MAG_Y_H       = 0x08,
      OUT_MAG_Z_H       = 0x0A,
      OUT_ACCEL_X_H     = 0x0C,
      OUT_ACCEL_Y_H     = 0x0E,
      OUT_ACCEL_Z_H     = 0x10,
      OUT_GYRO_X_H      = 0x12,
      OUT_GYRO_Y_H      = 0x14,
      OUT_GYRO_Z_H      = 0x16,
      OUT_TEMPERATURE_H = 0x18,
      
      OUT_PITCH_H       = 0x1C,
      
      CALIBRATE         = 0x1E,
    };

    CMPS12Class();

    uint8_t CMPS12_init(void);
    uint8_t CMPS12_init(bool calibration);
    uint8_t CMPS12_calibrate(void);
    uint8_t CMPS12_checkCalibrate(void);
    void    CMPS12_writeReg(uint8_t reg, uint8_t value);         
    uint8_t CMPS12_readReg(uint8_t reg);
    int16_t CMPS12_readReg16BitHL(uint8_t reg);
    
    uint8_t CMPS12_storeProfil(void);
    uint8_t CMPS12_eraseProfil(void);   
    uint8_t CMPS12_getRevision(void);
    uint8_t CMPS12_getCalibrate(void);

    double  CMPS12_getCompass(void);
    double  CMPS12_getCompassHighResolution(void);
    int8_t  CMPS12_getPitch(void);
    int16_t CMPS12_getPitch180(void);
    int8_t  CMPS12_getRoll(void);
    int16_t CMPS12_getMag_x(void);  
    int16_t CMPS12_getMag_y(void); 
    int16_t CMPS12_getMag_z(void);
    int16_t CMPS12_getAccel_x(void);
    int16_t CMPS12_getAccel_y(void);
    int16_t CMPS12_getAccel_z(void);                    
    int16_t CMPS12_getGyro_x(void);
    int16_t CMPS12_getGyro_y(void);
    int16_t CMPS12_getGyro_z(void);
    int16_t CMPS12_getTemperature(void);
        
    double  CMPS12_getMagnitude(double x, double y, double z);
    uint8_t CMPS12_getStatus(void);
    uint8_t CMPS12_getLast_nb_receive(void);
    uint8_t CMPS12_getAddress(void);
    
  private:
    uint8_t _address;
    uint8_t _last_status;
    uint8_t _last_nb_receive;     
};

#endif

