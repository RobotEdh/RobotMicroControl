#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <Arduino.h>
#include <Wire.h>

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_HIGH_ADDRESS  0b0011110
#define LIS3MDL_LOW_ADDRESS   0b0011100

#define LIS3MDL_ID  0x3D


#define WIRE_TRANSMIT_SUCESS          0x00 // Wire.endTransmission()- 0:success
#define WIRE_ERROR_TRANSMIT_TOOLONG   0x01 // Wire.endTransmission()- 1:data too long to fit in transmit buffer
#define WIRE_ERROR_TRANSMIT_ADR_NACK  0x02 // Wire.endTransmission()- 2:received NACK on transmit of address
#define WIRE_ERROR_TRANSMIT_DATA_NACK 0x03 // Wire.endTransmission()- 3:received NACK on transmit of data
#define WIRE_TRANSMIT_ERROR_OTHER     0x04 // Wire.endTransmission()- 4:other error
#define WIRE_REQUEST_ERROR            0x80 // Wire.requestFrom()- the number of bytes returned from the slave device != the number of bytes to request

class LIS3MDLClass
{
  public:

    // register addresses
    enum regAddr
    {
      WHO_AM_I    = 0x0F,

      CTRL_REG1   = 0x20,
      CTRL_REG2   = 0x21,
      CTRL_REG3   = 0x22,
      CTRL_REG4   = 0x23,
      CTRL_REG5   = 0x24,

      STATUS_REG  = 0x27,
      OUT_X_L     = 0x28,
      OUT_X_H     = 0x29,
      OUT_Y_L     = 0x2A,
      OUT_Y_H     = 0x2B,
      OUT_Z_L     = 0x2C,
      OUT_Z_H     = 0x2D,
      TEMP_OUT_L  = 0x2E,
      TEMP_OUT_H  = 0x2F,
      INT_CFG     = 0x30,
      INT_SRC     = 0x31,
      INT_THS_L   = 0x32,
      INT_THS_H   = 0x33,
    };

    LIS3MDLClass();

    uint8_t LIS3MDL_init(void);

    void    LIS3MDL_writeReg(uint8_t reg, uint8_t value);
    uint8_t LIS3MDL_readReg(uint8_t reg);
    int16_t LIS3MDL_readReg16BitLH(uint8_t reg);

    double LIS3MDL_getMag_x(void);
    double LIS3MDL_getMag_y(void);
    double LIS3MDL_getMag_z(void);
    double LIS3MDL_getTemperature(void);
    
    double LIS3MDL_getMagnitude(double x, double y,double z);
    uint8_t LIS3MDL_getStatus(void);
    uint8_t LIS3MDL_getAddress(void);
    
  private:
    uint8_t _address;
    uint8_t _last_status;
    uint8_t _last_nb_receive;
    double _mx_zero, _my_zero, _mz_zero;

    uint8_t LIS3MDL_testReg(uint8_t address);
};

#endif

