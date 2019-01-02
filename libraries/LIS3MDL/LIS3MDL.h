#ifndef LIS3MDL_h
#define LIS3MDL_h

#include <Arduino.h>

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

    int8_t LIS3MDL_init(void);

    int8_t  LIS3MDL_writeReg(uint8_t reg, uint8_t value);
    int8_t  LIS3MDL_readReg(uint8_t reg);
    int16_t LIS3MDL_readReg16Bit(uint8_t reg);

    double LIS3MDL_getMag_x(void);
    double LIS3MDL_getMag_y(void);
    double LIS3MDL_getMag_z(void);
    double LIS3MDL_getTemperature(void);
    
    double LIS3MDL_getMagnitude(double x, double y,double z);
    
  private:
    uint8_t address;
    double _mx_zero, _my_zero, _mz_zero;

    int16_t LIS3MDL_testReg(uint8_t address, regAddr reg);
};

#endif


