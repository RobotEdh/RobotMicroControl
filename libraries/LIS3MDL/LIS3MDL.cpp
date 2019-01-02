#include <LIS3MDL.h>
#include <Wire.h>
#include <math.h>

// Defines ////////////////////////////////////////////////////////////////

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

// Constructors ////////////////////////////////////////////////////////////////

LIS3MDLClass::LIS3MDLClass(void)
{

}

// Public Methods //////////////////////////////////////////////////////////////

int8_t LIS3MDLClass::LIS3MDL_init()
{
      int8_t status = 0;
      // check SA1 high address
      if (LIS3MDL_testReg(LIS3MDL_SA1_HIGH_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        address = LIS3MDL_SA1_HIGH_ADDRESS;
      }
      // check SA1 low address
      else if (LIS3MDL_testReg(LIS3MDL_SA1_LOW_ADDRESS, WHO_AM_I) == LIS3MDL_WHO_ID)
      {
        address = LIS3MDL_SA1_LOW_ADDRESS;
      }
      else return 0;
/*
Enables the LIS3MDL's magnetometer. Also:
- Selects ultra-high-performance mode for all axes
- Sets ODR (output data rate) to default power-on value of 10 Hz
- Sets magnetometer full scale (gain) to default power-on value of +/- 4 gauss
- Enables continuous conversion mode
Note that this function will also reset other settings controlled by
the registers it writes to.
*/        
    // 0xF0 = 0b1 11 100 0 0
    // TEMP_EN = 1; OM = 11 (ultra-high-performance mode for X and Y); DO = 100 (10 Hz ODR); FAST_ODR=0; ST=0
    status = LIS3MDL_writeReg(CTRL_REG1, 0xF0);
    if (status != 0) return status;

    // 0x00 = 0b0 00 0 0 0 00
    // FS = 00 (+/- 4 gauss full scale ie 6842 LSB/gauss); REBOOT=0; SOFT_RST=0
    status = LIS3MDL_writeReg(CTRL_REG2, 0x00);
    if (status != 0) return status;

    // 0x00 = 0b00 0 00 0 00
    // LP=0; SIM =0; MD = 00 (continuous-conversion mode)
    status = LIS3MDL_writeReg(CTRL_REG3, 0x00);
    if (status != 0) return status;

    // 0x0C = 0b0000 11 0 0
    // OMZ = 11 (ultra-high-performance mode for Z); BLE=0 (data LSb at lower address)
    status = LIS3MDL_writeReg(CTRL_REG4, 0x0C);
    if (status != 0) return status;  
    
    // 0x0C = 0b0 0 000000
    // FAST_READ=0; BDU=0
    status = LIS3MDL_writeReg(CTRL_REG5, 0x00);
    if (status != 0) return status; 
    
    return (int8_t)address;   
}

// Writes a mag register
int8_t LIS3MDLClass::LIS3MDL_writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  uint8_t status = Wire.endTransmission();
  return (-1*status);
}

// Reads a mag register
int8_t LIS3MDLClass::LIS3MDL_readReg(uint8_t reg)
{
  uint8_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  uint8_t status = Wire.endTransmission();
  if (status > 0) return (-1*status);
    
  Wire.requestFrom(address, (uint8_t)1);
  value = Wire.read();
  Wire.endTransmission();

  return (int8_t)value;
}

// Read a 16-bit register
int16_t LIS3MDLClass::LIS3MDL_readReg16Bit(uint8_t reg)
{
  uint16_t value;

  Wire.beginTransmission(address);
  Wire.write(reg);
  uint8_t status = Wire.endTransmission();
  if (status > 0) return (-1*status);

  Wire.requestFrom(address, (uint8_t)2);
  value  = (uint16_t)Wire.read() << 8; // value high byte
  value |=           Wire.read();      // value low byte

  return (int16_t)value;
}

double LIS3MDLClass::LIS3MDL_getMag_x()
{
  int16_t mag_x = LIS3MDL_readReg16Bit(OUT_X_L);
  return (((double)mag_x / 6842.0) - _mx_zero);  // Full Scale Range ±4 gauss => 6842 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getMag_y()
{
  int16_t mag_y = LIS3MDL_readReg16Bit(OUT_Y_L);
  return (((double)mag_y / 6842.0) - _my_zero);  // Full Scale Range ±4 gauss => 6842 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getMag_z()
{
  int16_t mag_z = LIS3MDL_readReg16Bit(OUT_Z_L);
  return (((double)mag_z / 6842.0) - _mz_zero);  // Full Scale Range ±4 gauss => 6842 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getTemperature()
{
  int16_t temperature = LIS3MDL_readReg16Bit(TEMP_OUT_L);
  return ((double)temperature / 8.0);  // 8 LSB/°C
}

double LIS3MDLClass::LIS3MDL_getMagnitude(double x, double y,double z)
{
  return sqrt((x*x)+(y*y)+(z*z));

}

// Private Methods //////////////////////////////////////////////////////////////

int16_t LIS3MDLClass::LIS3MDL_testReg(uint8_t address, regAddr reg)
{
  Wire.beginTransmission(address);
  Wire.write((uint8_t)reg);
  if (Wire.endTransmission() != 0)
  {
    return TEST_REG_ERROR;
  }

  Wire.requestFrom(address, (uint8_t)1);
  if (Wire.available())
  {
    return Wire.read();
  }
  else
  {
    return TEST_REG_ERROR;
  }
}