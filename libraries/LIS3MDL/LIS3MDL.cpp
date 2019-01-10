#include <Arduino.h>
#include <LIS3MDL.h>



LIS3MDLClass::LIS3MDLClass(void)
{

}


uint8_t LIS3MDLClass::LIS3MDL_init()
{
    uint8_t testReg = 0;
       
    _mx_zero = 0.0;
    _my_zero = 0.0;
    _mz_zero = 0.0;
    _last_status = 0;
    _last_nb_receive = 0;
      
    // check high address
    testReg = LIS3MDL_testReg(LIS3MDL_HIGH_ADDRESS);
    if (testReg == 0)
    {
        _address = LIS3MDL_HIGH_ADDRESS;
    }
    // check low address
    else if (_last_status >0) // try second address
    {
         testReg = LIS3MDL_testReg(LIS3MDL_LOW_ADDRESS);
         if (testReg == 0)
         {
           _address = LIS3MDL_LOW_ADDRESS;
         }
         else if (_last_status >0) return _last_status;    
    }
    
    if (testReg != 0)
    {
        Serial.print("Bad ID: 0x");Serial.print(testReg,HEX); Serial.print("Should be: 0x"); Serial.print(LIS3MDL_ID,HEX);
    }
        
/* Configure the LIS3MDL's magnetometer */
   
    // 0x00 = 0b0 10 0 0 0 00
    // FS = 10 (?12 gauss full scale ie 2281 LSB/gauss); REBOOT=0; SOFT_RST=0
    LIS3MDL_writeReg(CTRL_REG2, 0x40);
    if (_last_status > 0) return _last_status;

    // 0xF0 = 0b1 11 111 0 0
    // TEMP_EN = 1; OM = 11 (ultra-high-performance mode for X and Y); DO = 111 (80 Hz ODR); FAST_ODR=0; ST=0
    LIS3MDL_writeReg(CTRL_REG1, 0xFC);
    if (_last_status > 0) return _last_status;

    // 0x0C = 0b0000 11 0 0
    // OMZ = 11 (ultra-high-performance mode for Z); BLE=0 (data LSb at lower address)
    LIS3MDL_writeReg(CTRL_REG4, 0x0C);
    if (_last_status > 0) return _last_status;
        
    // 0x00 = 0b00 0 00 0 00
    // LP=0; SIM =0; MD = 00 (continuous-conversion mode)
    LIS3MDL_writeReg(CTRL_REG3, 0x00);
    if (_last_status > 0) return _last_status;
  
  uint8_t a = LIS3MDLClass::LIS3MDL_readReg(0x05);
  Serial.print("0x05: ");Serial.println(a,HEX);
   
  uint8_t b = LIS3MDLClass::LIS3MDL_readReg(0x06);
  Serial.print("0x06: ");Serial.println(b,HEX);
   
  uint8_t c = LIS3MDLClass::LIS3MDL_readReg(0x07);
  Serial.print("0x07: ");Serial.println(c,HEX);
   
  uint8_t d = LIS3MDLClass::LIS3MDL_readReg(0x08);
  Serial.print("0x08: ");Serial.println(d,HEX);
   
  uint8_t e = LIS3MDLClass::LIS3MDL_readReg(0x09);
  Serial.print("0x09: ");Serial.println(e,HEX);
   
  uint8_t h = LIS3MDLClass::LIS3MDL_readReg(0x0A);
  Serial.print("0x0A: ");Serial.println(h,HEX);
  
  uint8_t f;
    f = LIS3MDLClass::LIS3MDL_readReg(CTRL_REG1);
  Serial.print("CTRL_REG1: 0x");Serial.println(f,HEX);
     f = LIS3MDLClass::LIS3MDL_readReg(CTRL_REG2);
  Serial.print("CTRL_REG2: 0x");Serial.println(f,HEX);
     f = LIS3MDLClass::LIS3MDL_readReg(CTRL_REG3);
  Serial.print("CTRL_REG3: 0x");Serial.println(f,HEX);
     f = LIS3MDLClass::LIS3MDL_readReg(CTRL_REG4);
  Serial.print("CTRL_REG4: 0x");Serial.println(f,HEX);
     f = LIS3MDLClass::LIS3MDL_readReg(CTRL_REG5);
  Serial.print("CTRL_REG5: 0x");Serial.println(f,HEX);
    return 0;   
}

// Write an 8-bit register
void LIS3MDLClass::LIS3MDL_writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  _last_status = Wire.endTransmission();
}

// Read an 8-bit register
uint8_t LIS3MDLClass::LIS3MDL_readReg(uint8_t reg)
{
  uint8_t value=0;

  Wire.beginTransmission(_address);
  Wire.write(reg);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)1);
  if (_last_nb_receive != 1) {_last_status=WIRE_REQUEST_ERROR;  Serial.print("WIRE_REQUEST_ERROR 1, _last_nb_receive: ");Serial.println(_last_nb_receive);return _last_status;}
  value = Wire.read();

  return value;
}

// Read a 16-bit register low byte first and then high byte
int16_t LIS3MDLClass::LIS3MDL_readReg16BitLH(uint8_t reg)
{
  int16_t value;
  uint8_t regstatus = 0x00;

  while (regstatus == 0x00)  // wait for values overwritten
  {
    uint8_t b = LIS3MDL_readReg(STATUS_REG);
    if (_last_status > 0) return _last_status;
    regstatus = b & 0x80;
  }
  
  Wire.beginTransmission(_address);
  Wire.write(reg);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)2);
  if (_last_nb_receive != 2) {_last_status=WIRE_REQUEST_ERROR;  Serial.print("WIRE_REQUEST_ERROR 2, _last_nb_receive: ");Serial.println(_last_nb_receive);return _last_status;}
    
  uint8_t lsb = Wire.read(); // value low byte
  uint8_t msb = Wire.read(); // value high byte

  value  = (((int16_t)msb) << 8) | lsb;
  
  return value;
}

double LIS3MDLClass::LIS3MDL_getMag_x()
{
  int16_t mag_x = LIS3MDL_readReg16BitLH(OUT_X_L);
  if (_last_status >0) return (double)_last_status;
  return (((double)mag_x / 2281.0) - _mx_zero);  // Full Scale Range ?12 gauss => 2281 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getMag_y()
{
  
  int16_t mag_y = LIS3MDL_readReg16BitLH(OUT_Y_L);
  if (_last_status >0) return (double)_last_status;
  return (((double)mag_y / 2281.0) - _my_zero);  // Full Scale Range ?12 gauss => 2281 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getMag_z()
{
  int16_t mag_z = LIS3MDL_readReg16BitLH(OUT_Z_L);
  if (_last_status >0) return (double)_last_status;
  return (((double)mag_z / 2281.0) - _mz_zero);  // Full Scale Range ?12 gauss => 2281 LSB/gauss
}

double LIS3MDLClass::LIS3MDL_getTemperature()
{
  int16_t temperature = LIS3MDL_readReg16BitLH(TEMP_OUT_L);
  if (_last_status >0) return (double)_last_status;
  return ((double)temperature / 256.0 + 25.0);  // The nominal sensitivity is 8 LSB/?C and 0 output means T=25 ?C
}

double LIS3MDLClass::LIS3MDL_getMagnitude(double x, double y,double z)
{
  return sqrt((x*x)+(y*y)+(z*z));
}

uint8_t LIS3MDLClass::LIS3MDL_getStatus()
{
  return _last_status;
}

uint8_t LIS3MDLClass::LIS3MDL_getLast_nb_receive()
{
  return _last_nb_receive;
}

uint8_t LIS3MDLClass::LIS3MDL_getAddress()
{
  return _address;
}


uint8_t LIS3MDLClass::LIS3MDL_testReg(uint8_t address)
{
  uint8_t value=0;
  
  Wire.beginTransmission(address);
  Wire.write(WHO_AM_I);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(address, (uint8_t)1);
  if (_last_nb_receive != 1) {_last_status=WIRE_REQUEST_ERROR;  Serial.print("WIRE_REQUEST_ERROR 0, _last_nb_receive: ");Serial.println(_last_nb_receive);return _last_status;}

  value = Wire.read();
  
  if (value == LIS3MDL_ID) return 0;
  else                     return 1;
}