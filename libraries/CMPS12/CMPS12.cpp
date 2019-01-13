#include <Arduino.h>
#include <CMPS12.h>


CMPS12Class::CMPS12Class()
{

}

uint8_t CMPS12Class::CMPS12_init()
{
    return CMPS12Class::CMPS12_init(false); // no calibration by default
}    
    
uint8_t CMPS12Class::CMPS12_init(bool calibration)
{
    uint8_t revision = 0;
    uint8_t calib = 0;
    _last_status = 0;
    _last_nb_receive = 0;
    _address = CMPS12_ADDRESS;
    
     
    revision = CMPS12_getRevision();
    if (_last_status >0) {
          return _last_status;
    }      
    else {
         Serial.print("Revision: 0x");Serial.println(revision,HEX);
    }
    
    if (calibration) calib = CMPS12_calibrate();
    else             calib = CMPS12_checkCalibrate();

    return calib;  
}

uint8_t CMPS12Class::CMPS12_calibrate()
{    
    Serial.println("Start calibration");

    Serial.println("Gyro - being in a stationary state - 5 s");
    delay (5*1000); 
    Serial.println("Accelerometer - tilting the module to roughly 45 and 90 degrees on one axis - 10s");
    delay (10*1000); 
    Serial.println("Magnetometer - doing a few random movements - 10s"); 
    delay (10*1000); 
    Serial.println("Stop calibration");
          
    uint8_t calibrate = CMPS12_checkCalibrate();
    if (_last_status >0) {
       return _last_status;
    }      
    else 
       return calibrate;
    
}

uint8_t CMPS12Class::CMPS12_checkCalibrate()
{    
    uint8_t calibrate = CMPS12_getCalibrate();
    if (_last_status >0) {
         return _last_status;
    }      
    else {
         if ((calibrate&0x03)==0x03) Serial.println("Magnetometer calibration OK");
         else                        Serial.println("Magnetometer calibration KO");
         if ((calibrate&0x0C)==0x0C) Serial.println("Accelerometer calibration OK");
         else                        Serial.println("Accelerometer calibration KO");  
         if ((calibrate&0x30)==0x30) Serial.println("Gyro calibration OK");
         else                        Serial.println("Gyro calibration KO");    
         if ((calibrate&0xC0)==0xC0) Serial.println("System calibration OK");  
         else                        Serial.println("System calibration KO");       
    }
    
    if (calibrate == 0xFF)return 0;
    else                  return calibrate;    
    
 }    

void CMPS12Class::CMPS12_storeProfil()
{
  Wire.beginTransmission(_address);
  Wire.write(CMD_REVISION);
  
  Wire.write(0xF0);
  delay(20); //  20ms delay after each of the three bytes send
  Wire.write(0xF5);
  delay(20); //  20ms delay after each of the three bytes send
  Wire.write(0xF6);
  delay(20); //  20ms delay after each of the three bytes send
    
  _last_status = Wire.endTransmission();
}

void CMPS12Class::CMPS12_eraseProfil()
{
  Wire.beginTransmission(_address);
  Wire.write(CMD_REVISION);
  
  Wire.write(0xE0);
  delay(20); //  20ms delay after each of the three bytes send
  Wire.write(0xE5);
  delay(20); //  20ms delay after each of the three bytes send
  Wire.write(0xE2);
  delay(20); //  20ms delay after each of the three bytes send
    
  _last_status = Wire.endTransmission();
}

// Write an 8-bit register
void CMPS12Class::CMPS12_writeReg(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(_address);
  Wire.write(reg);
  Wire.write(value);
  _last_status = Wire.endTransmission();
}

// Read an 8-bit register
uint8_t CMPS12Class::CMPS12_readReg(uint8_t reg)
{
  uint8_t value=0;

  Wire.beginTransmission(_address);
  Wire.write(reg);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)1);
  if (_last_nb_receive != 1) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
  value = Wire.read();

  return value;
}

// Read a 16-bit register low byte high and then low byte
int16_t CMPS12Class::CMPS12_readReg16BitHL(uint8_t reg)
{
  int16_t value;
  
  Wire.beginTransmission(_address);
  Wire.write(reg);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)2);
  if (_last_nb_receive != 2) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
    
  uint8_t msb = Wire.read(); // value high byte
  uint8_t lsb = Wire.read(); // value low byte

  value  = (((int16_t)msb) << 8) | lsb;
  
  return value;
}

uint8_t CMPS12Class::CMPS12_getRevision()
{
  uint8_t revision = CMPS12_readReg(CMD_REVISION);
  if (_last_status >0) return _last_status;
  return revision; 
}

uint8_t CMPS12Class::CMPS12_getCalibrate()
{
  uint8_t calibrate = CMPS12_readReg(CALIBRATE);
  if (_last_status >0) return _last_status;
  return calibrate; 
}


double CMPS12Class::CMPS12_getCompass()
{
  uint8_t compass = CMPS12_readReg(OUT_COMPASS);
  if (_last_status >0) return (double)_last_status;
  return (((double)compass * 360.0 / 255.0));  // 0-255 for a full circle
}

double CMPS12Class::CMPS12_getCompassHighResolution()
{
  int16_t compassHighResolution = CMPS12_readReg16BitHL(OUT_COMPASS_H);
  if (_last_status >0) return (double)_last_status;
  return (((double)compassHighResolution / 10.0));  // 0-3599, representing 0-359.9 degrees
}


int8_t CMPS12Class::CMPS12_getPitch()
{
  uint8_t pitch = CMPS12_readReg(OUT_PITCH);
  if (_last_status >0) return (int8_t)_last_status;
  return -1*(int8_t)pitch; // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
}

int16_t CMPS12Class::CMPS12_getPitch180()
{
  int16_t pitch180 = CMPS12_readReg16BitHL(OUT_PITCH_H);
  if (_last_status >0) return (int8_t)_last_status;
  return pitch180; // signed byte giving angle in degrees from the horizontal plane (+/-180 degrees)
}


int8_t CMPS12Class::CMPS12_getRoll()
{
  uint8_t roll = CMPS12_readReg(OUT_ROLL);
  if (_last_status >0) return (int8_t)_last_status;
  return -1*(int8_t)roll; // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
}

int16_t CMPS12Class::CMPS12_getMag_x()
{
  int16_t mag_x = CMPS12_readReg16BitHL(OUT_MAG_X_H);
  if (_last_status >0) return (int16_t)_last_status;
  return mag_x;
}

int16_t CMPS12Class::CMPS12_getMag_y()
{
  
  int16_t mag_y = CMPS12_readReg16BitHL(OUT_MAG_Y_H);
  if (_last_status >0) return (int16_t)_last_status;
  return mag_y;  
}

int16_t CMPS12Class::CMPS12_getMag_z()
{
  int16_t mag_z = CMPS12_readReg16BitHL(OUT_MAG_Z_H);
  if (_last_status >0) return (int16_t)_last_status;
  return mag_z;  
}

int16_t CMPS12Class::CMPS12_getAccel_x()
{
  int16_t accel_x = CMPS12_readReg16BitHL(OUT_ACCEL_X_H);
  if (_last_status >0) return (int16_t)_last_status;
  return accel_x; 
}

int16_t CMPS12Class::CMPS12_getAccel_y()
{
  
  int16_t accel_y = CMPS12_readReg16BitHL(OUT_ACCEL_Y_H);
  if (_last_status >0) return (int16_t)_last_status;
  return accel_y;  
}

int16_t CMPS12Class::CMPS12_getAccel_z()
{
  int16_t accel_z = CMPS12_readReg16BitHL(OUT_ACCEL_Z_H);
  if (_last_status >0) return (int16_t)_last_status;
  return accel_z; 
}

int16_t CMPS12Class::CMPS12_getGyro_x()
{
  int16_t gyro_x = CMPS12_readReg16BitHL(OUT_GYRO_X_H);
  if (_last_status >0) return (int16_t)_last_status;
  return gyro_x;  
}

int16_t CMPS12Class::CMPS12_getGyro_y()
{
  
  int16_t gyro_y = CMPS12_readReg16BitHL(OUT_GYRO_Y_H);
  if (_last_status >0) return (int16_t)_last_status;
  return gyro_y;  
}

int16_t CMPS12Class::CMPS12_getGyro_z()
{
  int16_t gyro_z = CMPS12_readReg16BitHL(OUT_GYRO_Z_H);
  if (_last_status >0) return (int16_t)_last_status;
  return gyro_z;  
}


int16_t CMPS12Class::CMPS12_getTemperature()
{
  int16_t temperature = CMPS12_readReg16BitHL(OUT_TEMPERATURE_H);
  if (_last_status >0) return (int16_t)_last_status;
  return temperature;
}


double CMPS12Class::CMPS12_getMagnitude(double x, double y,double z)
{
  return sqrt((x*x)+(y*y)+(z*z));
}

uint8_t CMPS12Class::CMPS12_getStatus()
{
  return _last_status;
}

uint8_t CMPS12Class::CMPS12_getLast_nb_receive()
{
  return _last_nb_receive;
}

uint8_t CMPS12Class::CMPS12_getAddress()
{
  return _address;
}
