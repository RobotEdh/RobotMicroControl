#include <Arduino.h>
#include <CMPS12.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
//#define LOGTRACE   // Enable trace
#include <log.h>
extern File logFile;                // The loging class

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
    _previousTime = 0;
    
    _ax_zero = 0;  
    _ay_zero = 0; 
    _az_zero = 0; 
    _gyrox_zero = 0;      
    _gyroy_zero = 0;
    _gyroz_zero = 0;
    
    _a = 0.98;
    _address = CMPS12_ADDRESS;
    
     
    revision = CMPS12_getRevision();
    if (_last_status >0) {
          return _last_status;
    }      
    else {
         PRINTx("Revision: ",revision)
    }
    
    if (calibration) calib = CMPS12_calibrate();
    else             calib = CMPS12_checkCalibrate();

    return calib;  
}

uint8_t CMPS12Class::CMPS12_calibrate()
{    
    PRINTs("Start calibration")

    PRINTs("Gyro - being in a stationary state - 5 s")
    delay (5*1000); 
    PRINTs("Accelerometer - tilting the module to roughly 45 and 90 degrees on one axis - 10s")
    delay (10*1000); 
    PRINTs("Magnetometer - doing a few random movements - 10s")
    delay (10*1000); 
    PRINTs("Stop calibration")
          
    uint8_t calibrate = CMPS12_checkCalibrate();
    if (_last_status >0) {
       return _last_status;
    }      
    else 
       return calibrate;
    
}

uint8_t CMPS12Class::CMPS12_set_zero_values()
{
  int32_t ax_tot = 0;
  int32_t ay_tot = 0;
  int32_t az_tot = 0;
  int32_t gyrox_tot = 0;
  int32_t gyroy_tot = 0;
  int32_t gyroz_tot = 0;
  
  int16_t value = 0;
  
  PRINTs(">>>Start CMPS12_set_zero_values")
    
  _ax_zero = 0.0;
  _ay_zero = 0.0;
  _az_zero = 0.0;
  _gyrox_zero = 0.0;
  _gyroy_zero = 0.0;
  _gyroz_zero = 0.0;
  
  for (int i = 0; i < 255; i++) {
     value = CMPS12_getAccel_x();
     if (_last_status > 0) return _last_status; 
     else ax_tot += value;
        
     value = CMPS12_getAccel_y();
     if (_last_status > 0) return _last_status;
     else ay_tot += value;
        
     value = CMPS12_getAccel_z();
     if (_last_status > 0) return _last_status;
     else az_tot += value;   
        
     value = CMPS12_getGyro_x();
     if (_last_status > 0) return _last_status;
     else gyrox_tot += value;
        
     value = CMPS12_getGyro_y();
     if (_last_status > 0) return _last_status;   
     else gyroy_tot += value; 
            
     value = CMPS12_getGyro_z();
     if (_last_status > 0) return _last_status;
     else gyroz_tot += value;     
     
     delay(5);  // 5 ms for Sample
  }
  _ax_zero = ax_tot/255;
  _ay_zero = ay_tot/255;
  _az_zero = (az_tot/255) - 1000.0;  // keep 1g
  _gyrox_zero = gyrox_tot/255;
  _gyroy_zero = gyroy_tot/255;
  _gyroz_zero = gyroz_tot/255;
  
  PRINT("_ax_zero: ",_ax_zero)  
  PRINT("_ay_zero: ",_ay_zero)   
  PRINT("_az_zero: ",_az_zero)   
  PRINT("_gyrox_zero: ",_gyrox_zero)
  PRINT("_gyroy_zero: ",_gyroy_zero)   
  PRINT("_gyroz_zero: ",_gyroz_zero)
  
  PRINTs(">>>End OK CMPS12_set_zero_values")
  return 0;        
}


uint8_t CMPS12Class::CMPS12_checkCalibrate()
{    
    uint8_t calibrate = CMPS12_getCalibrate();
    if (_last_status >0) {
         return _last_status;
    }      
    else {
         if ((calibrate&0x03)==0x03) PRINTs("Magnetometer calibration OK")
         else                        PRINTs("Magnetometer calibration KO")
         if ((calibrate&0x0C)==0x0C) PRINTs("Accelerometer calibration OK")
         else                        PRINTs("Accelerometer calibration KO")  
         if ((calibrate&0x30)==0x30) PRINTs("Gyro calibration OK")
         else                        PRINTs("Gyro calibration KO")    
         if ((calibrate&0xC0)==0xC0) PRINTs("System calibration OK")  
         else                        PRINTs("System calibration KO")       
    }
    
    if (calibrate == 0xFF)return 0;
    else                  return calibrate;    
    
 }    

uint8_t CMPS12Class::CMPS12_storeProfil()
{
  CMPS12_writeReg(CMD_REVISION, 0xF0);
  if (_last_status > 0) return _last_status;
  delay(20); //  20ms delay after each of the three bytes send
      
  CMPS12_writeReg(CMD_REVISION, 0xF5);
  if (_last_status > 0) return _last_status;
  delay(20); //  20ms delay after each of the three bytes send
      
  CMPS12_writeReg(CMD_REVISION, 0xF6);
  if (_last_status > 0) return _last_status; 
  delay(20); //  20ms delay after each of the three bytes send
  
  return 0;
}

uint8_t CMPS12Class::CMPS12_eraseProfil()
{
  CMPS12_writeReg(CMD_REVISION, 0xE0);
  if (_last_status > 0) return _last_status;
  delay(20); //  20ms delay after each of the three bytes send
      
  CMPS12_writeReg(CMD_REVISION, 0xE5);
  if (_last_status > 0) return _last_status;
  delay(20); //  20ms delay after each of the three bytes send
      
  CMPS12_writeReg(CMD_REVISION, 0xE2);
  if (_last_status > 0) return _last_status; 
  delay(20); //  20ms delay after each of the three bytes send
  
  return 0;
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
  return (accel_x - _ax_zero); 
}

int16_t CMPS12Class::CMPS12_getAccel_y()
{
  
  int16_t accel_y = CMPS12_readReg16BitHL(OUT_ACCEL_Y_H);
  if (_last_status >0) return (int16_t)_last_status;
  return (accel_y - _ay_zero);  
}

int16_t CMPS12Class::CMPS12_getAccel_z()
{
  int16_t accel_z = CMPS12_readReg16BitHL(OUT_ACCEL_Z_H);
  if (_last_status >0) return (int16_t)_last_status;
  return (accel_z - _az_zero); 
}

int16_t CMPS12Class::CMPS12_getGyro_x()
{
  int16_t gyro_x = CMPS12_readReg16BitHL(OUT_GYRO_X_H);
  if (_last_status >0) return (int16_t)_last_status;
  return (gyro_x - _gyrox_zero);  
}

int16_t CMPS12Class::CMPS12_getGyro_y()
{
  
  int16_t gyro_y = CMPS12_readReg16BitHL(OUT_GYRO_Y_H);
  if (_last_status >0) return (int16_t)_last_status;
  return (gyro_y - _gyroy_zero);  
}

int16_t CMPS12Class::CMPS12_getGyro_z()
{
  int16_t gyro_z = CMPS12_readReg16BitHL(OUT_GYRO_Z_H);
  if (_last_status >0) return (int16_t)_last_status;
  return (gyro_z - _gyroz_zero);  
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


uint8_t CMPS12Class::CMPS12_get_roll_pitch_yaw(double angle[3])
{
  return CMPS12_get_roll_pitch_yaw(angle, true);
}

uint8_t CMPS12Class::CMPS12_get_roll_pitch_yaw(double angle[3], bool compFilter)
{
     double a_roll  = 0.0;
     double a_pitch = 0.0;
     double g_roll  = 0.0;
     double g_pitch = 0.0;
     double g_yaw   = 0.0;
     
         
     double ax = (double)CMPS12_getAccel_x();
     if (_last_status > 0) return _last_status;

     double ay = (double)CMPS12_getAccel_y();
     if (_last_status > 0) return _last_status;
         
     double az = (double)CMPS12_getAccel_z();
     if (_last_status > 0) return _last_status;; 
             
     a_pitch  = atan2(ay, az) * 180.0/PI;  //The returned value of atan2 is in the range [-pi, +pi] radians => convert result in degree
     a_roll = atan2(-1.0*ax, sqrt(ay*ay + az*az)) * 180.0/PI; //The returned value of atan2 is in the range [-pi, +pi] radians  => convert result in degree

     double gyrox = CMPS12_getGyro_x();
     if (_last_status > 0) return _last_status;
         
     double gyroy = CMPS12_getGyro_y();
     if (_last_status > 0) return _last_status;
           
     double gyroz = CMPS12_getGyro_z();
     if (_last_status > 0) return _last_status;
     
     _currentTime = millis();  
     if (_previousTime > 0) {
        long dt = _currentTime - _previousTime;

        // integrate the gyros angular velocity in degree/sec 
        g_roll  = gyrox * (double)dt/1000.0;
        g_pitch = gyroy * (double)dt/1000.0;
        g_yaw   = gyroz * (double)dt/1000.0; 
        
        if (compFilter) {    
           // adjust angles Roll & Pitch using complementary filter
           angle[0] = _a * (_previous_roll  + g_roll)  + (1.0 - _a) * a_roll;
           angle[1] = _a * (_previous_pitch + g_pitch) + (1.0 - _a) * a_pitch;    
           angle[2] = g_yaw;
        }
        else
        {
           angle[0] = a_roll;
           angle[1] = a_pitch;
           angle[2] = g_yaw;
        }           
     }
     else
     {
        angle[0] = a_roll;
        angle[1] = a_pitch;
        angle[2]  = 0.0;
     }  
      
     _previousTime = _currentTime;
     _previous_roll  =  angle[0];
     _previous_pitch =  angle[1]; 
     
     return 0;
}