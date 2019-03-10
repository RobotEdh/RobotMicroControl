#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;


void setup()
{
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  uint8_t calib = CMPS12.CMPS12_init(false); // initialize CMPS12, Do not calibration
  
  if (calib == 0)
  {
       Serial.println("Init OK");
  }
  else 
  {           
     uint8_t status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        Serial.print("Calibrate KO, calibrate status: 0b");Serial.println(calib,BIN);    
     }
     else 
     {           
        Serial.print("Init KO, I2C error: ");Serial.println(status);  
     }
  }    
}

void loop()
{
    uint8_t status = 0;
    double angle[3];
    
    double compass = CMPS12.CMPS12_getCompass();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getCompass: ");Serial.println(status);
    }
    
    double compassHighResolution = CMPS12.CMPS12_getCompassHighResolution();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getCompassHighResolution: ");Serial.println(status);
    }
    
    
    int8_t pitch = CMPS12.CMPS12_getPitch();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getPitch(): ");Serial.println(status);
    }
    
    int16_t pitch180 = CMPS12.CMPS12_getPitch180();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getPitch180: ");Serial.println(status);
    }
    
    int8_t roll = CMPS12.CMPS12_getRoll();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getRoll: ");Serial.println(status);
    }            
    
    
    int16_t mag_x = CMPS12.CMPS12_getMag_x();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getMag_x: ");Serial.println(status);
    }
    int16_t mag_y = CMPS12.CMPS12_getMag_y();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getMag_y: ");Serial.println(status);
    }
    int16_t mag_z = CMPS12.CMPS12_getMag_z();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getMag_z: ");Serial.println(status);
    }
    
    int16_t accel_x = CMPS12.CMPS12_getAccel_x();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getAccel_x: ");Serial.println(status);
    }
    int16_t accel_y = CMPS12.CMPS12_getAccel_y();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getAccel_y: ");Serial.println(status);
    }
    int16_t accel_z = CMPS12.CMPS12_getAccel_z();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getAccel_z: ");Serial.println(status);
    } 
    
    int16_t gyro_x = CMPS12.CMPS12_getGyro_x();
    status = CMPS12.CMPS12_getStatus();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getGyro_x: ");Serial.println(status);
    }
    int16_t gyro_y = CMPS12.CMPS12_getGyro_y();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getGyro_y: ");Serial.println(status);
    }
    int16_t gyro_z = CMPS12.CMPS12_getGyro_z();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getGyro_z: ");Serial.println(status);
    }    
   
    int16_t temperature = CMPS12.CMPS12_getTemperature();
    if (status > 0)
    {
       Serial.print("Error CMPS12_getTemperature: ");Serial.println(status);
    }    
    
    status = CMPS12.CMPS12_get_roll_pitch_yaw(angle);  // double range [-180;+180] ROLL, PITCH, YAW
    if (status > 0)
    {
       Serial.print("Error CMPS12_get_roll_pitch_yaw: ");Serial.println(status);
    }
    
    
      
    // display from Flash mem to avoir SRAM overload
    Serial.println(F("________________________________________________________________________"));
    Serial.print(F("compass 0-359.9 degrees: "));Serial.println(compass);
    Serial.print(F("compassHighResolution 0-359.9 degrees: "));Serial.println(compassHighResolution);  
    
    Serial.print(F("roll in degrees from the horizontal plane (+/- 90 degrees): "));Serial.println(roll);
    Serial.print(F("roll Calc: "));Serial.println(angle[0]);                   
    
    Serial.print(F("pitch in degrees from the horizontal plane (+/- 90 degrees): "));Serial.println(pitch);
    Serial.print(F("pitch180 in degrees from the horizontal plane (+/-180 degrees): "));Serial.println(pitch180);
    Serial.print(F("pitch Calc: "));Serial.println(angle[1]);    

    Serial.print(F("mag_x: "));Serial.println(mag_x);
    Serial.print(F("mag_y: "));Serial.println(mag_y);
    Serial.print(F("mag_z: "));Serial.println(mag_z);
    
    Serial.print(F("accel_x: "));Serial.println(accel_x);
    Serial.print(F("accel_y: "));Serial.println(accel_y);
    Serial.print(F("accel_z: "));Serial.println(accel_z);
    
    Serial.print(F("gyro_x: "));Serial.println(gyro_x);
    Serial.print(F("gyro_y: "));Serial.println(gyro_y);
    Serial.print(F("gyro_z: "));Serial.println(gyro_z); 
        
    Serial.print(F("temperature: "));Serial.println(temperature);

    delay(5000);
}