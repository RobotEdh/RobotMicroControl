#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;


void setup()
{
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  CMPS12.CMPS12_init();
}

void loop()
{
    uint8_t status = 0;
    
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
    
    Serial.print("compass: ");Serial.println(compass);
    Serial.print("compassHighResolution: ");Serial.println(compassHighResolution);  
    
    Serial.print("pitch: ");Serial.println(pitch);
    Serial.print("pitch180: ");Serial.println(pitch180);
    Serial.print("roll: ");Serial.println(roll);
                
    Serial.print("mag_x: ");Serial.println(mag_x);
    Serial.print("mag_y: ");Serial.println(mag_y);
    Serial.print("mag_z: ");Serial.println(mag_z);
    
    Serial.print("accel_x: ");Serial.println(accel_x);
    Serial.print("accel_y: ");Serial.println(accel_y);
    Serial.print("accel_z: ");Serial.println(accel_z);
    
    Serial.print("gyro_x: ");Serial.println(gyro_x);
    Serial.print("gyro_y: ");Serial.println(gyro_y);
    Serial.print("gyro_z: ");Serial.println(gyro_z); 
        
    Serial.print("temperature: ");Serial.println(temperature);

    delay(5000);
}
