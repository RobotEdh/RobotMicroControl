#include <Arduino.h>
#include <LIS3MDL.h>

LIS3MDLClass LIS3MDL;


void setup()
{
  uint8_t status = 0;
  uint8_t devid= 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  devid = LIS3MDL.LIS3MDL_init();
  if (devid > 0)
  {
    status = LIS3MDL.LIS3MDL_getStatus();
    if (status > 0)
    {
       Serial.print("Failed to detect and initialize magnetometer, error:");Serial.println(status);
    } 
    else
    {
       Serial.print("Baddevice ID: 0x"); Serial.print(devid);Serial.print(" should be: 0x"); Serial.println(LIS3MDL_ID);
    } 
  }
  else
  {
    uint8_t address = LIS3MDL.LIS3MDL_getAddress();
    Serial.print("Init OK, Address: 0x"); Serial.println(address,HEX);
  }

}

void loop()
{
    uint8_t status = 0;
    
    double mag_x = LIS3MDL.LIS3MDL_getMag_x();
    status = LIS3MDL.LIS3MDL_getStatus();
    if (status > 0)
    {
       Serial.print("Error LIS3MDL_getMag_x: ");Serial.println(status);
    }
    double mag_y = LIS3MDL.LIS3MDL_getMag_y();
    if (status > 0)
    {
       Serial.print("Error LIS3MDL_getMag_y: ");Serial.println(status);
    }
    double mag_z = LIS3MDL.LIS3MDL_getMag_z();
    if (status > 0)
    {
       Serial.print("Error LIS3MDL_getMag_z: ");Serial.println(status);
    }   

    double magnitude = LIS3MDL.LIS3MDL_getMagnitude(mag_x, mag_y, mag_z);
    double heading = (atan2(mag_y, mag_x) * 180.0) / PI;

    
    double temperature = LIS3MDL.LIS3MDL_getTemperature();
    if (status > 0)
    {
       Serial.print("Error LIS3MDL_getTemperature: ");Serial.println(status);
    }    
            
    Serial.print("mag_x (gauss): ");Serial.println(mag_x);
    Serial.print("mag_y (gauss): ");Serial.println(mag_y);
    Serial.print("mag_z (gauss): ");Serial.println(mag_z);
    Serial.print("magnitude(gauss): ");Serial.println(magnitude);
    Serial.print("heading: ");Serial.println(heading);
    Serial.print("temperature: ");Serial.println(temperature);

    delay(5000);
}
