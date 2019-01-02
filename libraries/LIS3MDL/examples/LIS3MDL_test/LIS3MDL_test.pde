/*
The sensor outputs provided by the library are the raw 16-bit values
obtained by concatenating the 8-bit high and low magnetometer data registers.
They can be converted to units of gauss using the
conversion factors specified in the datasheet for your particular
device and full scale setting (gain).

Example: An LIS3MDL gives a magnetometer X axis reading of 1292 with its
default full scale setting of +/- 4 gauss. The GN specification
in the LIS3MDL datasheet (page 8) states a conversion factor of 6842
LSB/gauss (where LSB means least significant bit) at this FS setting, so the raw
reading of 1292 corresponds to 1292 / 6842 = 0.1888 gauss.
*/

#include <Wire.h>
#include <LIS3MDL.h>

LIS3MDLClass LIS3MDL;


void setup()
{
  Serial.begin(9600);
  Wire.begin();

  int8_t address = LIS3MDL.LIS3MDL_init();
  if (address == 0)
  {
    Serial.println("Failed to detect and initialize magnetometer");
  }
  else if (address < 0)
  {
    Serial.println("Failed to configure magnetometer");
  }
  else
  {
    Serial.print("Init OK, Address: ");Serial.println(address,HEX);
  }

}

void loop()
{
    double mag_x = LIS3MDL.LIS3MDL_getMag_x();
    double mag_y = LIS3MDL.LIS3MDL_getMag_y();
    double mag_z = LIS3MDL.LIS3MDL_getMag_z();

    double magnitude = LIS3MDL.LIS3MDL_getMagnitude(mag_x, mag_y, mag_z);
    
    double temperature = LIS3MDL.LIS3MDL_getTemperature();
            
    Serial.print("mag_x (gauss): ");Serial.println(mag_x);
    Serial.print("mag_y (gauss): ");Serial.println(mag_y);
    Serial.print("mag_z (gauss): ");Serial.println(mag_z);
    Serial.print("magnitude(gauss): ");Serial.println(magnitude);
    Serial.print("temperature (°C): ");Serial.println(temperature);

    delay(100);
}