#include "Arduino.h"
#include "ICM20948.h"

#define SPI_SLAVESELECTED_PIN  53

ICM20948Class ICM20948;     
      

void setup()
{

  Serial.begin(115200); // initialize serial port
  Serial.println("Start setup");
 
  ICM20948.ICM20948_initializeSPI(SPI_SLAVESELECTED_PIN);

#ifdef I2C
  ICM20948.ICM20948_initializeI2C();
#endif
  
  uint8_t status = ICM20948.ICM20948_startupDefault(false); // Don't start Magnetometer
  if (status > 0)
  {
    switch (status) {
        case CHECK_DEVICE_ERROR: Serial.println("Init ICM20948 KO, StartupDefault error: CHECK_DEVICE_ERROR");
             break;
        case CHECK_STARTUP_ERROR: Serial.println("Init ICM20948 KO, StartupDefault error: CHECK_STARTUP_ERROR");
             break;
        default: Serial.print("Init ICM20948 KO, startupDefault failed, status: "); Serial.println(status);
    } 
  }
  else Serial.println("StartupDefault OK");  
}


void loop()
{

}
