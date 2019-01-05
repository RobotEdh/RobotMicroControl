#include <Arduino.h>
#include <MPU6050.h>
 
/* I2C interface is provided on pins:                                          */
/*         1 = Power +3V                                                       */
/*         2 = SCL                                                             */
/*         3 = SDA                                                             */
/*         9 = Ground                                                          */



MPU6050Class MPU6050;     
      


void setup()
{
  uint8_t status = 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C
  
  status = MPU6050.MPU6050_init(); // initialize MPU6050
  if (status > 0)
  {
    Serial.print("Failed to detect and initialize magMPU6050, error:");Serial.println(status);
  }
  else
  {
    uint8_t address = MPU6050.MPU6050_getAddress();
    Serial.print("Init OK, Address: "); Serial.println(address,HEX);
  }


  status =  MPU6050.MPU6050_CheckDeviceID();
  if (status > 0) {
     Serial.print("Wrong MPU6050 device ID:" ); Serial.print(status); Serial.print(" Should be: "); Serial.println(MPU6050_ID,HEX); 
  }
  else {
     Serial.print("MPU6050_CheckDeviceID OK : " );Serial.println(MPU6050_ID,HEX); 
  }
  
  status = MPU6050.MPU6050_calibrate();
  if (status > 0) {
     Serial.print("MPU6050_calibrate error:" ); Serial.println(status); 
  }
  
  double temperature = MPU6050.MPU6050_getTemperature();
  status = MPU6050.MPU6050_getStatus();
  if (status > 0)
  {
       Serial.print("Error MPU6050_getTemperature: ");Serial.println(status);
  }
  else
  {  
     Serial.print("temperature: "); Serial.println(temperature); 
  }

  Serial.println("End Init" ); 

}


void loop()
{
  long starttime = 0;
  double angle[3] = {0.0};
  uint8_t status = 0;
  
  starttime = millis();
  
  status = MPU6050.MPU6050_get_roll_pitch_yaw(angle);
  if (status > 0) {
     Serial.print("MPU6050_get_roll_pitch_yaw error:" ); Serial.println(status); 
  }
  else { 
     Serial.print("roll: ");Serial.println(angle[0]); Serial.print("pitch: ");Serial.println(angle[1]); Serial.print("yaw: ");Serial.println(angle[2]);
  }
  
  Serial.print("elasptime: "); Serial.println(millis() - starttime);

}

