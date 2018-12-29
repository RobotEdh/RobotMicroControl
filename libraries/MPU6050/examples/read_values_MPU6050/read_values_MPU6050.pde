#include <MPU6050.h>
 
/* I2C interface is provided on pins:                                          */
/*         1 = Power +3V                                                       */
/*         2 = SCL                                                             */
/*         3 = SDA                                                             */
/*         9 = Ground                                                          */



MPU6050Class MPU6050;     
      


void setup()
{
  
  Serial.begin(9600); // initialize serial port
  MPU6050.MPU6050_init();
 
  int8_t d = MPU6050.MPU6050_getDeviceID();
  Serial.print("device ID:" );   Serial.println(d); 

  MPU6050.MPU6050_calibrate();
  double temperature = MPU6050.MPU6050_getTemperature();  
  Serial.print("temperature: "); Serial.println(temperature); 

  int8_t  divider = MPU6050.MPU6050_getSampleDivider();
  Serial.print("divider: "); Serial.println(divider); 

  Serial.println("End Init" ); 

}


void loop()
{
  long starttime;
  double angle[3];
  
  starttime = millis();
  
  MPU6050.MPU6050_get_roll_pitch_yaw(angle);
  Serial.print("roll: ");Serial.println(angle[0]); Serial.print("pitch: ");Serial.println(angle[1]); Serial.print("yaw: ");Serial.println(angle[2]);

  Serial.print("elasptime: "); Serial.println(millis() - starttime);

}

