#include <Arduino.h>
#include "MPU6050.h"
 
/* I2C interface is provided on pins:                                       */
/*         1 = Power +5V                                                    */
/*         2 = Ground                                                       */
/*         3 = SCL                                                          */
/*         4 = SDA                                                          */
/*         7 = AD0 +3,3V to force address = 0x69                            */



MPU6050Class MPU6050;     
      


void setup()
{
  uint8_t status = 0;
  double accelZero[3] = {0.0};
  double gyroZero[3] = {0.0};
  
  Serial.begin(9600); // initialize serial port
  Serial.println("Start setup");
 
  
  Wire.begin(); // initialize I2C
  Wire.setClock(400000); //set I2C SCL to High Speed Mode of 400kHz
  
  status = MPU6050.MPU6050_init(); // initialize MPU6050
  if (status > 0)
  {
    Serial.print("Failed to detect and initialize MPU6050, error:");Serial.println(status);
   
  }
  else
  {
    uint8_t address = MPU6050.MPU6050_getAddress();
    Serial.print("Init OK, Address: "); Serial.println(address,HEX);
  }


  status =  MPU6050.MPU6050_checkDeviceID();
  if (status > 0) {
     Serial.print("Wrong MPU6050 device ID:" ); Serial.print(status); Serial.print(" Should be: "); Serial.println(MPU6050_ID,HEX); 
  }
  else {
     Serial.print("MPU6050_checkDeviceID OK, DeviceID: " );Serial.println(MPU6050_ID,HEX); 
  }
  
  status = MPU6050.MPU6050_calibrate();
  if (status > 0) {
     Serial.print("MPU6050_calibrate error:" ); Serial.println(status); 
  }
  else {
     Serial.println("MPU6050_calibrate OK" );
     MPU6050.MPU6050_getZero(accelZero, gyroZero);
     Serial.print("accel_x_zero: ");Serial.println(accelZero[0]); Serial.print("accel_y_zero: ");Serial.println(accelZero[1]); Serial.print("accel_z_zero: ");Serial.println(accelZero[2]);  
     Serial.print("gyro_roll_zero: ");Serial.println(gyroZero[0]); Serial.print("gyro_pitch_zero: ");Serial.println(gyroZero[1]); Serial.print("gyro_yaw_zero: ");Serial.println(gyroZero[2]);  

  }
  delay(5000);
  
  Serial.println("End Init" ); Serial.println("********" ); 

}


void loop()
{
  unsigned long starttime = 0;
  unsigned long stoptime = 0;
  double angle[2] = {0.0};
  double accel[3] = {0.0};
  double gyro[3] = {0.0};
  double temperature = 0;
  
  uint8_t status = 0;
  
  starttime = micros();
  status = MPU6050.MPU6050_compute_angle_gyro(0, angle, gyro); // dt is not managed by this main.
  stoptime = micros();
  
  if (status > 0) {
     Serial.print("MPU6050_compute_angle_gyro error:" ); Serial.println(status); 
  }
  else { 
     Serial.print("angle roll: ");Serial.print(angle[0]); Serial.println(" deg");Serial.print("angle pitch: ");Serial.print(angle[1]);Serial.println(" deg"); 
     Serial.print("gyro_roll: ");Serial.print(gyro[0]);Serial.println(" deg/sec");
     Serial.print("gyro_pitch: ");Serial.print(gyro[1]);Serial.println(" deg/sec");
     Serial.print("gyro_yaw: ");Serial.print(gyro[2]);Serial.println(" deg/sec");
  }
  Serial.print("elasptime: ");Serial.print(stoptime - starttime);Serial.println(" micros");Serial.println(" ");
  
  starttime = micros();
  status = MPU6050.MPU6050_compute_accel_gyro(accel, gyro);
  stoptime = micros();
  if (status > 0) {
     Serial.print("MPU6050_compute_accel_gyro error:" ); Serial.println(status); 
  }
  else { 
     Serial.print("accel_x: ");Serial.print(accel[0]);Serial.println(" g");
     Serial.print("accel_y: ");Serial.print(accel[1]);Serial.println(" g");
     Serial.print("accel_z: ");Serial.print(accel[2]);Serial.println(" g");  
     Serial.print("gyro_roll: ");Serial.print(gyro[0]);Serial.println(" deg/sec");
     Serial.print("gyro_pitch: ");Serial.print(gyro[1]);Serial.println(" deg/sec");
     Serial.print("gyro_yaw: ");Serial.print(gyro[2]);Serial.println(" deg/sec");  
  }
   Serial.print("elasptime: ");Serial.print(stoptime - starttime);Serial.println(" micros");Serial.println(" ");

  temperature = MPU6050.MPU6050_getTemperature(); 
  if (temperature > 0) {
     Serial.print("MPU6050_getTemperature, temperature:" ); Serial.println(temperature); Serial.println(" ");
  }
  else { 
         Serial.println("MPU6050_getTemperature, no temperature read" ); 
 }

 delay(6000);
}