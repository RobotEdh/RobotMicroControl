#include <Arduino.h> 
#include <Wire.h>       // I2C protocol

#include <robot.h>

 

 
void setup()
{
  int ret = SUCCESS;
  
  Serial.begin(9600); // initialize serial port
   
  Wire.begin(); // initialize I2C
   
  ret = robot_begin(); 
  if (ret == SUCCESS)  Serial.println("robot begin OK");
  else                 Serial.println("robot begin KO");


 
 }


void loop()
{
  robot_main(); 
}