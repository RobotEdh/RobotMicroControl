#include <Arduino.h> 
#include <Wire.h>       // I2C protocol

#include <robot.h>

void setup()
{
  Wire.begin(); // initialize I2C
   
  int ret = robot_begin(); 
}


void loop()
{
  robot_main(); 
}