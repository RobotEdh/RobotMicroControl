#include <Wire.h>       // I2C protocol
#include <Servo.h>      // Servo
#include <SD.h>         // SD-Card
#include <sdcard.h>     // SD-Card

#include <robot.h>
#include <motor.h>
#include <SharpIR.h>           // IR sensor
#include <CMPS03.h>            // Compass
#include <TMP102.h>            // Temperature sensor
#include <TiltPan.h>           // Tilt&Pan
#include <LSY201.h>            // Camera
#include <LiquidCrystal_I2C.h> // LCD
#include <Motion.h>            // Motion
#include <TEMT6000.h>          // Lux
#include <Sound.h>             // Sound

#include <IOTSerial.h>         // Serial lib to communicate with IOT
 

 
void setup()
{
  int ret = SUCCESS;
  
  Serial.begin(9600); // initialize serial port
  ret = robot_begin(); 
  if (ret == SUCCESS)  Serial.println("robot begin OK");
  else                 Serial.println("robot begin KO");


 
 }


void loop()
{
  robot_main(); 
}