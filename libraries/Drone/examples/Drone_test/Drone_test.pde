#include <Arduino.h> 
#include <Wire.h>       // I2C protocol
#include <Drone.h>

DroneClass  Drone;     

void setup()
{
  Wire.begin(); // initialize I2C
   
  Drone.Drone_init();
}


void loop()
{
  Drone.Drone_main();
}
