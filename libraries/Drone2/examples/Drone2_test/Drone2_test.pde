#include <Arduino.h> 
#include <Drone2.h>

Drone2Class  Drone;     

void setup()
{
  Drone.Drone_init();
}


void loop()
{
  Drone.Drone_main();
}
