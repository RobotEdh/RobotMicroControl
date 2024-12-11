#include <Arduino.h> 
#include <Drone3.h>

Drone3Class  Drone;     

void setup()
{
  Drone.Drone_init();
}


void loop()
{
  Drone.Drone_main();
}
