#include <Arduino.h> 
#include <Drone3.h>

Drone3Class  Drone;     

void setup()
{
  uint8_t status = Drone.Drone_init();
  if (status > 0)
  { 
     Serial.print("Error Drone_init, status :");Serial.println(status);
     while (1) {}
  }
  Serial.println("Drone_init OK");
}


void loop()
{
  Drone.Drone_main();
}
