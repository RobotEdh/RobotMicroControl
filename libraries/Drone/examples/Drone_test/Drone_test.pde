#include <MPU6050.h>
#include <RC.h>
#include <MotorESC.h>
#include <Drone.h>

DroneClass  Drone;     

void setup()
{
  Serial.begin(9600); // initialize serial port
  
  Serial.println("Start Init" );  
  
  Drone.Drone_init();
 
  Serial.println("End Init" ); 
}


void loop()
{
  Drone.Drone_main();
}
