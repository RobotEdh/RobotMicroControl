#include <Motion.h>

MotionClass Motion;

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Motion.Motion_init(); // initialize the default pin 28 connected to the sensor
 
}


void loop()
{
  int  status;
  long startTime, elapsedTime;
  
  Serial.print(" --> get status of motion sensor "); 
  
  startTime = micros();
  status = Motion.Motion_status();
  elapsedTime = micros() - startTime; // take 8 us
  
  Serial.print("0 (no motion) / 1 (motion) status: ");
  Serial.print(status); 
  Serial.print(" - elapsedTime: ");
  Serial.println(elapsedTime); 
 
  delay(1000); //make it readable
  
}