#include <Sound.h>

SoundClass Sound;

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Sound.Sound_init(); // initialize the default pin A3 connected to the sensor
 
}


void loop()
{
  int  value;
  long startTime, stopTime, elapsedTime;
  
  Serial.print(" --> read noise value of the Sound Detector: "); 
  
  startTime = micros();
  value = Sound.Sound_getNoise();
  stopTime = micros();
  elapsedTime = stopTime - startTime;
  
  Serial.print("value between 0 (no noise) and 1023 (huge noise): ");
  Serial.print(value); 
  Serial.print(" - elapsedTime: ");
  Serial.println(elapsedTime); 
 
  delay(1000); //make it readable
  
}