#include <Wire.h>       // I2C protocol for Temperature sensor

#include <TMP102.h>
 
/* I2C interface is provided on pins:                                          */
/*         1 = Power +3V                                                       */
/*         2 = SCL                                                             */
/*         3 = SDA                                                             */
/*         9 = Ground                                                          */



TMP102Class TMP102;     /* The Temperature sensor class */
double temperature;     /* temperature */

void setup()
{
  
  Serial.begin(9600); // initialize serial port
   
   Wire.begin(); // initialize I2C
  
}


void loop()
{
  int  value;
  long startTime, stopTime, elapsedTime;
  
  Serial.println(" --> read temperature in Celsius from TMP102"); 
  
  startTime = micros();
  
  temperature = TMP102.TMP102_read();  

  stopTime = micros();
  elapsedTime = stopTime - startTime; // take 5 us
  
  Serial.print("temperature: ");
  Serial.println(temperature); 
  Serial.print(" - elapsedTime: ");
  Serial.println(elapsedTime); 
 
  delay(1000); //make it readable

}

