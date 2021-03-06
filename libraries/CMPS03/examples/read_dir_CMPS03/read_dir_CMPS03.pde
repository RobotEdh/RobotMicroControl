#include <Wire.h>       // I2C protocol for Compass

#include <CMPS03.h>
 
/* I2C interface is provided on pins:                                          */
/*         1 = Power +5V                                                       */
/*         2 = SCL1 in standard but SCL2 on MAX32 following change made by EDH */
/*         3 = SDA  in standard but SDA2 on MAX32 following change made by EDH */
/*         9 = Ground  */



CMPS03Class CMPS03;     /* The Compass class */
int revision;  /* revison number: 16 during testing this lib */
int direction; /* direction between 0-254, 0: North */

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C
  
  revision = CMPS03.CMPS03_revision();
 
  Serial.print("Revision: ");
  Serial.println(revision);
  
  delay(1000); //make it readable 
}


void loop()
{

  
  Serial.println(" --> read direction from CMPS03"); 

  direction = CMPS03.CMPS03_read();
 
  Serial.print("Direction: ");
  Serial.println(direction); 
 
  delay(1000); //make it readable
  
  return;
}

