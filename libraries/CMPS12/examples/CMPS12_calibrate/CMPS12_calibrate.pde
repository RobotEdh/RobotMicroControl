#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;


void setup()
{
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  uint8_t status  = CMPS12.CMPS12_init();
  
  if (status == 0)
 {
   void CMPS12.CMPS12_writeReg(CMD_REVISION, 0xF0,);
 } 
  
  
}

void loop()
{

}
