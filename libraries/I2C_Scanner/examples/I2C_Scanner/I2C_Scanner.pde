 
#include <I2C_Scanner.h>
 
 
I2C_ScannerClass I2C_Scanner;
 
void setup()
{
  Serial.begin(9600);
  
  I2C_Scanner.I2C_Scanner_init(); 
  
  Serial.println("Init OK");
}
 
 
void loop()
{
  I2C_Scanner.I2C_Scanner_scan(); 
  
   delay(5000);           // wait 5 seconds for next scan
}
