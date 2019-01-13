#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;


void setup()
{
  uint8_t status = 0;
  uint8_t calib = 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C
  
  Serial.println("Init CMPS12");
  calib = CMPS12.CMPS12_init(false); // Do not calibration
  
  status = CMPS12.CMPS12_getStatus();
  if (status == 0)  
  {
     Serial.println(" ");
     Serial.println("Erase profile CMPS12");
     status = CMPS12.CMPS12_eraseProfil();
     if (status == 0)
     {
        Serial.println("Erase profile OK");   
     }
     else 
     {           
        Serial.print("Erase profile KO, I2C error: ");Serial.println(status);  
     }             
  }
  else 
  {           
     Serial.print("Init KO, I2C error: ");Serial.println(status);  
  }       
          
}

void loop()
{

}