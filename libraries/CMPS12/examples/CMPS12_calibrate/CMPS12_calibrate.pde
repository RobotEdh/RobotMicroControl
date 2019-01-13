#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;


void setup()
{
  uint8_t status = 0;
  uint8_t calib = 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C
  
  
  Serial.println("Start init");
  calib = CMPS12.CMPS12_init(true); // Do calibration
  if (calib == 0)
  {
     CMPS12.CMPS12_storeProfil();
   
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        Serial.println("Store profile OK");   
     }
     else 
     {           
        Serial.print("Store profile KO, I2C error: ");Serial.println(status);  
     }             
  }
  else 
  {           
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        Serial.print("Calibrate KO, calibrate status: 0b");Serial.println(calib,BIN);    
     }
     else 
     {           
        Serial.print("Init KO, I2C error: ");Serial.println(status);  
     } 
  }       
  Serial.println("End init");
    
  Serial.println(" ");
  Serial.println("Check Calibrate");
  calib = CMPS12.CMPS12_checkCalibrate();
  
  status = CMPS12.CMPS12_getStatus();     
  if (status == 0)
  {           
     if (calib == 0)
     {
        Serial.println("Calibrate OK");    
     }
     else 
     {           
        Serial.print("Calibrate KO, calibrate status: 0b");Serial.println(calib,BIN); 
     } 
  }
  else 
  {           
     Serial.print("Check calibrate KO, I2C error: ");Serial.println(status);   
  }  
          
}

void loop()
{

}