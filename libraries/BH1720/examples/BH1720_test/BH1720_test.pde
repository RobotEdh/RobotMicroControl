#include <Arduino.h>
#include <BH1720.h>

BH1720Class BH1720;


void setup()
{
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  uint8_t status = BH1720.BH1720_init(); // initialize BH1720
  
  if (status == 0)
  {
       Serial.println("Init OK");
  }
  else 
  {           
        Serial.print("Init KO, I2C error: ");Serial.println(status);  
  }    
}

void loop()
{
    uint8_t status = 0;
    
    double lux = BH1720.BH1720_getLux();
    status = BH1720.BH1720_getStatus();
    if (status > 0)
    {
       Serial.print("Error BH1720_getLux: ");Serial.println(status);
    }
    else
    {
       Serial.print("Lux: ");Serial.println(lux);
    }
    
    delay(5000);
}