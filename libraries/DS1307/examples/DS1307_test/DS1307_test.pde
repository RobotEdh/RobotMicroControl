#include <Arduino.h>
#include <DS1307.h>

DS1307Class DS1307;


void setup()
{
  uint8_t status = 0;
  
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  status = DS1307.DS1307_init();
  if (status == ERROR_RTC_STOPPED)
  {
    Serial.println("RTC stopped");
  }  
  else if (status > 0)
  {   
      Serial.print("Failed to init DS1307, error:"); Serial.println(status);   
  }
  else
  {
    uint8_t address = DS1307.DS1307_getAddress();
    Serial.print("Init OK, Address: 0x"); Serial.println(address,HEX);
  }

}

void loop()
{ 
    DateTime_t now;
    uint8_t status = 0;
    
    status = DS1307.DS1307_read_current_datetime(&now);
    if (status > 0)
    {
       Serial.print("Error DS1307_read_current_datetime: ");Serial.println(status);
    }
    else
    {
       Serial.print("Date : ");
       Serial.print(now.days);
       Serial.print("/");
       Serial.print(now.months);
       Serial.print("/");
       Serial.print(now.year + 2000);
       Serial.print("  Heure : ");
       Serial.print(now.hours);
       Serial.print(":");
       Serial.print(now.minutes);
       Serial.print(":");
       Serial.println(now.seconds);
    }    

    delay(1000);
}
