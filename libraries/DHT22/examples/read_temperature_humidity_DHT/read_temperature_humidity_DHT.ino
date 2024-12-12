#include <DHT22.h>

// Data wire is plugged into port 38 on the Arduino
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
#define DHT22_PIN 7

// Setup a DHT22 instance
DHT22Class DHT22;

void setup(void)
{
  // start serial port
  Serial.begin(9600);
   
  DHT22.DHT22_init(DHT22_PIN);
  
  Serial.println("Init OK");
}

void loop(void)
{ 
  Serial.print("Requesting data...");
  
  DHT22_ERROR_t errorCode = DHT22.readData();
  if (errorCode == DHT_ERROR_NONE)
  {
      Serial.print(DHT22.getTemperatureC());
      Serial.print("C ");
      Serial.print(DHT22.getHumidity());
      Serial.println("%");  
  }
  else
  {
        Serial.println( szDHT_errors[errorCode]);
  }
  
  delay(10000);
}
