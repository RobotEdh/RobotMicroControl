#include <ESP8266WiFi.h>
#include "IoTWiFiServer.h"

IoTWiFiServerClass IoTWiFiServer;

unsigned long previousTime = 0;

  
void setup()
{

    //Serial.begin(9600);        // open serial ports, used for logging, sets data rate to 9600 bps
    
    //Serial.println("Begin setup"); 
   
    //Serial.println("Call IoTWSbegin");
    IoTWiFiServer.IoTWSbegin(); 
   
    //Serial.println("End setup"); 
 }



void loop() {
    
    IoTWiFiServer.IoTWShandleClient(); 
}
