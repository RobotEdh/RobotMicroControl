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
    unsigned long freqInfos = 0;
    unsigned long currentTime = millis();
    
    int ret = 0;

    
    freqInfos = IoTWiFiServer.IoTWSgetfreqInfos(); 

    
    IoTWiFiServer.IoTWShandleClient(); 
  
    if ((freqInfos > 0) && (currentTime > previousTime + freqInfos)) {
        previousTime = currentTime;
        //Serial.print("freqInfos: "); Serial.println(freqInfos); 
        
        ret = IoTWiFiServer.IoTWSRobotInfos();
        //Serial.print("Call IoTWSRobotInfos, ret: "); Serial.println(ret); 
    }  
   
}
