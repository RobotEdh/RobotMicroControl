#include <ESP8266WiFi.h>
#include "IoTWiFiServer.h"

IoTWiFiServerClass IoTWiFiServer;

  
void setup()
{
    IoTWiFiServer.IoTWSbegin(); 
   
}



void loop() {
    String a;
    a= "CMD=TEST|PARAM=0|10|0|321|5|9876|2345|6789|1789|2789|>";
    
    Serial.print("call");

    int ret = IoTWiFiServer.IoTWSRobotCmd(a); 
    Serial.print("ret: ");Serial.println(ret); 

    delay (70*1000);
}
