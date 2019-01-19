#include <ESP8266WiFi.h>
#include "IoTWiFiServer.h"

IoTWiFiServerClass IoTWiFiServer;

unsigned long previousTime = 0;

  
void setup()
{
    IoTWiFiServer.IoTWSbegin(); 
   
    Serial.println("End setup"); 
 }



void loop() {
    String a;
    a= "CMD=TEST|PARAM=0|10|0|321|5|9876|2345|6789|1789|2789|>";
    
    Serial.print("call IoTWSRobotCmd with msg: ");Serial.println(a);  

    int ret = IoTWiFiServer.IoTWSRobotCmd(a); 
    Serial.print("ret: ");Serial.println(ret); 

    delay (5000);
}

