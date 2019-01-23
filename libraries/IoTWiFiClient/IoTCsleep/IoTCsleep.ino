#include <ESP8266WiFi.h>
#include <IOTSerial.h>
#include "IoTWiFiClient.h"

IoTWiFiClientClass IoTWiFiClient;

  
void setup()
{
    char infos[1000];
    uint8_t tag;
    int n;
    uint16_t size;
    int ret = SUCCESS; 
     
    //Serial.begin(9600);        // open serial ports, used for logging, sets data rate to 9600 bps
   
    //Serial.println("Call IoTWCbegin");
    ret = IoTWiFiClient.IoTWCbegin();
   
    //Serial.println("End setup"); 
    
    ret = IoTWiFiClient.IoTWCReceive(infos, &tag, &n, &size);
    //Serial.print("ret: ");  Serial.println(ret); 
  
    if (ret == SUCCESS) {
        //Serial.print("tag: "); Serial.println((int)tag); 
        //Serial.print("n: "); Serial.println(n);
        
        if (tag == PICTURE) {
            ret = IoTWiFiClient.IoTWCSendPicture(n, size);
        }
        else if (tag == INFOS) {
            ret = IoTWiFiClient.IoTWCSendInfos(infos);       
       }  
    } 
      
    ESP.deepSleep(0,WAKE_RF_DEFAULT); // sleep until wake up on pin RST 
 }


void loop() {
   
}