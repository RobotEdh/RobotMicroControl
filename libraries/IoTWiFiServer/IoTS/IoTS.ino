#include <IoTWiFiServer.h>

IoTWiFiServerClass IoTWiFiServer;

unsigned long previousTime = 0;

  
void setup()
{
    int ret = 0;
    
    //Serial.begin(9600);        // open serial ports, used for logging, sets data rate to 9600 bps
    
    //Serial.println("Begin setup"); 
   
    //Serial.println("Call IoTWSbegin");
    ret = IoTWiFiServer.IoTWSbegin(); 
    if (ret != 0) exit (0);
    
    //Serial.println("End setup"); 
 }



void loop() {
    
    IoTWiFiServer.IoTWShandleClient(); 
}