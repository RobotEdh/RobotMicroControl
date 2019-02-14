#include <IoTWiFiClient.h>

IoTWiFiClientClass IoTWiFiClient;

 
void setup()
{
    int ret = 0; 

    ret = IoTWiFiClient.IoTWCbegin();
    if (ret != 0) exit (0);
    
    IoTWiFiClient.ESPblink(3);  // 3 blinks   
 }


void loop() {

    char infos[1000];
    uint8_t tag;
    int n;
    uint16_t size;
    int ret = 0;

    ret = IoTWiFiClient.IoTWCReceive(infos, &tag, &n, &size);
  
    if (ret == SUCCESS) {
       if (tag == PICTURE) {
          IoTWiFiClient.ESPblink(5);  // 5 blinks
          ret = IoTWiFiClient.IoTWCSendPicture(n, size);
       }
       else if (tag == INFOS) {
          IoTWiFiClient.ESPblink(7);  // 7 blinks
          ret = IoTWiFiClient.IoTWCSendInfos(infos);       
       }
       else if (tag == SLEEP) {
          // sleep until wake up on pin RST
          IoTWiFiClient.ESPblink(12);  // 12 blinks
          IoTWiFiClient.IoTWCend();  // end Serial    
       }
    }      
}