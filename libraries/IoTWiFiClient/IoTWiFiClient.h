#ifndef IoTWiFiClient_h
#define IoTWiFiClient_h

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <IOTSerial.h>
#include <ESPcredentials.h>

#define SUCCESS              0

class IoTWiFiClientClass
{
  public:
  void ESPblink(int n);
  /* Blink the led                                              */  
    
  int IoTWCbegin (void);
  /* Connect to SSID                                            */
  /* Init IOTSerial                                             */
 
  void IoTWCend (void);
  /* End IOTSerial                                              */ 
   
  int IoTWCSendPicture (int n, uint16_t size); 
  /* call IoTWCSendInfos                                        */
  /* call 3 times IoTWCSendFastPicture                          */
  
  int IoTWCSendInfos (char *infos);
  /* connect to the server using WIFI                           */
  /* send the infos                                             */
  
  int IoTWCReceive (char *infos, uint8_t *type, int *n, uint16_t *size);
  /* read the messages from the Robot using IOTSerial.IOTSread  */
  /* get tags& values using IOTSerial.IOTSgetTags               */
  /* type = PICTURE or INFOS                                    */
  /* n = picture number in case of tag = PICTURE                */
  /* size = size of the picture in case of tag = PICTURE        */
};

#endif
