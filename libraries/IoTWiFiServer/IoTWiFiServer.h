#ifndef IoTWiFiServer_h
#define IoTWiFiServer_h

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <IOTSerial.h>
#include <ESPcredentials.h>

#define SUCCESS              0

class IoTWiFiServerClass
{
 public:
    
  IoTWiFiServerClass();
  
  int IoTWSbegin (void);
  /* Sets pin D4 as output used to interrupt the robot  */
  /* Connect to SSID                                    */
  /* Start HTTP server: /robotCmd/                      */
  /* Init IOTSerial                                     */
  
   void IoTWShandleRoot(void);
  /* call IoTWSRobotCmd(command)                        */
  /* tcpServer.send(200, "application/json", infos)     */
  
  void IoTWShandleClient(void);
  /* handleClient                                       */
  
  void IoTWShandleNotFound(void);
  /* Do nothing                                          */
  
  int  IoTWSRobotCmd(String command);
  /* interrupt the Robot                                        */
  /* send the command to the Robot using IOTSerial.IOTSsend     */
  /* read the response from the Robot using IOTSerial.IOTSread  */
  /* get tags using IOTSerial.IOTSgetTags                       */
 
 private:
   uint8_t _cmdId;
   int16_t  _result;
   uint16_t  _AlertStatus,_PictureNumber,_MotorState,_Direction,_ObstacleStatus,_Distance,_Temperature,_Humidity,_Brightness,_Noise;
   char infos [700];
   
};

#endif