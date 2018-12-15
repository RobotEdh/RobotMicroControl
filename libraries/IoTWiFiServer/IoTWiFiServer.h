#ifndef IoTWiFiServer_h
#define IoTWiFiServer_h

#define SUCCESS              0

class IoTWiFiServerClass
{
 public:
    
  IoTWiFiServerClass();
  
  void IoTWSbegin (void);
  /* Sets pin D4 as output used to interrupt the robot  */
  /* Connect to SSID WIFICOTEAU2                        */
  /* Start HTTP server: /robotCmd/                      */
  /* Init IOTSerial                                     */
  
   void IoTWShandleRoot(void);
  /* call IoTWSRobotCmd(command)                        */
  /* tcpServer.send(200, "application/json", infos)     */
  
  void IoTWShandleClient(void);
  /* handleClient                                       */
  
  void IoTWShandleNotFound(void);
  /* Do nothing                                          */
  
  unsigned long IoTWSgetfreqInfos(void);     
  /* return freqInfos                                   */
  
  
  int  IoTWSRobotInfos (void);
  
  int  IoTWSRobotCmd(String command);
  /* interrupt the Robot                                        */
  /* send the command to the Robot using IOTSerial.IOTSsend     */
  /* read the response from the Robot using IOTSerial.IOTSread  */
  /* get tags using IOTSerial.IOTSgetTags                       */
 
 private:
   unsigned long _freqInfos; 
   uint8_t _cmdId;
   char infos [700];
   char result [700];
   
};

#endif
