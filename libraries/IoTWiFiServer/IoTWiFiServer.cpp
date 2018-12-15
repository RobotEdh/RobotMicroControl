
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <IOTSerial.h>
#include "IoTWiFiServer.h"


const char *ssid     = "WIFICOTEAU2";
const char *password = "kitesurf9397";

ESP8266WebServer tcpServer (80);
IOTSerialClass IOTSerial;


// Constructor
IoTWiFiServerClass::IoTWiFiServerClass()
{
    _freqInfos = 0; 
    _cmdId = 0;
    sprintf(infos,"{\"result\": %u, \"AlertStatus\": %u, \"PictureNumber\": %u, \"MotorState\": %u, \"Direction\": %u, \"Distance\": %u, \"Temperature\": %u, \"Brightness\": %u, \"Noise\": %u}",0,0,0,0,0,0,0,0,0);

}


unsigned long IoTWiFiServerClass::IoTWSgetfreqInfos()
{
    return _freqInfos;
}

void IoTWiFiServerClass::IoTWShandleRoot() {
    String command=tcpServer.arg(0);
    
    int ret = IoTWiFiServerClass::IoTWSRobotCmd(command);
    
    if (ret >= 0) {
        tcpServer.send(200, "application/json", infos); 
    }
}

void IoTWiFiServerClass::IoTWShandleClient() {

    tcpServer.handleClient(); 

}

void IoTWiFiServerClass::IoTWShandleNotFound() {


}

void IoTWiFiServerClass::IoTWSbegin()
{
     int ret = SUCCESS;
    
    //Serial.println("Begin IoTWSbegin");
    
    // led on
    pinMode(5, OUTPUT); 
    digitalWrite(5, HIGH);
     
    pinMode(4, OUTPUT);            // sets pin as output,used to interrupt the robot
   
    // We start by connecting to a WiFi network

    //Serial.print("Connecting to ");
    //Serial.println(ssid);
             
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      //Serial.print(".");
    }
 
    //Serial.println("");
    //Serial.println("WiFi connected");  
    //Serial.println("IP address: ");
    //Serial.println(WiFi.localIP());
    
    if ( MDNS.begin ( "esp8266" ) ) {
         // Serial.println ( "MDNS responder started" );
    }

    tcpServer.on ( "/robotCmd/", std::bind(&IoTWiFiServerClass::IoTWShandleRoot, this));
    tcpServer.onNotFound (std::bind(&IoTWiFiServerClass::IoTWShandleNotFound, this));
    tcpServer.begin();
    //Serial.println ( "HTTP server started" );    
    
    ret = IOTSerial.IOTSbegin(0); // Serial port
    
    digitalWrite(4, LOW); // reset interrupt of the Robot
    digitalWrite(5, LOW); // led off
                          
}

int IoTWiFiServerClass::IoTWSRobotInfos ()
{
    uint16_t param[MAX_PARAMS];    
    uint8_t msg[MSG_SIZE_MAX];
    uint8_t msg_len = 0;
    uint8_t tag [MAX_TAGS];
    uint16_t value [MAX_TAGS];
    uint8_t nbtags;  
    int ret = SUCCESS;
    
    //Serial.println("Begin IoTWSRobotInfos");
    
    digitalWrite(5, HIGH); // led on
    
    digitalWrite(4, HIGH); // interrupt the Robot
    _cmdId++;
    IOTSerial.IOTSsend(0, CMD, CMD_INFOS, param, 0, _cmdId); // send the command INFOS to the Robot
    digitalWrite(4, LOW); // reset interrupt of the Robot
    //Serial.print("Call IOTSsend");Serial.print(", _cmdId: "); Serial.println((int)_cmdId);    
 
    ret = IOTSerial.IOTSread(0, msg, &msg_len);
    //Serial.print("Call IOTSread, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);

    if (ret != SUCCESS) {
       //Serial.println("error IOTSread");
       return ret;  
    }
  
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
    if (nbtags < 1)          return -1; 
    
    //Serial.print("tag[0]: ");Serial.println((int)tag[0],HEX); 
    if (tag[0]!= TAG_CMDID)   return -2;
    //Serial.print("value[0]: ");Serial.println((int)value[0]); 
    if (value[0] != _cmdId) { 
        // Serial.print("Bad cmdId, cmdId received: ");Serial.print((int)value[0]);Serial.print(" <> cmdId sent: ");Serial.println((int)cmdId);
        ret =IOTSerial.IOTSflush(0);
        return ret;
    }

    //Serial.print("tag[1]: "); Serial.println((int)tag[1]);  
    if (tag[1]!= TAG_RESP)   return -3;
    //Serial.print("value[1]: "); Serial.println((int)value[1]);        
    if (value[1] == RESP_KO) return -4;
    if (value[1] != RESP_OK) return -5;
        
   /* Serial.print("value[2]: "); Serial.println((int)value[2]);               
    Serial.print("value[3]: "); Serial.println((int)value[3]);               
    Serial.print("value[4]: "); Serial.println((int)value[4]);               
    Serial.print("value[5]: "); Serial.println((int)value[5]);               
    Serial.print("value[6]: "); Serial.println((int)value[6]);               
    Serial.print("value[7]: "); Serial.println((int)value[7]);               
    Serial.print("value[8]: "); Serial.println((int)value[8]);               
    Serial.print("value[9]: "); Serial.println((int)value[9]);               
    Serial.print("value[10]: "); Serial.println((int)value[10]);               
    Serial.print("value[11]: "); Serial.println((int)value[11]);               
   */
     
    // encode in JSON
    sprintf(infos,"{\"result\": %u, \"AlertStatus\": %u, \"PictureNumber\": %u\"MotorState\": %u, \"Direction\": %u, \"Distance\": %u, \"Temperature\": %u, \"Brightness\": %u, \"Noise\": %u}",0,value[2],value[3],value[4],value[5],value[6],value[7],value[8],value[9]);
      
    //Serial.println("End IoTWSRobotInfos");
    
    digitalWrite(5, LOW); // led off

    return SUCCESS;  
}


int IoTWiFiServerClass::IoTWSRobotCmd(String command) {
  int cmdTag = -1;
  int paramTag = -1;
  int Separator = -1;
  int Start = -1;
  int Stop = -1;
  String szcmd;
  String szparam[10];
  int iparam[10];
  int ret = SUCCESS;

  uint8_t cmd;
  uint16_t param[MAX_PARAMS];
  uint8_t paramlen = 0;
  uint8_t msg[MSG_SIZE_MAX];
  uint8_t msg_len = 0;
  uint8_t tag [MAX_TAGS];
  uint16_t value [MAX_TAGS];
  uint8_t nbtags;
  
    
  //Serial.print("Start IoTWSRobotCmd, command :"); Serial.println(command);

  cmdTag = command.indexOf("CMD=");
  if (cmdTag != -1)
  {   
       //Serial.print("CMD=");
       Separator = command.indexOf('|', cmdTag + 1);
       Stop = command.indexOf(">");
       if (Stop == -1) return -1;
       else if (Separator == -1) szcmd = command.substring(cmdTag + 4, Stop);
       else szcmd = command.substring(cmdTag + 4, Separator);
       //Serial.println(szcmd);

       paramTag = command.indexOf("PARAM=");
       if (paramTag != -1)
       {   
          Start = paramTag + 6;
          Separator = command.indexOf('|', Start + 1);
          for (int p=0; (Separator != -1) && (Start < Stop) ; p++)
          {
               szparam[p] = command.substring(Start, Separator);
               iparam[p] = szparam[p].toInt();
               //Serial.print("param"); Serial.print(p); Serial.print("=");
               //Serial.print(szparam[p]); Serial.print("/"); Serial.println(iparam[p]);
               Start = Separator + 1;
               if ( Start < Stop) Separator = command.indexOf('|', Start + 1);              
          }                                   
      }
  
  }


       if (szcmd == "START")
       {
               cmd = CMD_START;
               param[paramlen++] = 0;
       }        
       else if (szcmd == "STOP")
       {
               cmd = CMD_STOP;
        }                                 
       else if (szcmd == "CHECK")
       {
               cmd = CMD_CHECK;
       }                                    
       else if (szcmd == "PICTURE")
       {
               cmd = CMD_PICTURE;
       } 
       else if (szcmd == "ALERT")
       {
               cmd = CMD_ALERT;
       } 
       else if (szcmd == "TURN")
       {
               cmd = CMD_TURN;
               param[paramlen++] = abs(iparam[0]);
               param[paramlen++] = ((iparam[0] != abs(iparam[0])) ? (1):(0));
       }                                                                       
       else if (szcmd == "CHECK_AROUND")
       {
               cmd = CMD_CHECK_AROUND;
       }                                    
       else if (szcmd == "MOVE_TILT_PAN")
       {
               cmd = CMD_MOVE_TILT_PAN;
               param[paramlen++] = abs(iparam[0]);
               param[paramlen++] = ((iparam[0] != abs(iparam[0])) ? (1):(0));
               param[paramlen++] = abs(iparam[1]);
               param[paramlen++] = ((iparam[1] != abs(iparam[1])) ? (1):(0));
       }                                    
       else if (szcmd == "GO")
       {
               cmd = CMD_GO;
               param[paramlen++] = iparam[0];
       } 
       else if (szcmd == "PI")
       {
               cmd = CMD_PI;
               param[paramlen++] = iparam[0];
               param[paramlen++] = iparam[1];
       }        
       else if (szcmd == "INFOS_FREQ")
       {
               _freqInfos = (unsigned long)iparam[0]*1000;
              // Serial.print("new freqInfos: "); Serial.println(freqInfos);
               return SUCCESS;  
       }
       else if (szcmd == "INFOS_GET")
       {
               //Serial.print("call IoTWSRobotInfos");
               ret = IoTWiFiServerClass::IoTWSRobotInfos();
               return SUCCESS;  
       }      
       else if (szcmd == "TEST")
       {
               //Serial.print("Test OK command with param: "); ;  Serial.println((int)iparam[0]);    
               return SUCCESS;
       }
  
    /*Serial.print("cmd: "); Serial.println((int)cmd);               
    if (paramlen > 0) {Serial.print("param[0]: "); Serial.println((int)param[0]);}             
    if (paramlen > 1) {Serial.print("param[1]: "); Serial.println((int)param[1]);}             
    if (paramlen > 2) {Serial.print("param[2]: "); Serial.println((int)param[2]);}
    if (paramlen > 3) {Serial.print("param[3]: "); Serial.println((int)param[3]);}      
  */
  
    digitalWrite(4, HIGH); // interrupt the Robot
    _cmdId++;
    ret = IOTSerial.IOTSsend(0, CMD, cmd, param, paramlen, _cmdId); // send the command to the Robot
    digitalWrite(4, LOW); // reset interrupt of the Robot
    //Serial.print("Call IOTSsend");Serial.print(", paramlen: "); Serial.print((int)paramlen);Serial.print(", cmdId: "); Serial.println((int)_cmdId);
 
    ret = IOTSerial.IOTSread(0, msg, &msg_len); // read the response from the Robot
    //Serial.print("Call IOTSread, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);
    if (ret < 0) return -99;
        
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
    if (nbtags < 1)          return -1; 
    
    //Serial.print("tag[0]: ");Serial.println((int)tag[0],HEX); 
    if (tag[0]!= TAG_CMDID)   return -2;
    //Serial.print("value[0]: ");Serial.println((int)value[0],HEX);     
    if (value[0] != _cmdId) {
        //Serial.print("Bad cmdId, cmdId received: ");Serial.print((int)value[0]);Serial.print(" <> cmdId sent: ");Serial.println((int)_cmdId);
        ret = IOTSerial.IOTSflush(0);
        return ret;
    }

    //Serial.print("tag[1]: ");Serial.println((int)tag[1],HEX); 
    if (tag[1]!= TAG_RESP)   return -3;
    //Serial.print("value[1]: ");Serial.println((int)value[1],HEX);          
    if (value[1] == RESP_KO) return -4;
    if (value[1] != RESP_OK) return -5;
        
    //Serial.print("value[2]: ");Serial.println((int)value[2]);                
    if (nbtags > 2) {
            sprintf(infos,"{\"result\": %u, \"AlertStatus\": %u, \"PictureNumber\": %u, \"MotorState\": %u, \"Direction\": %u, \"Distance\": %u, \"Temperature\": %u, \"Brightness\": %u, \"Noise\": %u}",value[2],0,0,0,0,0,0,0,0);
            return value[2]; 
    }
 
    //Serial.println("End IoTWSRobotCmd, command");

    return SUCCESS;
    
}


