
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <IOTSerial.h>
#include "IoTWiFiServer.h"


const char *ssid     = "WIFICOTEAU";
const char *password = "kitesurf9397";

ESP8266WebServer tcpServer (80);
IOTSerialClass IOTSerial;


// Constructor
IoTWiFiServerClass::IoTWiFiServerClass()
{
    _cmdId = 0;
    
    _AlertStatus = 0;
    _PictureNumber = 0;
    _MotorState = 0;
    _Direction = 0;
    _ObstacleStatus = 0;
    _Distance = 0;
    _Temperature = 0;
    _Humidity = 0;
    _Brightness = 0;
    _Noise = 0;      
}


void IoTWiFiServerClass::IoTWShandleRoot() {
    String command=tcpServer.arg(0);
    
    int ret = IoTWiFiServerClass::IoTWSRobotCmd(command);
    
    sprintf(infos,"{\"result\": %d, \"AlertStatus\": %u, \"PictureNumber\": %u, \"MotorState\": %u, \"Direction\": %u, \"ObstacleStatus\": %u, \"Distance\": %u, \"Temperature\": %u, \"Humidity\": %u, \"Brightness\": %u, \"Noise\": %u}",_result,_AlertStatus,_PictureNumber,_MotorState,_Direction,_ObstacleStatus,_Distance,_Temperature,_Humidity,_Brightness,_Noise);
    tcpServer.sendHeader("Access-Control-Allow-Origin", "*");
    tcpServer.send(200, "application/json", infos);
   
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
     
    // MAC address 
    byte mac[6]; 
    WiFi.macAddress(mac);
    /*Serial.print("MAC: ");
    Serial.print(mac[0],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.println(mac[5],HEX);
    */
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
    _result = 0;
    
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
    }        
    else if (szcmd == "STOP")
    {
           cmd = CMD_STOP;
    }                                 
    else if (szcmd == "CHECK")
    {
           cmd = CMD_CHECK;
           param[paramlen++] = iparam[0];              
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
    else if (szcmd == "GET_INFOS")
    {
            cmd = CMD_GET_INFOS; 
    }      
    else if (szcmd == "TEST")
    {
           //Serial.print("Test OK command with param: "); ;  Serial.println((int)iparam[0]);    
           return SUCCESS;
    }
  

    digitalWrite(4, HIGH); // interrupt the Robot
    _cmdId++;
    ret = IOTSerial.IOTSsend(0, CMD, cmd, param, paramlen, _cmdId); // send the command to the Robot
    digitalWrite(4, LOW); // reset interrupt of the Robot
    //Serial.print("Call IOTSsend");Serial.print(", paramlen: "); Serial.print((int)paramlen);Serial.print(", cmdId: "); Serial.println((int)_cmdId);
 
    ret = IOTSerial.IOTSread(0, msg, &msg_len); // read the response from the Robot
    //Serial.print("Call IOTSread, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);
    if (ret < 0) {_result = -99; return _result;}
        
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
    if (nbtags < 1)          {_result = -1; return _result;}
    
    //Serial.print("tag[0]: ");Serial.println((int)tag[0],HEX); 
    if (tag[0]!= TAG_CMDID)  {_result = -2; return _result;}
    //Serial.print("value[0]: ");Serial.println((int)value[0],HEX);     
    if (value[0] != _cmdId)  {_cmdId = value[0]; _result = -3; return _result;} // align Cmdid for next command

    //Serial.print("tag[1]: ");Serial.println((int)tag[1],HEX); 
    if (tag[1]!= TAG_RESP)   {_result = -4; return _result;}
    //Serial.print("value[1]: ");Serial.println((int)value[1],HEX);          
    if (value[1] == RESP_KO) {_result = -5; return _result;}
    if (value[1] != RESP_OK) {_result = -6; return _result;}
    
    if (szcmd == "START")
    {
           _MotorState = value[2];
           return SUCCESS;
    }        
    else if (szcmd == "STOP")
    {
           _MotorState = value[2];        
           return SUCCESS;          
    }                                 
    else if (szcmd == "CHECK")
    {
           _AlertStatus = value[2]; 
           return SUCCESS;              
    }                                    
    else if (szcmd == "PICTURE")
    {
           _PictureNumber = value[2]; 
           return SUCCESS;  
    } 
    else if (szcmd == "ALERT")
    {
           _PictureNumber = value[2]; 
           return SUCCESS; 
    } 
    else if (szcmd == "TURN")
    {
           return SUCCESS; ;
    }                                                                       
    else if (szcmd == "CHECK_AROUND")
    {
           _ObstacleStatus = value[2]; 
           return SUCCESS;  
    }                                    
    else if (szcmd == "MOVE_TILT_PAN")
    {
           return SUCCESS; 
    }                                    
    else if (szcmd == "GO")
    {
           _AlertStatus = value[2];
           _PictureNumber = value[3];
           _MotorState = value[4];
           _Direction = value[5];
           _ObstacleStatus = value[6];           
           _Distance = value[7];
           _Temperature = value[8];
           _Humidity = value[9];  
           _Brightness = value[10];
           _Noise = value[11];                                              
           return SUCCESS; 
    } 
    else if (szcmd == "PI")
    {
           return SUCCESS; 
    }        
    else if (szcmd == "GET_INFOS")
    {
           _AlertStatus = value[2];
           _PictureNumber = value[3];
           _MotorState = value[4];
           _Direction = value[5];
           _ObstacleStatus = value[6];           
           _Distance = value[7];
           _Temperature = value[8];
           _Humidity = value[9];  
           _Brightness = value[10];
           _Noise = value[11];        
           return SUCCESS; 
    }      

  
}

