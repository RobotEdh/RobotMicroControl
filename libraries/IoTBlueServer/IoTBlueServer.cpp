#include <IoTBlueServer.h>

// create ble serial instance, see pinouts above
BLESerial BLESerial(BLE_REQ, BLE_RDY, BLE_RST);
IOTSerialClass IOTSerial;


// Constructor
IoTBlueServerClass::IoTBlueServerClass()
{
    _cmdId = 0;
    _result = 0;
    
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



int IoTBlueServerClass::IoTBSbegin()
{
   const char * localName = "Robot";
   int ret = SUCCESS;
      
  // led on
  pinMode(LED_PIN, OUTPUT); 
  digitalWrite(LED_PIN, LOW);
     
  pinMode(INTERRUPT_PIN, OUTPUT);            // sets pin as output,used to interrupt the robot
  
  // custom services and characteristics can be added as well
  BLESerial.setDeviceName(localName);
  BLESerial.setLocalName(localName);
   
  BLESerial.begin();
    
  ret = IOTSerial.IOTSbegin(0); // Serial port
  if (ret != 0) {
       //Serial.print ("Error IOTSbegin: ");Serial.println (ret);
       return ret;
  }
  ret = IOTSerial.IOTSflush(0); // start clean
  if (ret != 0) {
       //Serial.print ("Error IOTSflush: ");Serial.println (ret);
       return ret;
  } 
       
  digitalWrite(INTERRUPT_PIN, LOW); // reset interrupt of the Robot
  digitalWrite(LED_PIN, HIGH); // led off
  
  return SUCCESS;
}


int IoTBlueServerClass::IoTBSmain()
{
    uint8_t msg[BLE_ATTRIBUTE_MAX_VALUE_LENGTH+1]={0};
    uint8_t msg_len = 0;
    int ret = SUCCESS;
    
    BLESerial.poll();
    
    ret = IoTBSReadMsg(msg, &msg_len, 60000UL); // read the response from the Robot, timeout 60s
    
    if (ret < 0  ) {
       //Serial.println("error: "); Serial.println(ret); 
       return -1;
    }
    
    String cmd = (String)(char*)msg;
    ret = IoTBSRobotCmd(cmd);
    
    if (ret < 0) {
       //Serial.print("[error] "); 
       return -2;
    }
    
    sprintf(infos,"{\"result\": %d, \"AlertStatus\": %u, \"PictureNumber\": %u, \"MotorState\": %u, \"Direction\": %u, \"ObstacleStatus\": %u, \"Distance\": %u, \"Temperature\": %u, \"Humidity\": %u, \"Brightness\": %u, \"Noise\": %u}",_result,_AlertStatus,_PictureNumber,_MotorState,_Direction,_ObstacleStatus,_Distance,_Temperature,_Humidity,_Brightness,_Noise);
    BLESerial.print("ok");

    return SUCCESS;
}


int IoTBlueServerClass::IoTBSReadMsg(uint8_t *msg, uint8_t *msglen, unsigned long timeout)
{
	int     byteread = 0;
    int     ibuf = -9;
    uint8_t len = 0;
	uint8_t istop = 0;
	uint8_t i = 0;
	
    unsigned long previousTime;
    unsigned long previousTime0=millis();
    
	*msglen = 0;
	
	//Get the response from the IOT and add it to the response string
	while ((istop == 0) && (i < BLE_ATTRIBUTE_MAX_VALUE_LENGTH) && ( millis()< previousTime0 + timeout))
	{
        ibuf = -9;
        previousTime = millis();
        	    
  		while((byteread == 0) && ( millis()< previousTime + timeout)) // waiting for data until 60 seconds in the serial buffer
        {
            byteread = BLESerial.available();
              
        }
 
		if (byteread>0) {
		     ibuf = BLESerial.read();
             //Serial.print(" - byteread: ");Serial.print(byteread);Serial.print(" - ibuf: 0x"); Serial.print(ibuf,HEX); Serial.print("/"); Serial.println((isalnum(ibuf))?((char)ibuf):(' '));
             byteread--;  // one byte read
             msg[i++] = (uint8_t)ibuf; // fill message
             if((uint8_t) ibuf == 0x3E) istop = 1;  // stop when > received
        }
        //else {Serial.print("byteread: ");Serial.println(byteread);}       
         
  
	}  // end while
	    
    if (i == 0) return ERROR_SERIAL_NO_MSG;
    if (i == BLE_ATTRIBUTE_MAX_VALUE_LENGTH) return ERROR_SERIAL_MSG_SIZE_MAX;
    
    *msglen = i;
	return SUCCESS;
}


int IoTBlueServerClass::IoTBSRobotCmd(String command) {
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
  uint8_t msg[BLE_ATTRIBUTE_MAX_VALUE_LENGTH];
  uint8_t msg_len = 0;
  uint8_t tag [MAX_TAGS];
  uint16_t value [MAX_TAGS];
  uint8_t nbtags;
  
   //Serial.print("Start IoTBSRobotCmd, command :"); Serial.println(command);
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

       paramTag = command.indexOf("P=");
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
    else if (szcmd == "M")
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
           cmd = CMD_TEST;
           param[paramlen++] = iparam[0];
           param[paramlen++] = iparam[1];
           param[paramlen++] = iparam[2];
           param[paramlen++] = iparam[3];
           param[paramlen++] = iparam[4];
           param[paramlen++] = iparam[5];
           param[paramlen++] = iparam[6];
           param[paramlen++] = iparam[7]; 
           param[paramlen++] = iparam[8];
           param[paramlen++] = iparam[9];                                            
    }
    
    ret = IOTSerial.IOTSflush(0); // clean before serial com 

    digitalWrite(LED_PIN, LOW); // led on
    digitalWrite(INTERRUPT_PIN, HIGH); // interrupt the Robot
    _cmdId++;
    ret = IOTSerial.IOTSsend(0, CMD, cmd, param, paramlen, _cmdId); // send the command to the Robot
    digitalWrite(INTERRUPT_PIN, LOW); // reset interrupt of the Robot
    //Serial.print("Call IOTSsend");Serial.print(", paramlen: "); Serial.print((int)paramlen);Serial.print(", cmdId: "); Serial.println((int)_cmdId);
    delay(100);
    ret = IOTSerial.IOTSread(0, msg, &msg_len, 60000UL); // read the response from the Robot, timeout 60s
    //Serial.print("Call IOTSread, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);
    digitalWrite(LED_PIN, HIGH); // led off 
    if (ret < 0) {_AlertStatus = (uint16_t)msg_len; _result = ret; return _result;}
     
    IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
    
    //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);  
    if (nbtags < 1)          {_AlertStatus = (uint16_t)nbtags;_result = -1; return _result;} 
    //Serial.print("tag[0]: ");Serial.println((int)tag[0],HEX); 
    if (tag[0]!= TAG_CMDID)  {_AlertStatus = (uint16_t)tag[0]; _result = -2; return _result;}
    //Serial.print("tag[1]: ");Serial.println((int)tag[1],HEX); 
    if (tag[1]!= TAG_RESP)   {_AlertStatus = (uint16_t)tag[1]; _result = -3; return _result;}
    //Serial.print("value[1]: ");Serial.println((int)value[1],HEX);          
    if (value[1] == RESP_KO) {_AlertStatus = value[1]; _result = -4; return _result;}
    if (value[1] != RESP_OK) {_AlertStatus = value[1]; _result = -5; return _result;}
    //Serial.print("value[0]: ");Serial.println((int)value[0],HEX);     
    if ((uint8_t)value[0] != _cmdId)  {_AlertStatus = value[0]; _PictureNumber = (uint16_t)_cmdId; _result = -6; return _result;}   
         
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
           _AlertStatus =    value[2];
           _PictureNumber =  value[3];
           _MotorState =     value[4];
           _Direction =      value[5];
           _ObstacleStatus = value[6];           
           _Distance =       value[7];
           _Temperature =    value[8];
           _Humidity =       value[9];  
           _Brightness =     value[10];
           _Noise =          value[11]; 
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
           _AlertStatus =    value[2];
           _PictureNumber =  value[3];
           _MotorState =     value[4];
           _Direction =      value[5];
           _ObstacleStatus = value[6];           
           _Distance =       value[7];
           _Temperature =    value[8];
           _Humidity =       value[9];  
           _Brightness =     value[10];
           _Noise =          value[11];                                              
           return SUCCESS; 
    } 
    else if (szcmd == "PI")
    {
           return SUCCESS; 
    }        
    else if (szcmd == "GET_INFOS")
    {
           _AlertStatus =    value[2];
           _PictureNumber =  value[3];
           _MotorState =     value[4];
           _Direction =      value[5];
           _ObstacleStatus = value[6];           
           _Distance =       value[7];
           _Temperature =    value[8];
           _Humidity =       value[9];  
           _Brightness =     value[10];
           _Noise =          value[11];       
           return SUCCESS; 
    }      
    else if (szcmd == "TEST")
    {
           _AlertStatus =    value[2];
           _PictureNumber =  value[3];
           _MotorState =     value[4];
           _Direction =      value[5];
           _ObstacleStatus = value[6];           
           _Distance =       value[7];
           _Temperature =    value[8];
           _Humidity =       value[9];  
           _Brightness =     value[10];
           _Noise =          value[11];        
           return SUCCESS; 
    }
    return -10;  // unknow command
}