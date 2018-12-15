/*
  IOTSerial.cpp - Library for commmunicating in serial mode with IOT 
  Created by EDH, May 12, 2018.
  Released into the public domain.
  Use SoftwareSerial at 38400 bauds
*/
#if defined(ESP8266)  
  #define Serial1 Serial 
  #define Serial2 Serial 
#endif 

#include <IOTSerial.h>

     
// Constructor
IOTSerialClass::IOTSerialClass()
{
}

// Initialize the Serial2 port
int IOTSerialClass::IOTSbegin(int snum)
{
	// baud rate is 38400
	switch (snum) {
    case 0:
      Serial.begin(38400);
      break;
    case 1:
      Serial1.begin(38400);
      break;          
    case 2:
      Serial2.begin(38400);
      break;
    default: 
      return -1;
      break;      
   }

   return SUCCESS;
	
}

void IOTSerialClass::IOTSgetTags(uint8_t *buf, uint8_t *tag, uint16_t *value, uint8_t *nbtags)
{
   uint8_t i=0;
   uint8_t p=0;
   
   while (p < MAX_TAGS)
   {
       value[p] = 0;
       p++;
   }

   p = 0;
   while ((buf[i] == TAGSYMBOL) && (p < MAX_TAGS))
   {
       tag[p]   = buf[i+1];
       value[p] = (uint16_t)(buf[i+2]<<8) | buf[i+3];
       p++;
       i=i+4;
   }
   *nbtags = p;
   
}  

int IOTSerialClass::IOTSflush(int snum)
{
    int ibuf =-1;
    
	switch (snum) {
    case 0:
      while(Serial.available() > 0) ibuf = Serial.read();
      break;
    case 1:
      while(Serial1.available() > 0) ibuf = Serial1.read();
      break;          
    case 2:
      while(Serial2.available() > 0) ibuf = Serial2.read();
      break;
    default: 
      return -1;
      break;      
   }    
    
   return SUCCESS; 
}  

// Read a raw message
int IOTSerialClass::IOTSRawread(int snum)
{
  unsigned long previousTime;
  
  previousTime = millis();
	    
  switch (snum) {
        
        case 0:
		  while((Serial.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  return Serial.read();
          break;
        
        case 1:
		  while((Serial1.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  Serial.print("IOTSread, read ");
 
		  return Serial1.read();
          break;          
        
        case 2:
		  while((Serial2.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  Serial.print("IOTSread, read ");
 
		  return Serial2.read();
          break;
        default: 
          return -1;
          break;              
  } // end switch	    
}




// Read a structured message
int IOTSerialClass::IOTSread(int snum, uint8_t *msg, uint8_t *msglen)
{
	int ibuf =-1;
	uint8_t istart = 0;
	uint8_t istop  = 0;
	uint8_t i = 0;
	unsigned long previousTime;

	//Get the response from the IOT and add it to the response string
	while ((istop != 2) && (i < MSG_SIZE_MAX))
	{
	    previousTime = millis();
	    
	    switch (snum) {
        
        case 0:
		  while((Serial.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  ibuf = Serial.read();
          break;
        
        case 1:
		  while((Serial1.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  Serial.print("IOTSread, read ");
 
		  ibuf = Serial1.read();
          Serial.print("ibuf: "); Serial.print(ibuf,HEX); Serial.print("/"); Serial.println((isalnum(ibuf))?((char)ibuf):(' '));
          break;          
        
        case 2:
		  while((Serial2.available() == 0) && ( millis()< previousTime+ (30*1000))) // waiting for data until 30 seconds in the serial buffer
          {
#if defined(ESP8266)            
              yield();  // to avoid watchdog timer reseting the ESP8266. 
#endif              
          }
		  Serial.print("IOTSread, read ");
 
		  ibuf = Serial2.read();
          Serial.print("ibuf: "); Serial.print(ibuf,HEX); Serial.print("/"); Serial.println((isalnum(ibuf))?((char)ibuf):(' '));
          break;
        default: 
          return -1;
          break;              
       } // end switch	    
	    
		if (ibuf == -1) return -10; // serial buffer empty
		
		if      (ibuf == SBN1) istart = 1;
		else if (ibuf == SBN2) istart++;
		else if (ibuf == EBN1) istop = 1;
		else if (ibuf == EBN2) istop++;    
        else if (istart == 2) msg[i++] = (uint8_t)ibuf;
	}
    if (i == 0) return -1;
    if (i == MSG_SIZE_MAX) return -11;
    
    *msglen = i;
	return SUCCESS;
}


// Send a raw message
int IOTSerialClass::IOTSRawsend(int snum, uint8_t buf) 
{
	switch (snum) {
    case 0:
        Serial.write(buf);
      break;
    case 1:
        Serial1.write(buf);
      break;          
    case 2:
       Serial2.write(buf);
      break;
    default: 
      return -1;
      break;      
   }    
    
   return SUCCESS;     
}


// Send a structured  message
int IOTSerialClass::IOTSsend(int snum, uint8_t msgtype) // Response short
{
    return IOTSsend(snum, msgtype,  (uint8_t)0 , (uint16_t)0, (uint8_t)0, (uint8_t)0);
}

int IOTSerialClass::IOTSsend(int snum, uint8_t msgtype, uint16_t *param, uint8_t paramlen, uint8_t cmdId)  // Response full
{
    return IOTSsend(snum, msgtype,  (uint8_t)0, param, paramlen, cmdId);
}

int IOTSerialClass::IOTSsend(int snum, uint8_t msgtype, uint16_t *param, uint8_t paramlen)  // Infos, Picture
{
     return IOTSsend(snum, msgtype,  (uint8_t)0, param, paramlen,  (uint8_t)0);
}


int IOTSerialClass::IOTSsend(int snum, uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId) //Full, Command
{
	switch (snum) {
    case 0:
      IOTSsend0(msgtype, cmdType, param, paramlen, cmdId);
      break;
    case 1:
      IOTSsend1(msgtype, cmdType, param, paramlen, cmdId);
      break;          
    case 2:
      IOTSsend2(msgtype, cmdType, param, paramlen, cmdId);
      break;
    default: 
      return -1;
      break;      
   }    
    
   return SUCCESS;     
}

void IOTSerialClass::IOTSsend0(uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId)
{
    Serial.write(SBN1);
    Serial.write(SBN2);
    
    Serial.write(TAGSYMBOL);
    Serial.write(TAG_CMDID);
    Serial.write(uint8_t(0));
    Serial.write(cmdId);
    
    Serial.write(TAGSYMBOL);
    
    switch (msgtype) {
        
    case RESP_OK:
    case RESP_KO:
       Serial.write(TAG_RESP);
       Serial.write(uint8_t(0));
       Serial.write(msgtype);
       break; 
   
    case CMD:
       Serial.write(TAG_CMD);
       Serial.write(uint8_t(0));
       Serial.write(cmdType);
       break;  
   
    case INFOS:
       Serial.write(TAG_INFOS);
       Serial.write(uint8_t(0));
       Serial.write(paramlen);
       break; 
       
    case PICTURE:
       Serial.write(TAG_PICTURE);
       Serial.write(uint8_t(0));
       Serial.write(paramlen);
       break;                
    }     
        
    for (uint8_t j=0; j<paramlen; j++)
    {
       Serial.write(TAGSYMBOL);
       Serial.write(TAG_PARAM);
       Serial.write((uint8_t)(param[j]>>8));
       Serial.write((uint8_t)(param[j]));
    }
        
    Serial.write(EBN1);
    Serial.write(EBN2);

}

void IOTSerialClass::IOTSsend1(uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId)
{
    Serial1.write(SBN1);
    Serial1.write(SBN2);
    
    Serial1.write(TAGSYMBOL);
    Serial1.write(TAG_CMDID);
    Serial1.write(uint8_t(0));
    Serial1.write(cmdId);
    
    Serial1.write(TAGSYMBOL);
    
    switch (msgtype) {
        
    case RESP_OK:
    case RESP_KO:
       Serial1.write(TAG_RESP);
       Serial1.write(uint8_t(0));
       Serial1.write(msgtype);
       break; 
   
    case CMD:
       Serial1.write(TAG_CMD);
       Serial1.write(uint8_t(0));
       Serial1.write(cmdType);
       break;  
   
    case INFOS:
       Serial1.write(TAG_INFOS);
       Serial1.write(uint8_t(0));
       Serial1.write(paramlen);
       break; 
       
    case PICTURE:
       Serial1.write(TAG_PICTURE);
       Serial1.write(uint8_t(0));
       Serial1.write(paramlen);
       break;                
    }     
        
    for (uint8_t j=0; j<paramlen; j++)
    {
       Serial1.write(TAGSYMBOL);
       Serial1.write(TAG_PARAM);
       Serial1.write((uint8_t)(param[j]>>8));
       Serial1.write((uint8_t)(param[j]));
    }
        
    Serial1.write(EBN1);
    Serial1.write(EBN2);

}


void IOTSerialClass::IOTSsend2(uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId)
{
    Serial2.write(SBN1);
    Serial2.write(SBN2);
    
    Serial2.write(TAGSYMBOL);
    Serial2.write(TAG_CMDID);
    Serial2.write(uint8_t(0));
    Serial2.write(cmdId);
    
    Serial2.write(TAGSYMBOL);
    
    switch (msgtype) {
        
    case RESP_OK:
    case RESP_KO:
       Serial2.write(TAG_RESP);
       Serial2.write(uint8_t(0));
       Serial2.write(msgtype);
       break; 
   
    case CMD:
       Serial2.write(TAG_CMD);
       Serial2.write(uint8_t(0));
       Serial2.write(cmdType);
       break;  
   
    case INFOS:
       Serial2.write(TAG_INFOS);
       Serial2.write(uint8_t(0));
       Serial2.write(paramlen);
       break; 
       
    case PICTURE:
       Serial2.write(TAG_PICTURE);
       Serial2.write(uint8_t(0));
       Serial2.write(paramlen);
       break;                
    }     
        
    for (uint8_t j=0; j<paramlen; j++)
    {
       Serial2.write(TAGSYMBOL);
       Serial2.write(TAG_PARAM);
       Serial2.write((uint8_t)(param[j]>>8));
       Serial2.write((uint8_t)(param[j]));
    }
        
    Serial2.write(EBN1);
    Serial2.write(EBN2);

}



