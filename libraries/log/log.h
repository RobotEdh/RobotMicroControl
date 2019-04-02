#ifndef LOG_h
#define LOG_h
	
#ifdef NOLOG    // no logging
   
   #define PRINTbegin
   #define PRINTflush  
   #define PRINTend
    
   #define PRINTa(s)  
   #define PRINTs(s)       
   #define PRINT(s,v)    
   #define PRINTx(s,v)      
   #define PRINTb(s,v)  
  
   #define PRINTi(s,v1,v2) 
   #define PRINTj(s,v1,v2)
   #define PRINT6(s,v1,v2,v3,v4,v5,v6)   
   #define PRINTd 
   #define PRINTt              
	
#elif defined  LOGSERIAL    // log Serial

   #define PRINTbegin Serial.begin(115200);
   #define PRINTflush
   #define PRINTend   Serial.end(); 
   
   #define PRINTa(s)   { Serial.print(F(s)); }
   #define PRINTs(s)   { Serial.println(F(s)); }
   #define PRINT(s,v)  { Serial.print(F(s)); Serial.println(v); }
   #define PRINTx(s,v) { Serial.print(F(s)); Serial.print(F("0x")); Serial.println(v, HEX); }
   #define PRINTb(s,v) { Serial.print(F(s)); Serial.print(F("0b")); Serial.println(v, BIN); }
    
   #define PRINTi(s,v1,v2)  { Serial.print(F(s)); Serial.print(F("["));Serial.print(v1);Serial.print(F("]: "));Serial.println(v2); }  
   #define PRINTj(s,v1,v2)  { Serial.print(F(s)); Serial.print(F("["));Serial.print(v1);Serial.print(F("]: 0x"));Serial.println(v2,HEX); }
   #define PRINT6(s,v1,v2,v3,v4,v5,v6) { Serial.print(F(s));Serial.print(v1,HEX);Serial.print(F(":"));Serial.print(v2,HEX);Serial.print(F(":"));Serial.print(v3,HEX);Serial.print(F(":"));Serial.print(v4,HEX);Serial.print(F(":"));Serial.print(v5,HEX);Serial.print(F(":"));Serial.println(v6,HEX); }  
   #define PRINTd { Serial.print(F("Date: "));Serial.print(now.days);Serial.print(F("/"));Serial.print(now.months);Serial.print(F("/"));Serial.println(now.year + 2000); }
   #define PRINTt { Serial.print(F("Time: "));Serial.print(now.hours);Serial.print(F(":"));Serial.print(now.minutes);Serial.print(F(":"));Serial.println(now.seconds); }

   #define PRINTTRACE Serial.print(millis());Serial.print(F("#"));Serial.print(__PRETTY_FUNCTION__);Serial.print(F("->"));Serial.print(__LINE__);Serial.println(F("|"));

#elif defined  LOGSERIAL1    // log Serial1

   #define PRINTbegin Serial1.begin(115200);
   #define PRINTflush
   #define PRINTend   Serial1.end(); 
   
   #define PRINTa(s)   { Serial1.print(F(s)); }   
   #define PRINTs(s)   { Serial1.println(F(s)); }
   #define PRINT(s,v)  { Serial1.print(F(s)); Serial1.println(v); }
   #define PRINTx(s,v) { Serial1.print(F(s)); Serial1.print(F("0x")); Serial1.println(v, HEX); }
   #define PRINTb(s,v) { Serial1.print(F(s)); Serial1.print(F("0b")); Serial1.println(v, BIN); }
 
   #define PRINTi(s,v1,v2)  { Serial1.print(F(s)); Serial1.print(F("["));Serial1.print(v1);Serial1.print(F("]: "));Serial1.println(v2); }  
   #define PRINTj(s,v1,v2)  { Serial1.print(F(s)); Serial1.print(F("["));Serial1.print(v1);Serial1.print(F("]: 0x"));Serial1.println(v2,HEX); }
   #define PRINT6(s,v1,v2,v3,v4,v5,v6) { Serial1.print(F(s));Serial1.print(v1,HEX);Serial1.print(F(":"));Serial1.print(v2,HEX);Serial1.print(F(":"));Serial1.print(v3,HEX);Serial1.print(F(":"));Serial1.print(v4,HEX);Serial1.print(F(":"));Serial1.print(v5,HEX);Serial1.print(F(":"));Serial1.println(v6,HEX); }  
   #define PRINTd { Serial1.print(F("Date: "));Serial1.print(now.days);Serial1.print(F("/"));Serial1.print(now.months);Serial1.print(F("/"));Serial1.println(now.year + 2000); }
   #define PRINTt { Serial1.print(F("Time: "));Serial1.print(now.hours);Serial1.print(F(":"));Serial1.print(now.minutes);Serial1.print(F(":"));Serial1.println(now.seconds); }
     	
#elif defined LOGSDCARD    // log SD card   
          
   #include <SD.h>
   #include <sdcard.h>
   #define  CS_PIN  10  
    
   #define PRINTbegin   if (SD.begin(CS_PIN)) logFile=SD.open("log.txt", FILE_WRITE);logFile.println(F(" ")); logFile.println(F("BEGIN LOG")); logFile.println(F("*********"));logFile.println(F(" "));
   #define PRINTflush   logFile.println(F(" "));logFile.flush();
   #define PRINTend     logFile.println(F(" "));logFile.println(F("END LOG"));logFile.println(F("*******"));logFile.println(F(" "));logFile.close();
   
   #ifdef LOGTRACE
      #define PRINTTRACE logFile.print(millis());logFile.print(F("#"));logFile.print(__PRETTY_FUNCTION__);logFile.print(F("->"));logFile.print(__LINE__);logFile.print(F("|"));
   #else
      #define PRINTTRACE
   #endif
   
   #define PRINTa(s)    { PRINTTRACE logFile.print(F(s)); }   
   #define PRINTs(s)    { PRINTTRACE logFile.println(F(s)); }
   #define PRINT(s,v)   { PRINTTRACE logFile.print(F(s)); logFile.println(v); }
   #define PRINTx(s,v)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("0x")); logFile.println(v, HEX); }
   #define PRINTb(s,v)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("0b")); logFile.println(v, BIN); }
  
   #define PRINTi(s,v1,v2)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("["));logFile.print(v1);logFile.print(F("]: "));logFile.println(v2); } 
   #define PRINTi2(s,v1,v2) { PRINTTRACE logFile.print(F(s)); logFile.print(F("["));logFile.print(v1);logFile.print(F("]|"));logFile.println(v2); }  
  
   #define PRINTj(s,v1,v2)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("["));logFile.print(v1);logFile.print(F("]: 0x"));logFile.println(v2,HEX); }   
   #define PRINT6(s,v1,v2,v3,v4,v5,v6) { PRINTTRACE logFile.print(F(s));logFile.print(v1,HEX);logFile.print(F(":"));logFile.print(v2,HEX);logFile.print(F(":"));logFile.print(v3,HEX);logFile.print(F(":"));logFile.print(v4,HEX);logFile.print(F(":"));logFile.print(v5,HEX);logFile.print(F(":"));logFile.println(v6,HEX); }     
   #define PRINTd { logFile.print(F("Date: "));logFile.print(now.days);logFile.print(F("/"));logFile.print(now.months);logFile.print(F("/"));logFile.println(now.year + 2000); }
   #define PRINTt { logFile.print(F("Time: "));logFile.print(now.hours);logFile.print(F(":"));logFile.print(now.minutes);logFile.print(F(":"));logFile.println(now.seconds); }
 
#endif

#endif