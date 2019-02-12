#ifndef LOG_h
#define LOG_h
	
#ifdef NOLOG    // no logging
   
   #define PRINTbegin
   #define PRINTflush  
   #define PRINTend 
    
   #define PRINTs(s)       
   #define PRINT(s,v)    
   #define PRINTx(s,v)      
   #define PRINTb(s,v)  
   
   #define PRINTd 
   #define PRINTt              
	
#elif defined  LOGSERIAL    // log Serial

   #define PRINTbegin Serial.begin(9600);
   #define PRINTflush
   #define PRINTend   Serial.end(); 
   
   #define PRINTs(s)   { Serial.println(F(s)); }
   #define PRINT(s,v)  { Serial.print(F(s)); Serial.println(v); }
   #define PRINTx(s,v) { Serial.print(F(s)); Serial.print(F("0x")); Serial.println(v, HEX); }
   #define PRINTb(s,v) { Serial.print(F(s)); Serial.print(F("0b")); Serial.println(v, BIN); }
   
   #define PRINTd { Serial.print(F("Date: "));Serial.print(now.days);Serial.print(F("/"));Serial.print(now.months);Serial.print(F("/"));Serial.println(now.year + 2000); }
   #define PRINTt { Serial.print(F("Time: "));Serial.print(now.hours);Serial.print(F(":"));Serial.print(now.minutes);Serial.print(F(":"));Serial.println(now.seconds); }
     	
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
   
   #define PRINTs(s)    { PRINTTRACE logFile.println(F(s)); }
   #define PRINT(s,v)   { PRINTTRACE logFile.print(F(s)); logFile.println(v); }
   #define PRINTx(s,v)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("0x")); logFile.println(v, HEX); }
   #define PRINTb(s,v)  { PRINTTRACE logFile.print(F(s)); logFile.print(F("0b")); logFile.println(v, BIN); }
   
   #define PRINTd { logFile.print(F("Date: "));logFile.print(now.days);logFile.print(F("/"));logFile.print(now.months);logFile.print(F("/"));logFile.println(now.year + 2000); }
   #define PRINTt { logFile.print(F("Time: "));logFile.print(now.hours);logFile.print(F(":"));logFile.print(now.minutes);logFile.print(F(":"));logFile.println(now.seconds); }
 
#endif

#endif