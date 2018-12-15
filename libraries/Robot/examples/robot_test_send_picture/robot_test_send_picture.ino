

#define PAYLOAD_SIZE 100
#define FILE_OPEN_ERROR  -1000
#define FILE_CLOSE_ERROR -1001


#include <sdcard.h>            // SD Card
#include <SD.h> 
#include <IOTSerial.h>         // Serial lib to communicate with IOT

//Create an instance of the SD file
extern SdFile root;          // SD Root
      SdFile FilePicture;   // SD File

IOTSerialClass IOTSerial;    // The IOT serial

void setup()
{ 
  int ret =0;
  Serial.begin(9600); // initialize serial port
  
  ret = initSDCard();
  if (!FilePicture.open(&root, "test63.jpg", O_READ)) return FILE_OPEN_ERROR; 
  
  // initialize the IOT Serial 1 
  IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
  Serial.println ("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK");
   
  Serial.println("End setup ok"); 
}



int test()
{
 uint16_t param[MAX_PARAMS];
 uint8_t paramlen = 0;
 uint8_t msg[MSG_SIZE_MAX];
 uint8_t msg_len = 0;
 uint8_t tag [MAX_TAGS];
 uint16_t value [MAX_TAGS];
 uint8_t nbtags;
 
 uint16_t nbytes = 0;
 uint16_t oldnbytes = 0;
 uint16_t tbytes = 0;
 uint8_t buf[PAYLOAD_SIZE];
  
 uint16_t resp[RESP_SIZE];
 uint8_t resplen = 0;
 
 uint8_t n = 7; //picture nb
 
 int ret = SUCCESS;
 
 Serial.println("Start robot_Send_Picture");


 
 
     resp[ALERT_STATUS] = 1;
     Serial.print("alert status: ");Serial.println((int)resp[ALERT_STATUS]);
     
     resp[NO_PICTURE] = 0;  // No Picture
     Serial.print("no_picture: ");Serial.println((int)resp[NO_PICTURE]);
     
     resp[MOTOR_STATE] = 0;
     Serial.print("motor_state: ");Serial.println((int)resp[MOTOR_STATE]);
     
     resp[DIRECTION] =3;
     Serial.print("direction: ");Serial.println((int)resp[DIRECTION]);
    
     resp[DISTANCE] = 4;
     Serial.print("distance: ");Serial.println((int)resp[DISTANCE]);
     
     resp[TEMPERATURE] = 5;
     Serial.print("temperature: ");Serial.println((int)resp[TEMPERATURE]);

     resp[BRIGHTNESS] = 6;
     Serial.print("brightness: ");Serial.println((int)resp[BRIGHTNESS]);
     
     resp[NOISE] = 7;
     Serial.print("noise: ");Serial.println((int)resp[NOISE]);



     resplen = RESP_SIZE;
 
    //Send the Infos message 
    Serial.println("Call IOTSsend 1 INFOS"); 
    ret = IOTSerial.IOTSsend (1, INFOS, resp, resplen); 
    
    Serial.println("Send the first Infos message OK"); 
    
    
 
 //Send the Picture message 
 param[0] = n;
 param[1] = (uint16_t)FilePicture.fileSize();
 paramlen = 2;
 Serial.print("param[0]: "); Serial.print((int)param[0]); Serial.print(", param[1]: "); Serial.println((int)param[1]);
 
 ret = IOTSerial.IOTSsend(1, PICTURE, param, paramlen); 
            
 //Read the message replied to be sure that the client is ready to receive the picture
 ret = IOTSerial.IOTSread(1, msg, &msg_len);
 Serial.print("Call IOTSread 1, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);

 if (ret != SUCCESS) {
     Serial.println("error IOTSread");  
     ret = IOTSerial.IOTSflush(1);
     return 0;
 }
 
 //Decode the message
 IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
 Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
 if (nbtags < 1)          return -1; 
    
 Serial.print("tag[0]: ");Serial.println((int)tag[0],HEX); 
 if (tag[0]!= TAG_CMDID)   return -2;
 Serial.print("tag[1]: "); Serial.println((int)tag[1]);  
 if (tag[1]!= TAG_RESP)   return -3;
 Serial.print("value[1]: "); Serial.println((int)value[1]);        
 if (value[1] == RESP_KO) return -4;
 if (value[1] != RESP_OK) return -5;

 // read from the file until there's nothing else in it:
 while ((nbytes = FilePicture.read(buf, sizeof(buf))) > 0 && ret == SUCCESS) {
       for (uint16_t i = 0;i<nbytes;i++)
       {
         ret = IOTSerial.IOTSRawsend(1, buf [i]); 
         tbytes++;
       }
       oldnbytes = nbytes;
 }// while 
  
 //Close file
 if (!FilePicture.close()) return FILE_CLOSE_ERROR; 

  
 Serial.print("last nbytes: "); Serial.println((int)oldnbytes);
 Serial.print("last buf [oldnbytes-2]: "); Serial.println((int)buf[oldnbytes-2],HEX);
 Serial.print("last buf [oldnbytes-1]: "); Serial.println((int)buf[oldnbytes-1],HEX); 
 Serial.print("total bytes: "); Serial.println((int)tbytes);  
 
 
 
  
     resp[ALERT_STATUS] = 1;
     Serial.print("alert status: ");Serial.println((int)resp[ALERT_STATUS]);
     
     resp[NO_PICTURE] = n;
     Serial.print("no_picture: ");Serial.println((int)resp[NO_PICTURE]);
     
     resp[MOTOR_STATE] = 0;
     Serial.print("motor_state: ");Serial.println((int)resp[MOTOR_STATE]);
     
     resp[DIRECTION] =3;
     Serial.print("direction: ");Serial.println((int)resp[DIRECTION]);
    
     resp[DISTANCE] = 4;
     Serial.print("distance: ");Serial.println((int)resp[DISTANCE]);
     
     resp[TEMPERATURE] = 5;
     Serial.print("temperature: ");Serial.println((int)resp[TEMPERATURE]);

     resp[BRIGHTNESS] = 6;
     Serial.print("brightness: ");Serial.println((int)resp[BRIGHTNESS]);
     
     resp[NOISE] = 7;
     Serial.print("noise: ");Serial.println((int)resp[NOISE]);

     resplen = RESP_SIZE;
 
 
    //Send the Infos message 
    Serial.println("Call IOTSsend 1 INFOS"); 
    ret = IOTSerial.IOTSsend (1, INFOS, resp, resplen); 
    
    Serial.println("Send the second Infos message OK"); 
 
 
 
 
 
 
 Serial.println("End OK robot_IOT"); 

 return SUCCESS; 
 
}


void loop()
{

 int ret = SUCCESS;
 
 ret = test();
 
 delay (500000);
 
}
