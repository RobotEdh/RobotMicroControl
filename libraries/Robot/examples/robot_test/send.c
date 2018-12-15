

#define PAYLOAD_SIZE 100

#define SUCCESS              0
#define FILE_OPEN_ERROR  -1000
#define FILE_CLOSE_ERROR -1001


#include <sdcard.h>     // used to read the picture on a SD-Card
//Create an instance of the SD file
File FilePicture; 

int16_t nbytes, tbytes;
char buf[PAYLOAD_SIZE-1];     //First byte is used as indicator
uint8_t buffer[PAYLOAD_SIZE];

void setup()
{ 


  Serial.begin(9600); // initialize serial port
  Serial1.begin(9600); // initialize serial 1 port
    if (!SD.begin(8)) Serial.println("\n\n\ninitialization SD card failed!"); else Serial.println("\n\n\ninitialization card OK!");
  FilePicture = SD.open("test63.jpg", O_READ);
  Serial.println("open ok "); 
  tbytes=0; 
 
}



void loop()
{
 int ret=0;
 

  
  // read from the file until there's nothing else in it:
  while ((nbytes = FilePicture.read(buf, sizeof(buf))) > 0 && ret == SUCCESS) {
      tbytes=tbytes+nbytes;Serial.print("tbytes: "); Serial.println(tbytes);
       if (nbytes == sizeof(buf)) 
       {
           buffer[0] = 0;
       }
       else
       {
           buffer[0] = 1; //end file read
       }  
    
       unsigned int idx = 0;
       for (unsigned int i = 1;i<nbytes+1;i++)
       {
           buffer [i] = buf[idx++];
           Serial1.write( buffer [i]);
     
       }
            

       if (ret != SUCCESS ) return -1;
 
  }// while
  
  

}
