#include <IOTSerial.h>         // Serial lib to communicate with IOT

IOTSerialClass IOTSerial;    // The IOT serial

void setup()
{ 
  int ret = SUCCESS;
  Serial.begin(9600); // initialize serial port
  
  // initialize the IOT Serial 1 
  ret = IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
  Serial.println ("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK");
   
  Serial.println("End setup ok"); 
}



int test()
{
 uint16_t resp[RESP_SIZE];
 uint8_t resplen = 0;;
  
 int ret = SUCCESS;
 
     Serial.println("Start robot_Send_Infos");
 
     resp[ALERT_STATUS] = 1;
     Serial.print("alert status: ");Serial.println((int)resp[ALERT_STATUS]);
     
     resp[NO_PICTURE] = 7;
     Serial.print("no_picture: ");Serial.println((int)resp[NO_PICTURE]);
     
     resp[MOTOR_STATE] = 2;
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
     Serial.print("noise: ");Serial.println((int)resp[NOISE5]);



     resplen = RESP_SIZE;
 
    //Send the Infos message 
    Serial.println("Call IOTSsend 1 INFOS"); 
    ret = IOTSerial.IOTSsend (1, INFOS, resp, resplen); 
    
    Serial.println("End OK robot_IOT"); 

    return SUCCESS; 
 
}


void loop()
{

 int ret = SUCCESS;
 
 ret = test();
 
 delay (500000);
 
}