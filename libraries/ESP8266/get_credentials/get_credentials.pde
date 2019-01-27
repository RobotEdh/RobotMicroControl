#include <ESPcredentials.h>

void setup() {
  
  char ssid[31];
  char password[64];
 
  Serial.begin (9600);


  int ret =  get_credentials(ssid, password);
  
  if (ret == 0) {
     Serial.print ("ssid: ");Serial.println (ssid);
     Serial.print ("password: ");Serial.println (password);   
  }
  else  {  
     Serial.print ("Error get credentials: ");Serial.println (ret);
  }   
}

void loop() {  
}
