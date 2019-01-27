#include <ESPcredentials.h>

void setup() {
  
  String buf = "SSID=sssssss|PWD=pppppppp>";  // updatethe string ad follow:  SSID=sssssss|PWD=pppppppp> with
                                              // sssssss = SSID max 31 chars
                                              // pppppppp = password max 64 chars
                                              
  int address = 0;
 
  Serial.begin (9600);
  Serial.print ("Set credentials value:");  Serial.println (buf);
  
  EEPROM.begin(ESP_MEM_SIZE); 
      
  while (address < (int)buf.length()) {
     EEPROM.write(address,(char)buf[address]);    // write a byte to the current address of the EEPROM
     address++;
  }
  
  if(EEPROM.commit()) Serial.println ("Set credentials OK");
  else                Serial.println ("Set credentials KO");

}
 
void loop() {  
}
