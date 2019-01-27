#include <ESPcredentials.h>
   
int get_credentials(char* ssid,  char* password) {
  
  uint8_t data = 0;
  int address = 0;
  int SSIDTag = -1;
  int PWDTag = -1;
  int separatorTag = -1;
  String buf;
  String Stssid;
  String STpassword;
  
  EEPROM.begin(ESP_MEM_SIZE);
   
  while (address < ESP_MEM_SIZE &&  data !=0x3E) {  // read until char '>' (0x3E) found
     data = EEPROM.read(address);                   // read a byte from the current address of the EEPROM
     buf.concat((char)data);
     address++;
  }
  if (address == ESP_MEM_SIZE) return NO_STOP;
   
  // decode  SSID=sssssss|PWD=pppppppp>
  SSIDTag = buf.indexOf("SSID=");
  if (SSIDTag == -1) return NO_SSID;
  
  separatorTag = buf.indexOf("|");
  if (separatorTag == -1) return NO_SEPARATOR;
  Stssid = buf.substring(SSIDTag + 5, separatorTag);
  
  PWDTag = buf.indexOf("PWD=");
  if (PWDTag == -1) return NO_PWD;
  STpassword = buf.substring(PWDTag + 4, address-1);
  
  strcpy(ssid, Stssid.c_str());
  strcpy(password, STpassword.c_str());
  
  return SUCCESS;
}