#include <arduino.h>
#include <EEPROM.h>

#define ESP_MEM_SIZE 512

#define SUCCESS       0
#define NO_STOP      -1
#define NO_SSID      -2
#define NO_SEPARATOR -3
#define NO_PWD       -4
   
int get_credentials( char* ssid, char* password);
