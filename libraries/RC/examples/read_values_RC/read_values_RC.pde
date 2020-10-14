#include <RC.h>

RCClass RC;     

void setup()
{
  
  Serial.begin(115200); // initialize serial port
  
  Serial.println("Start Init" );  
  
  RC.RC_init();
 
  Serial.println("End Init" ); 

}


void loop()
{
  long starttime;
  int16_t RC_command[NBCHANNELS];
  const char *szChannels[] = {"THROTTLE", "ROLL", "PITCH", "YAW", "AUX1", "AUX2"};
  
  Serial.println("move tick for 5s to get RC Commands"); 
  delay(5*1000);
  starttime = millis();
  
  RC.RC_getCommands(RC_command);
 
  Serial.print("elasptime: "); Serial.println(millis() - starttime);
  for (int i = 0; i < NBCHANNELS; i++) { // read data from all channels
     Serial.print("RC_command[");Serial.print(i); Serial.print("] - ");Serial.print(szChannels[i]);Serial.print(": ");Serial.println(RC_command[i]);
  }
}
