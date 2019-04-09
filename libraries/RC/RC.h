#ifndef RC_h
#define RC_h

#include <Arduino.h>

#define NBCHANNELS 6 
#define THROTTLE   0
#define ROLL       1
#define PITCH      2
#define YAW        3
#define AUX1       4
#define AUX2       5

#define MAXPPM      1920.0
#define MAXPPMAUX2  1520.0 
#define MINPPM      1100.0 
#define MIDPPM      1510.0 //(MINPPM+MAXPPM)/2 

class RCClass
{
  public:
    RCClass();
    
    void RC_init(void);
    void RC_getCommands(int16_t RC_command[NBCHANNELS]);
    
  private:
    uint16_t RC_readRaw(uint8_t chan);
};

#endif /* RC_h */
