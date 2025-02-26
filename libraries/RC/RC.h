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

#define MAXSPEED     120.0 //degree/sec
#define MAXPPM      2000.0
#define MAXPPMAUX2  1520.0 
#define MINPPM      1100.0 
#define MIDPPM      1500.0 
#define NEUTRALBAND    8.0 

class RCClass
{
  public:
    RCClass();
    
    void RC_init(void);
    void RC_getAngularSpeedCommands(double RC_angularspeedcommand[NBCHANNELS]); // obsolete
    
    double RC_get_a1();
    double RC_get_b1();
    double RC_get_a2();
    double RC_get_b2();
    
    void RC_getCommands(int16_t RC_command[NBCHANNELS]);
    
  private:
    uint16_t RC_readRaw(uint8_t chan);
    double _a1, _b1, _a2, _b2;
};

#endif /* RC_h */
