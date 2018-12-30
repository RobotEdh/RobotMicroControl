#ifndef RC_h
#define RC_h

#define NBCHANNELS     6 
#define MAXPPM      1920
#define MAXPPMAUX2  1520 
#define MINPPM      1100 
#define MIDPPM      MAXPPM/MINPPM 

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
