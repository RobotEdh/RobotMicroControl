#ifndef RC_h
#define RC_h

#define NBCHANNELS 4 
#define MAXPPM      2000 
#define MINPPM      1000 
#define MIDRC       MAXPPM/MINPPM 

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
