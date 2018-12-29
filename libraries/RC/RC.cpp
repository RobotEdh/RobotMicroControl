#include <Arduino.h>
#include <RC.h>

//RAW RC values will be store here
volatile uint16_t rcValue[3] = {1500, 1500, 1500}; // interval [1000;2000]

RCClass::RCClass(void)
{
}

ISR (PCINT0_vect)
{
    static volatile uint32_t edgeTime[NBCHANNELS];
    static volatile uint8_t lastb; // previous PORTB
    uint8_t thisb; // current PORTB
    uint32_t currTime, dTime; // current time, delta time

    
    thisb = PORTB;          // read PORT B   
    currTime = micros();

    if ((thisb ^ lastb) & 0x02) {  // RB1 changed (0x02= 10)  ROLLPIN    
      if (!(thisb & 0x02)) {       // RB1 is low                        
        dTime = currTime-edgeTime[0];                            
        if (900<dTime && dTime<2200) {  // filter erroneous values                             
          rcValue[0] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[0] = currTime; // RB1 is high                            
    }        
        
    if ((thisb ^ lastb) & 0x04) {  // RB2 changed (0x04= 100)  PITCHPIN     
      if (!(thisb & 0x04)) {       // RB2 is low                        
        dTime = currTime-edgeTime[1];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[1] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[1] = currTime; // RB2 is high                            
    }        

    if ((thisb ^ lastb) & 0x08) {  // RB3 changed (0x08= 1000)   YAWPIN    
      if (!(thisb & 0x08)) {       // RB3 is low                        
        dTime = currTime-edgeTime[2];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[2] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[2] = currTime; // RB3 is high                            
    } 
    
    if ((thisb ^ lastb) & 0x10) {  // RB4 changed (0x10= 10000)   THROTTLE    
      if (!(thisb & 0x10)) {       // RB4 is low                        
        dTime = currTime-edgeTime[3];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[3] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[3] = currTime; // RB4 is high                            
    }            
    
    lastb = thisb;    // Memorize the current state of PORT B
 
    
 }  // end of PCINT0_vect
 

void RCClass::RC_init()
{ 
  // pin change interrupt
  /*
D0	  PCINT16 (PCMSK2 / PCIF2 / PCIE2)
D1	  PCINT17 (PCMSK2 / PCIF2 / PCIE2)
D2	  PCINT18 (PCMSK2 / PCIF2 / PCIE2)
D3	  PCINT19 (PCMSK2 / PCIF2 / PCIE2)
D4	  PCINT20 (PCMSK2 / PCIF2 / PCIE2)
D5	  PCINT21 (PCMSK2 / PCIF2 / PCIE2)
D6	  PCINT22 (PCMSK2 / PCIF2 / PCIE2)
D7	  PCINT23 (PCMSK2 / PCIF2 / PCIE2)

D8	  PCINT0  (PCMSK0 / PCIF0 / PCIE0)
D9	  PCINT1  (PCMSK0 / PCIF0 / PCIE0)
D10	  PCINT2  (PCMSK0 / PCIF0 / PCIE0)
D11	  PCINT3  (PCMSK0 / PCIF0 / PCIE0)
D12	  PCINT4  (PCMSK0 / PCIF0 / PCIE0)
D13	  PCINT5  (PCMSK0 / PCIF0 / PCIE0)

A0	  PCINT8  (PCMSK1 / PCIF1 / PCIE1)
A1	  PCINT9  (PCMSK1 / PCIF1 / PCIE1)
A2	  PCINT10 (PCMSK1 / PCIF1 / PCIE1)
A3	  PCINT11 (PCMSK1 / PCIF1 / PCIE1)
A4	  PCINT12 (PCMSK1 / PCIF1 / PCIE1)
A5	  PCINT13 (PCMSK1 / PCIF1 / PCIE1)
*/
  PCMSK0 |= bit (PCINT0);  // want pin 8
  PCMSK0 |= bit (PCINT1);  // want pin 9
  PCMSK0 |= bit (PCINT2);  // want pin 10
  PCMSK0 |= bit (PCINT3);  // want pin 11
  PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);   // enable pin change interrupts for D8 to D13
  DDRB = DDRB & 0b11000000;   // Set pins 8 à 13 of PORTB as input
  PORTB;                      //read PortB to clear any mismatch
}


uint16_t RCClass::RC_readRaw(uint8_t chan)
{
  uint16_t data;
   
  cli();		// turn off interrupts
  data = rcValue[chan];  // Let's copy the data Atomically
  sei();		// turn on interrupts
 
  return data; // We return the value correctly copied when the IRQ's where disabled
}

void RCClass::RC_getCommands(int16_t RC_command[NBCHANNELS])
{
  uint16_t rcData[NBCHANNELS];
 
  for (int i = 0; i < NBCHANNELS; i++) { // read data from all channels
        rcData[i] = RC_readRaw(i);
        RC_command[i] = min(abs(rcData[i]-MIDRC),500);         // interval [#1000;#2000] ...
        if (rcData[i]<MIDRC) RC_command[i] = -RC_command[i]; // ...translated to interval [-500; +500]
  }
}