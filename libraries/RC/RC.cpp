#include <Arduino.h>
#include <RC.h>

//RAW RC values will be store here
volatile uint16_t rcValue[NBCHANNELS] = {MIDPPM, MIDPPM, MIDPPM, MINPPM, MINPPM, MINPPM}; 

RCClass::RCClass(void)
{
}

ISR (PCINT0_vect)
{
    static volatile uint32_t edgeTime[NBCHANNELS];
    static volatile uint8_t lastb; // previous PORTB
    uint8_t thisb;                 // current PORTB
    uint32_t currTime, dTime;      // current time, delta time

    
    thisb = PINB;          // read PORT B   
    currTime = micros();

    if ((thisb ^ lastb) & 0b000001) {  // RB1 changed ROLL   
      if (!(thisb & 0b000001)) {       // RB1 is low                        
        dTime = currTime-edgeTime[0];                            
        if (900<dTime && dTime<2200) {  // filter erroneous values                             
          rcValue[0] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[0] = currTime; // RB1 is high                            
    }        
        
    if ((thisb ^ lastb) & 0b000010) {  // RB2 changed PITCH    
      if (!(thisb & 0b000010)) {       // RB2 is low                        
        dTime = currTime-edgeTime[1];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[1] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[1] = currTime; // RB2 is high                            
    }        

    if ((thisb ^ lastb) & 0b000100) {  // RB3 changed YAW    
      if (!(thisb & 0b000100)) {       // RB3 is low                        
        dTime = currTime-edgeTime[2];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[2] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[2] = currTime; // RB3 is high                            
    } 
    
    if ((thisb ^ lastb) & 0b001000) {  // RB4 changed THROTTLE    
      if (!(thisb & 0b001000)) {       // RB4 is low                        
        dTime = currTime-edgeTime[3];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[3] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[3] = currTime; // RB4 is high                            
    }            
     
    if ((thisb ^ lastb) & 0b010000) {  // RB5 changed AUX1    
      if (!(thisb & 0b010000)) {       // RB5 is low                        
        dTime = currTime-edgeTime[4];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[4] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[4] = currTime; // RB5 is high                            
    }
    
    if ((thisb ^ lastb) & 0b100000) {  // RB6 changed AUX2    
      if (!(thisb & 0b100000)) {       // RB6 is low                        
        dTime = currTime-edgeTime[5];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[5] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[5] = currTime; // RB6 is high                            
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
  PCMSK0 |= bit (PCINT4);  // want pin 12
  PCMSK0 |= bit (PCINT5);  // want pin 13
  PCIFR  |= bit (PCIF0);   // clear any outstanding interrupts
  PCICR  |= bit (PCIE0);   // enable pin change interrupts for D8 to D13
  DDRB = DDRB & 0b11000000;   // Set pins 8 à 13 of PORTB as input
  PINB;                      //read PortB to clear any mismatch
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
  uint16_t RC_data;
  for (int i = 0; i < NBCHANNELS; i++) {         // read data from all channels
        RC_data = RC_readRaw(i);
        if (i < 3) RC_command[i] = RC_data - MIDPPM;                                                   // roll, pitch, yaw
        else if (i == 3) { if (RC_data < 1.1*MINPPM)     RC_command[i] = 0; else RC_command[i] = RC_data;} // throttle 
        else if (i == 4) { if (RC_data > 0.9*MAXPPM)     RC_command[i] = 1; else RC_command[i] = 0;}       // aux1
        else if (i == 5) { if (RC_data > 0.9*MAXPPMAUX2) RC_command[i] = 1; else RC_command[i] = 0;}       // aux2    
  } // end for

}