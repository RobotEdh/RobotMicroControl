#include <Arduino.h>
#include <RC.h>

//RAW RC values will be store here
volatile uint16_t rcValue[NBCHANNELS] = {0}; 

RCClass::RCClass(void)
{
}

ISR (PCINT0_vect)
{
    static volatile uint32_t edgeTime[NBCHANNELS]= {0};
    static volatile uint8_t lastb = PINB; // previous PORTB
    uint8_t thisb;                        // current PORTB
    uint32_t currTime; // current time, 
    uint32_t dTime;    // duration high level = time changing low - time changing high

    
    thisb = PINB;          // read PORT B   
    currTime = micros();
    
    //THROTTLE
    if ((thisb ^ lastb) & 0x01) {   // RB1 changed 
      if (!(thisb &  0x01)) {       // RB1 is low => compute duration high level                        
        dTime = currTime-edgeTime[THROTTLE];                            
        if (900<dTime && dTime<2200) {  // filter erroneous values                             
          rcValue[THROTTLE] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[THROTTLE] = currTime; // RB1 is high => store time rising high level                            
    }        
       
    //ROLL    
    if ((thisb ^ lastb) &  0x02) {  // RB2 changed 
      if (!(thisb & 0x02)) {       // RB2 is low => compute duration high level                        
        dTime = currTime-edgeTime[ROLL];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[ROLL] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[ROLL] = currTime; // RB2 is high => store time rising high level                            
    }        

    // PITCH
    if ((thisb ^ lastb) & 0x04) {  // RB3 changed  
      if (!(thisb & 0x04)) {       // RB3 is low => compute duration high level                        
        dTime = currTime-edgeTime[PITCH];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[PITCH] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[PITCH] = currTime; // RB3 is high => store time rising high level                            
    } 
    
    //YAW
    if ((thisb ^ lastb) & 0x08) {  // RB4 changed
      if (!(thisb & 0x08)) {       // RB4 is low => compute duration high level                        
        dTime = currTime-edgeTime[YAW];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[YAW] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[YAW] = currTime; // RB4 is high  => store time rising high level                           
    }            
    
    //AUX1 
    if ((thisb ^ lastb) & 0x10) {  // RB5 changed   
      if (!(thisb & 0x10)) {       // RB5 is low => compute duration high level                        
        dTime = currTime-edgeTime[AUX1];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[AUX1] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[AUX1] = currTime; // RB5 is high  => store time rising high level                           
    }
    
    //AUX2
    if ((thisb ^ lastb) & 0x20) {  // RB6 changed    
      if (!(thisb & 0x20)) {       // RB6 is low  => compute duration high level                       
        dTime = currTime-edgeTime[AUX2];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[AUX2] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[AUX2] = currTime; // RB6 is high => store time rising high level                            
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
  DDRB  = DDRB  & B11000000; // set pins 8 à 13 of PORTB as input 
  PORTB = PORTB | B00111111; // activate pull-up resitors on pins 8 à 13 of PORTB in order to avoid random values
  uint8_t p = PINB;          // read PortB to clear any mismatch
  
  rcValue[THROTTLE] = MINPPM;
  rcValue[ROLL]     = MIDPPM;
  rcValue[PITCH]    = MIDPPM;
  rcValue[YAW]      = MIDPPM;
  rcValue[AUX1]     = MINPPM;
  rcValue[AUX2]     = MINPPM;
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
  double RC_data;
  
  for (int i = 0; i < NBCHANNELS; i++) {         // read data from all channels
        RC_data = (double)RC_readRaw(i);
        if ((i == ROLL) || (i == PITCH) ||(i == YAW)) RC_command[i] = (int16_t)(RC_data - MIDPPM)* 180.0 * 2.0 /(MAXPPM -MINPPM); // roll, pitch, yaw convert to range [-180;+180]
        else if (i == THROTTLE) { if (RC_data < 1.1*MINPPM) RC_command[i] = 0; else RC_command[i] = (int16_t)RC_data;}           // throttle 
        else if (i == AUX1) { if (RC_data > 0.9*MAXPPM)     RC_command[i] = 1; else RC_command[i] = 0;}                          // aux1
        else if (i == AUX2) { if (RC_data > 0.9*MAXPPMAUX2) RC_command[i] = 1; else RC_command[i] = 0;}                          // aux2    
  } // end for

}