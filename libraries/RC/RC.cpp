#include <Arduino.h>
#include <RC.h>

//RAW RC values will be store here
volatile uint16_t rcValue[NBCHANNELS] = {0}; 

RCClass::RCClass(void)
{
}

ISR (PCINT2_vect) // ISR(PCINT2_vect){} for pins PCINT16-PCINT23 (PK0- PK7)
{
    static volatile uint32_t edgeTime[NBCHANNELS]= {0};
    static volatile uint8_t lastk = PINK; // previous PORTK
    uint8_t thisk;                        // current PORTK
    uint32_t currTime; // current time, 
    uint32_t dTime;    // duration high level = time changing low - time changing high

    
    thisk = PINK;          // read PORT K  
    currTime = micros();
    
    if ((thisk ^ lastk) & 0x01) {   // RK0 changed 
      if (!(thisk &  0x01)) {       // RK0 is low => compute duration high level                        
        dTime = currTime-edgeTime[THROTTLE];                            
        if (900<dTime && dTime<2200) {  // filter erroneous values                             
          rcValue[THROTTLE] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[THROTTLE] = currTime; // RK0 is high => store time rising high level                            
    }        
        
    if ((thisk ^ lastk) &  0x02) {  // RK1 changed 
      if (!(thisk & 0x02)) {       // RK1 is low => compute duration high level                        
        dTime = currTime-edgeTime[ROLL];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[ROLL] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[ROLL] = currTime; // RK1 is high => store time rising high level                            
    }        

    if ((thisk ^ lastk) & 0x04) {  // RK2 changed  
      if (!(thisk & 0x04)) {       // RK2 is low => compute duration high level                        
        dTime = currTime-edgeTime[PITCH];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[PITCH] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[PITCH] = currTime; // RK2 is high => store time rising high level                            
    } 
    
    if ((thisk ^ lastk) & 0x08) {  // RK3 changed
      if (!(thisk & 0x08)) {       // RK3 is low => compute duration high level                        
        dTime = currTime-edgeTime[YAW];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[YAW] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[YAW] = currTime; // RK3 is high  => store time rising high level                           
    }            
    
    if ((thisk ^ lastk) & 0x10) {  // RK4 changed   
      if (!(thisk & 0x10)) {       // RK4 is low => compute duration high level                        
        dTime = currTime-edgeTime[AUX1];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[AUX1] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[AUX1] = currTime; // RK4 is high  => store time rising high level                           
    }
    
    if ((thisk ^ lastk) & 0x20) {  // RK5 changed    
      if (!(thisk & 0x20)) {       // RK5 is low  => compute duration high level                       
        dTime = currTime-edgeTime[AUX2];                             
        if (900<dTime && dTime<2200) {  // filter erroneous values                              
          rcValue[AUX2] = (uint16_t)dTime;                             
        }                                                            
      }
      else
        edgeTime[AUX2] = currTime; // RK5 is high => store time rising high level                            
    }
           
    lastk = thisk;    // Memorize the current state of PORT K
 
    
 }  // end of PCINT2_vect
 

void RCClass::RC_init()
{ 
/*ISR(PCINT2_vect){} for pins PCINT16-PCINT23 (PK0- PK7)

A8	  PCINT16 (PCMSK2 / PCIF2 / PCIE2) PK0
A9	  PCINT17 (PCMSK2 / PCIF2 / PCIE2) PK1
A10	  PCINT18 (PCMSK2 / PCIF2 / PCIE2) PK2
A11	  PCINT19 (PCMSK2 / PCIF2 / PCIE2) PK3
A12	  PCINT20 (PCMSK2 / PCIF2 / PCIE2) PK4
A13	  PCINT21 (PCMSK2 / PCIF2 / PCIE2) PK5
A14	  PCINT22 (PCMSK2 / PCIF2 / PCIE2) PK6
A15	  PCINT23 (PCMSK2 / PCIF2 / PCIE2) PK7
*/
  PCMSK2 |= bit (PCINT16);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT16
  PCMSK2 |= bit (PCINT17);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT17
  PCMSK2 |= bit (PCINT18);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT18
  PCMSK2 |= bit (PCINT19);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT19
  PCMSK2 |= bit (PCINT20);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT20
  PCMSK2 |= bit (PCINT21);  // PCMSK2 – Pin Change Mask Register 2 => want pin PCINT21
  PCIFR  |= bit (PCIF2);   // PCIFR – Pin Change Interrupt Flag Register => Bit 2 – PCIF2: clear any outstanding interrupts 2
  PCICR  |= bit (PCIE2);   // PCICR – Pin Change Interrupt Control Register => Bit 2 – PCIE2: enable pin change interrupts 2
  DDRK  &= B11000000; // DDRK – Port K Data Direction Register => set pins 0-5 of PORTK as input 
  PORTK |= B00111111; // activate pull-up resitors on pins 8 to 13 of PORTK in order to avoid random values
  uint8_t p = PINK;          // read PortB to clear any mismatch
  
  rcValue[THROTTLE] = MINPPM;
  rcValue[ROLL]     = MIDPPM;
  rcValue[PITCH]    = MIDPPM;
  rcValue[YAW]      = MIDPPM;
  rcValue[AUX1]     = MINPPM;
  rcValue[AUX2]     = MINPPM;
  
  // compute linear coefficients before and after the neutral band.
  _a1 = MAXSPEED / (MIDPPM - NEUTRALBAND - MINPPM);
  _b1 = -MAXSPEED - (_a1 * MINPPM);
  _a2 = MAXSPEED / (MAXPPM - MIDPPM + NEUTRALBAND);
  _b2 = -_a2 * (MIDPPM + NEUTRALBAND);
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
        if (RC_data > MAXPPM) RC_data = MAXPPM; 
        if (RC_data < MINPPM) RC_data = MINPPM; 
            
        if ((i == ROLL) || (i == PITCH)) { 
           if ((RC_data > MIDPPM+NEUTRALBAND) || (RC_data < MIDPPM-NEUTRALBAND)) RC_command[i] = (int16_t)(RC_data - MIDPPM)* 45.0 * 2.0 /(MAXPPM -MINPPM); // roll, pitch converted to range [-45;+45]
           else                                                                  RC_command[i] = 0; 
        }     
        else if (i == YAW) { 
           if ((RC_data > MIDPPM+NEUTRALBAND) || (RC_data < MIDPPM-NEUTRALBAND)) RC_command[i] = (int16_t)(RC_data - MIDPPM)* 90.0 * 2.0 /(MAXPPM -MINPPM); // yaw converted to range [-90;+90] 
           else                                                                  RC_command[i] = 0; 
        }           
        else if (i == THROTTLE) { if (RC_data < 1.1*MINPPM) RC_command[i] = 0; else RC_command[i] = (int16_t)RC_data;}           // throttle = PPM [1100;2000]
        else if (i == AUX1) { if (RC_data > 0.9*MAXPPM)     RC_command[i] = 1; else RC_command[i] = 0;}                          // aux1
        else if (i == AUX2) { if (RC_data > 0.9*MAXPPMAUX2) RC_command[i] = 1; else RC_command[i] = 0;}                          // aux2    
  } // end for

}

void RCClass::RC_getAngularSpeedCommands(double RC_angularspeedcommand[NBCHANNELS])
{
    
  double RC_data;
  
  for (int i = 0; i < NBCHANNELS; i++) {         // read data from all channels
        RC_data = (double)RC_readRaw(i);
        
        if ((i == ROLL) || (i == PITCH) || (i == YAW)) {
            if (RC_data > MIDPPM+NEUTRALBAND)      RC_angularspeedcommand[i] = (_a1 * RC_data) + _b1;
            else if (RC_data < MIDPPM-NEUTRALBAND) RC_angularspeedcommand[i] = (_a2 * RC_data) + _b2;
            else                                   RC_angularspeedcommand[i] = 0.0;    
        }
        else if (i == THROTTLE) { if (RC_data < 1.1*MINPPM) RC_angularspeedcommand[i] = 0.0; else RC_angularspeedcommand[i] = RC_data;}           // throttle 
        else if (i == AUX1) { if (RC_data > 0.9*MAXPPM)     RC_angularspeedcommand[i] = 1; else RC_angularspeedcommand[i] = 0;}                          // aux1
        else if (i == AUX2) { if (RC_data > 0.9*MAXPPMAUX2) RC_angularspeedcommand[i] = 1; else RC_angularspeedcommand[i] = 0;}                          // aux2    
  } // end for

  // change signes
  RC_angularspeedcommand[PITCH] = -RC_angularspeedcommand[PITCH];
  
}

double RCClass::RC_get_a1()
{
  return _a1;
}
double RCClass::RC_get_b1()
{
  return _b1;
}
double RCClass::RC_get_a2()
{
  return _a2;
}
double RCClass::RC_get_b2()
{
  return _b2;
}