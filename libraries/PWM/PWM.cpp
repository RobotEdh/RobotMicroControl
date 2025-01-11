#include <Arduino.h>

#include <PWM.h>

#ifdef MEASURE_ON
   volatile uint8_t Nb_tick_Low;
   volatile uint8_t Nb_tick_High;
#endif
  
uint32_t pwmUs[4];  

   
PWMClass::PWMClass()
{
}

ISR(TIMER5_COMPA_vect)  // Interrupt trigerred when TCNT5 = OCR5A
{ 
  static bool low_level = true;
  static int32_t tick;
  static int32_t pwmTick[4];

#ifdef MEASURE_ON  
  TCNT2 = 0;
#endif   
  if (low_level == true) //end low level
  {
     low_level = false;
     tick = LONGTICK;
     for (uint8_t i=0;i<3;i++) {                              
        pwmTick[i] = usToTicks(pwmUs[i]);          // convert pulse in us to tick
        if (pwmTick[i] < tick) tick = pwmTick[i];  // determine min pulse       
     }
#ifdef MEASURE_ON       
     Nb_tick_Low = TCNT2 ; 
#endif      
     PORTx |= B00001111;                               // Px0...Px3 Turned on
     OCR5A = TCNT5 + tick;                             // set next Timer5 interrupt compare 
  }
  else // high level in progress
  {
     uint32_t previoustick = tick+TRIMTICK;
     tick = LONGTICK;
     for (uint8_t i=0;i<3;i++) { 
        if (pwmTick [i] > TRIMTICK) {                // still high level
           pwmTick [i] -= previoustick;              // compute remaining tick
           if (pwmTick [i] < TRIMTICK){              // end pulse
              PORTx &= ~(1<<i);                      // Pxi Turned off
           }
           else {
              if (pwmTick[i] < tick) tick = pwmTick [i];  // determine min remaining pulse
           }                                                             
        }

     }   
     if (tick == LONGTICK) {  
        low_level = true;    // all channels low
        TCNT5 = 0;            // reset Timer5 register value to 0
     }
#ifdef MEASURE_ON       
     Nb_tick_High = TCNT2 ; 
#endif      
     OCR5A = TCNT5 + tick - TRIMTICK; // set next Timer5 interrupt compare, add a trim to avoid too close interrupt
  }   
}  

#ifdef MEASURE_ON   
uint8_t PWMClass::get_Nb_tick_Low()  {return Nb_tick_Low;}
uint8_t PWMClass::get_Nb_tick_High() {return Nb_tick_High;}
#endif
   
void PWMClass::PWMInit()
{        
#ifdef MEASURE_ON      
    TCCR2A = 0;  //normal mode
    TCCR2B = 1;  //Bit 2:0 ? CSn2:0: Clock Select (No prescaling)
    TIMSK2 = 0;  //no interrupt enable
#endif 
    DDRx |=  B00001111;   //Px0...Px3 set to 1 for output
    PORTx &= B11110000;  // Px0...Px3 Turned off
    
    for (uint8_t i=0;i<4;i++) pwmUs[i]= MINPWM;
    
    //TCCR5A ? Timer/Counter 5 Control Register A
    //Bit 1:0 ? WGM51:0: Waveform Generation Mode
    TCCR5A = 0;             //normal counting mode
    /*     Normal Mode
           The simplest mode of operation is the Normal mode (WGMn3:0 = 0). In this mode the counting direction is always
           up (incrementing), and no counter clear is performed. The counter simply overruns when it passes its maximum
           16-bit value (MAX = 0xFFFF = 65 535) and then restarts from the BOTTOM (0x0000). In normal operation the Timer/Counter Overflow Flag (TOVn)
           will be set in the same timer clock cycle as the TCNTn becomes zero. The TOVn Flag in this case behaves like a 17th bit,
           except that it is only set, not cleared. However, combined with the timer overflow interrupt that automatically clears the TOVn Flag,
           the timer resolution can be increased by software. There are no special cases to consider in the Normal mode,
           a new counter value can be written anytime.
           The Input Capture unit is easy to use in Normal mode. However, observe that the maximum interval between the
           external events must not exceed the resolution of the counter. If the interval between events are too long, the timer
           overflow interrupt or the prescaler must be used to extend the resolution for the capture unit.
    */    
    //TCCR5B ? Timer/Counter 5 Control Register B
    //Bit 2:0 ? CSn2:0: Clock Select
    TCCR5B = _BV(CS51);     // set prescaler of 8
    
    //The two Timer/Counter I/O locations (TCNTnH and TCNTnL, combined TCNTn) give direct access, both for read and for write operations, to the Timer/Counter unit 16-bit counte
    //Modifying the counter (TCNTn) while the counter is running introduces a risk of missing a compare match between TCNTn and one of the OCRnx Registers.
    //Writing to the TCNTn Register blocks (removes) the compare match on the following timer clock for all compare units.
    TCNT5 = 0;              // clear the timer count
    
    OCR5A = LONGTICK; 
    
    //TIFR5 ? Timer/Counter5 Interrupt Flag Register
    //Bit 1 ? OCF1A: Timer/Counter1, Output Compare A Match Flag
    //This flag is set in the timer clock cycle after the counter (TCNT5 value matches the Output Compare Register A (OCR5A).
    //OCF5A can be cleared by writing a logic one to its bit location.
    TIFR5 = _BV(OCF5A);     // clear any pending interrupts
    
    //TIMSK5 ? Timer/Counter 5 Interrupt Mask Register
    //Bit 1 ? OCIE5A: Timer/Countern, Output Compare A Match Interrupt Enable
    TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt
}

void PWMClass::writeServo (uint8_t index, uint8_t degree) {
  if (degree > 180) degree = 180;  
  pwmUs[index] = map(degree,0,180,MIN_PULSE_SERVO,MAX_PULSE_SERVO);
}

void PWMClass::writeESC (uint8_t index, uint32_t duration_us) {  
  pwmUs[index] = duration_us;
}
