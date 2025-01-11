#ifndef PWM_h
#define PWM_h

//#define MEASURE_ON

#define DDRx  DDRA
#define PORTx PORTA  // PORT A => PA0:D22, PA1:D23, PA2:D24, PA3:D25

#define LONGTICK 8000  //  4 ms (1 ms => 2000 ticks)
#define TRIMTICK 26    // 13 us (1 us => 2 ticks)
#define MINPWM 1000
#define MIN_PULSE_SERVO       544     // the shortest pulse sent to a servo
#define MAX_PULSE_SERVO      2400     // the longest pulse sent to a servo

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to ticks (assumes prescaler of 8) 


class PWMClass
{
  public:
   
   PWMClass();
   
   void PWMInit();
   void writeServo (uint8_t index, uint8_t degree);
   void writeESC (uint8_t index, uint32_t duration_us);
#ifdef MEASURE_ON   
   uint8_t get_Nb_tick_Low();
   uint8_t get_Nb_tick_High();
#endif
   
   private:
};

#endif