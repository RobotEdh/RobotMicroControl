#ifndef PWM_h
#define PWM_h

//#define MEASURE_ON

#ifdef ARDUINO_AVR_MEGA2560 
#define DDRx  DDRA
#define PORTx PORTA  // PORT A => PA0:D22, PA1:D23, PA2:D24, PA3:D25

#define LONGTICK 8000  //  4 ms/250HZ => 2000 ticks)
#define TRIMTICK 26    // 13 us (1 us => 2 ticks)


#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to ticks (assumes prescaler of 8) 

#else

#define PWM_PIN_MOTOR_0 0   
#define PWM_PIN_MOTOR_1 1
#define PWM_PIN_MOTOR_2 2   
#define PWM_PIN_MOTOR_3 3
const uint8_t pwm_pins[4]={PWM_PIN_MOTOR_0,PWM_PIN_MOTOR_1,PWM_PIN_MOTOR_2,PWM_PIN_MOTOR_3};


#define PWM_RESOLUTION          16      // Set duty resolution to 16 bits
#define PWM_FREQUENCY           400.0    // Frequency in Hertz. Set frequency at 400Hz
#define PWM_TIMER_WIDTH_TICKS 65536.0   // 2**PWM_RESOLUTION)

#define usToTicks(_us)    ((double)_us * PWM_FREQUENCY * PWM_TIMER_WIDTH_TICKS/1000000.0)     // converts microseconds to ticks

#endif

#define MINPWM 1000

#define MIN_PULSE_SERVO       544     // the shortest pulse sent to a servo
#define MAX_PULSE_SERVO      2400     // the longest pulse sent to a servo

class PWMClass
{
  public:
   
   PWMClass();
   
   bool PWMInit();
   bool writeServo (uint8_t index, uint8_t degree);
   bool writeESC (uint8_t index, uint32_t duration_us);
#ifdef MEASURE_ON   
   uint8_t get_Nb_tick_Low();
   uint8_t get_Nb_tick_High();
#endif
   
   private:
};

#endif