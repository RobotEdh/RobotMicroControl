#ifndef MotorESC_h
#define MotorESC_h

#include <Arduino.h>
#include <Servo.h>
#include <RC.h>

#define MINPWM 1000
#define MAXPWM 2000
#define MINPWMTHRO 1150
#define MAXPWMTHRO 1850
#define MAXABSPWMPID 200 // to be TESTED
#define MAXABSPID 79 // computed based on value of the coeff ans max error for PID


#define NBMOTORS 4
#define Motor1Pin 5  //FRONT_LEFT
#define Motor2Pin 6  //FRONT_RIGHT
#define Motor3Pin 7  //REAR_RIGHT
#define Motor4Pin 8  //REAR_LEFT

#define LED_PIN         13 // LED for init, pint 13 for MEGA2560

#define MOTORLOGFREQ 1 //record every 5 ticks ie 100 ms at 50Hz

class MotorESCClass
{
  public:
   
   MotorESCClass();
   
   void MotorESC_init(int16_t value, int8_t motor);
   void MotorESC_init();
   void MotorESC_test(void); 
   void MotorESC_writeOneMotor(uint8_t no, int16_t value);
   void MotorESC_writeAllMotors(int16_t value);
   void MotorESC_RunMotors(int16_t ESC_command[4], uint32_t tick);
   
   private:
    
   void MotorESC_writeMotors(void);
   int16_t _motor[NBMOTORS];
};

#endif