#ifndef MotorESC2_h
#define MotorESC2_h

#include <Arduino.h>
#include <RC.h>
#include <Servo.h>

#define MINPWM 1000
#define MAXPWM 2000
#define MAXPWMTHRO 1700

#define Motor1Pin 5  //FRONT_RIGHT
#define Motor2Pin 6  //REAR_RIGHT
#define Motor3Pin 7  //REAR_LEFT
#define Motor4Pin 8  //FRONT_LEFT

#define NBMOTORS 4

#define LED_PIN         13 // LED for init, pint 13 for MEGA2560

#define MOTORLOGFREQ 1 //record every 5 ticks ie 100 ms at 50Hz

class MotorESC2Class
{
  public:
   
   MotorESC2Class();
   
   void MotorESC_init();
   void MotorESC_MixPID(int16_t ESC_command[4], uint32_t tick);
   void MotorESC_runMotors(int8_t no, int16_t value);
   
   private:
   void MotorESC_sendPWMtoESC(); 
   int16_t _motor[NBMOTORS];
};

#endif