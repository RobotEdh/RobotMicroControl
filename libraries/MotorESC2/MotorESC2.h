#ifndef MotorESC2_h
#define MotorESC2_h

#include <Arduino.h>
#include <RC.h>

#ifdef SERVO 
#include <Servo.h>

#define Motor1Pin 5  //FRONT_RIGHT
#define Motor2Pin 6  //REAR_RIGHT
#define Motor3Pin 7  //REAR_LEFT
#define Motor4Pin 8  //FRONT_LEFT
#endif


#define MINPWM 1000
#define MAXPWM 2000
#define MAXPWMTHRO 1700


#define NBMOTORS 4

class MotorESC2Class
{
  public:
   
   MotorESC2Class();
   
   void MotorESC_init(int led);
   void MotorESC_MixPID(int16_t ESC_command[4], uint16_t tick);
   void MotorESC_runMotors(int8_t no, uint16_t value);
   void MotorESC_pulsePWM(uint32_t duration);
      
   private:
   void MotorESC_sendPWMtoESC();
   void MotorESC_pulsePWM();

   uint16_t _motor[NBMOTORS];
};

#endif