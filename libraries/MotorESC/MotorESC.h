#ifndef MotorESC_h
#define MotorESC_h

#include <Arduino.h>

/* this is the value for the ESCs when they are not armed */
#define MINCOMMAND  936
#define MAXCOMMAND 1875

#define MINCHECK 1200

#define Motor1Pin 1
#define Motor2Pin 1
#define Motor3Pin 1
#define Motor4Pin 1

class MotorESCClass
{
  public:

  enum rc {
     ROLL,
     PITCH,
     YAW,
     THROTTLE
   };
   
   MotorESCClass();
   
   void MotorESC_init(void);
   void MotorESC_writeOneMotor(uint8_t no, int16_t value);   
   void MotorESC_writeAllMotors(int16_t value);
   void MotorESC_RunMotors(int16_t roll, int16_t pitch, int16_t yaw, int16_t throttle);
    
  private:
  void MotorESC_writeMotors(void);
  int16_t _motor[4];  // 4 motors
};

#endif