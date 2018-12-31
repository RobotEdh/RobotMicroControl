#ifndef MotorESC_h
#define MotorESC_h

#include <Arduino.h>
#include <RC.h>

#define MINPWM   0
#define MAXPWM 255

#define NBMOTORS 4
const char szMotors[NBMOTORS][20]={    
"REAR_RIGHT",
"FRONT_RIGHT",
"REAR_LEFT",
"FRONT_LEFT"
};

#define Motor1Pin 1
#define Motor2Pin 1
#define Motor3Pin 1
#define Motor4Pin 1

class MotorESCClass
{
  public:
   
   MotorESCClass();
   
   void MotorESC_init(void);
   void MotorESC_writeOneMotor(uint8_t no, int16_t value);   
   void MotorESC_writeAllMotors(int16_t value);
   void MotorESC_RunMotors(int16_t ESC_command[4]);
    
  private:
  void MotorESC_writeMotors(void);
  int16_t _motor[NBMOTORS];
};

#endif