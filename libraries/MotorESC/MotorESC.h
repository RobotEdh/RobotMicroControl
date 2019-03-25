#ifndef MotorESC_h
#define MotorESC_h

#include <Arduino.h>
#include <RC.h>

#define MINPWM 127
#define MAXPWM 255
#define MAXPWMTHRO 200

#define NBMOTORS 4
const char szMotors[NBMOTORS][20]={    
"FRONT_LEFT",
"FRONT_RIGHT",
"REAR_RIGHT",
"REAR_LEFT"
};
#define Motor1Pin 5  //FRONT_LEFT
#define Motor2Pin 6  //FRONT_RIGHT
#define Motor3Pin 7  //REAR_RIGHT
#define Motor4Pin 8  //REAR_LEFT


class MotorESCClass
{
  public:
   
   MotorESCClass();
   
   void MotorESC_init(void);
   void MotorESC_test(void);
   void MotorESC_writeOneMotor(uint8_t no, int16_t value);   
   void MotorESC_writeAllMotors(int16_t value);
   void MotorESC_RunMotors(int16_t ESC_command[4]);
    
  private:
  void MotorESC_writeMotors(void);
  int16_t _motor[NBMOTORS];
};

#endif