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

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) //Code in here will only be compiled if an Arduino Uno (or older) is used.
 #define Motor1Pin 3
 #define Motor2Pin 5
 #define Motor3Pin 6
 #define Motor4Pin 9  // this PIN 9 is part of PORTB used alsoe for RC => ESC and RC can't used simultanously with Arduino Uno
#else
 #define Motor1Pin 3
 #define Motor2Pin 4
 #define Motor3Pin 5
 #define Motor4Pin 6
#endif


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