#ifndef MotorESC_h
#define MotorESC_h

#include <Arduino.h>
#include <RC.h>

#define STOPPWM 127
#define MINPWM 140
#define MAXPWM 255
#define MAXPWMTHRO 230

#define NBMOTORS 4
#define Motor1Pin 5  //FRONT_LEFT
#define Motor2Pin 6  //FRONT_RIGHT
#define Motor3Pin 7  //REAR_RIGHT
#define Motor4Pin 8  //REAR_LEFT

#define LED_PIN         13 // LED for init, pint 13 for MEGA2560


#define MOTORLOGDATASIZE 170 //multiple of block size 512 minus 4 bytes for Start and Stop
#define MOTORLOGFREQ 5 //log every 5 cycles

const uint8_t startMotorLog[2]={0xFB,0xFC};
const uint8_t stopMotorLog[2] ={0xFD,0xFE};  

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