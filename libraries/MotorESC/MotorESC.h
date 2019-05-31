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


#define MOTORLOGDATASIZE 113 //multiple of block size 512 (struct 9 bytes * 113 = 1017 + 7 bytes start/stop)
#define MOTORLOGFREQ 1 //record every 5 ticks ie 100 ms at 50Hz

const uint8_t startMotorLog[2]={0xFB,0xFC};
const uint8_t stopMotorLog[5] ={0xFD,0xFE,0xFE,0xFE,0xFE};  

class MotorESCClass
{
  public:
   
   MotorESCClass();
   
   void MotorESC_init(void);
   void MotorESC_test(void);
   void MotorESC_writeOneMotor(uint8_t no, int16_t value);   
   void MotorESC_writeAllMotors(int16_t value);
   void MotorESC_RunMotors(int16_t ESC_command[4], uint32_t tick);
    
  private:
  void MotorESC_writeMotors(void);
  int16_t _motor[NBMOTORS];
};

#endif