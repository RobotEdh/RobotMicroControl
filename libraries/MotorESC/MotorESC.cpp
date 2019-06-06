#include <Arduino.h>
#include <MotorESC.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
//#define LOGTRACE   // Enable trace
#include <log.h>
File logFile; 

struct motor_record_type  // 9 bytes
{
     uint8_t throttle;
     uint8_t motor0;
     uint8_t motor1;
     uint8_t motor2;
     uint8_t motor3;
     uint32_t tick;     
};

struct motor_record_block_type {  //(struct 9 bytes * 56 = 504 + 8 bytes start/stop)
     const uint8_t startMotorLog[2]={0xFB,0xFC};
     motor_record_type motor_record[MOTORLOGDATASIZE];
     const uint8_t stopMotorLog[6] ={0xFD,0xFE,0xFE,0xFE,0xFE,0xFE};  
};
motor_record_block_type motor_record_block;
int motor_t = 0;  

MotorESCClass::MotorESCClass()
{
}
        
void MotorESCClass::MotorESC_init()
{ 
  PRINTs(">Start MotorESC_init")

  pinMode(LED_PIN, OUTPUT);
    
  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
  MotorESC_writeAllMotors(STOPPWM);
    
  PRINTs("you have 15 s to connect the ESC to power...") 
  digitalWrite(LED_PIN, HIGH);  // turn on Led for 15s
  delay(15*1000); /* 15 s to connect the ESC to power */
  digitalWrite(LED_PIN, LOW);  // turn on Led
  PRINTs("... done")
  
  PRINTs("<End MotorESC_init")
}
 
void MotorESCClass::MotorESC_writeMotors ()
{ 
  analogWrite(Motor1Pin, _motor[0]);
  analogWrite(Motor2Pin, _motor[1]);    
  analogWrite(Motor3Pin, _motor[2]);
  analogWrite(Motor4Pin, _motor[3]); 
}
  
/**************************************************************************************/
/************          Writes the command to one Motor               ******************/
/**************************************************************************************/
void MotorESCClass::MotorESC_writeOneMotor(uint8_t no, int16_t value) {   // Sends commands to one motor
  for (int i=0;i<NBMOTORS;i++) {
    _motor[i]=STOPPWM;  // stop all motors...
  }
  _motor[no]=value; // ...except for the motor selected
   
  MotorESC_writeMotors();
}


/**************************************************************************************/
/************          Writes the command to all Motors              ******************/
/**************************************************************************************/
void MotorESCClass::MotorESC_writeAllMotors(int16_t value)    // Sends same commands to all motors
{ 
  for (int i=0;i<NBMOTORS;i++) {
    _motor[i]=value;
  }
  MotorESC_writeMotors();
}

// int16_t range [-180;+180]          for ROLL, PITCH, YAW
//               0 or [MINPPM;MAXPPM] for THROTTLE
void MotorESCClass::MotorESC_RunMotors(int16_t ESC_command[4], uint32_t tick)
{
  int16_t maxMotor = 0;
  int16_t minMotor = 0;
  int i;
  int count = -1;
  
  if (ESC_command[THROTTLE] == 0) 
  { 
     for(i=0; i< NBMOTORS; i++) _motor[i] = STOPPWM;
     if (motor_t > 0) { // force dump
             motor_record_block.motor_record[motor_t].tick = tick;
             motor_record_block.motor_record[motor_t].throttle = 0;
             motor_record_block.motor_record[motor_t].motor0 = (uint8_t)_motor[0];
             motor_record_block.motor_record[motor_t].motor1 = (uint8_t)_motor[1];
             motor_record_block.motor_record[motor_t].motor2 = (uint8_t)_motor[2];
             motor_record_block.motor_record[motor_t].motor3 = (uint8_t)_motor[3]; 
             motor_t++; 
             for(int z=motor_t; z< MOTORLOGDATASIZE+1; z++) motor_record_block.motor_record[motor_t].tick = 0;// reset end tab
             count = logFile.write((const uint8_t *)&motor_record_block, 512);
             if (count != 512) PRINT("bad count written: ",count);  
             motor_t = 0;                                           
     }
  }
  else
  {   
     int16_t throttle = map(ESC_command[THROTTLE], MINPPM, MAXPPM, MINPWM, MAXPWM);
     throttle = constrain(throttle, MINPWM, MAXPWMTHRO);  // to give room for PID ajustement
     
     #define PIDMIX(X,Y,Z) ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z
    _motor[0] = PIDMIX(-1,-1,-1); //Front Left
    _motor[1] = PIDMIX(+1,-1,+1); //Front Right
    _motor[2] = PIDMIX(+1,+1,-1); //Rear Right
    _motor[3] = PIDMIX(-1,+1,+1); //Rear Left

    for(i=0; i< NBMOTORS; i++) {
       _motor[i] = map(_motor[i], -90, 90, -(MAXPWM-MINPWM)/2, (MAXPWM-MINPWM)/2);
       _motor[i] = _motor[i] + throttle;
#ifdef LOGSERIAL
       PRINTi2("motor",i,_motor[i]) 
#endif
       if((_motor[i]-MAXPWM) > maxMotor) maxMotor = _motor[i]-MAXPWM;
       if((MINPWM - _motor[i]) > minMotor) minMotor = MINPWM - _motor[i];
    }
#ifndef LOGSERIAL
    if ((tick%MOTORLOGFREQ) == 0 ) { // record every 5 times ie 100 ms at 50Hz
          motor_record_block.motor_record[motor_t].throttle = (uint8_t)throttle;
          motor_record_block.motor_record[motor_t].motor0 = (uint8_t)_motor[0];
          motor_record_block.motor_record[motor_t].motor1 = (uint8_t)_motor[1];
          motor_record_block.motor_record[motor_t].motor2 = (uint8_t)_motor[2];
          motor_record_block.motor_record[motor_t].motor3 = (uint8_t)_motor[3];
          motor_record_block.motor_record[motor_t].tick = tick;                    
          motor_t++;
          if (motor_t == MOTORLOGDATASIZE) { // need to dump
             count = logFile.write((const uint8_t *)&motor_record_block, 512);
             if (count != 512) PRINTi2("bad count written: ",tick,count)
             motor_t = 0;                                             
          }
    } 
#endif
    
    if (maxMotor > 0) {
#ifdef LOGSERIAL        
       PRINT("maxMotor|",maxMotor)
#endif       
       for(i=0; i< NBMOTORS; i++) {
          _motor[i] = _motor[i] - maxMotor;
          if((MINPWM - _motor[i]) > minMotor) minMotor = MINPWM - _motor[i];
       }
    }
    
    if (minMotor > 0) {
#ifdef LOGSERIAL        
       PRINT("minMotor|",minMotor)
#endif        
       for(i=0; i< NBMOTORS; i++) {
          _motor[i] = _motor[i] + minMotor;
       }  
    }
    
    if ((minMotor > 0) || (maxMotor > 0)) {
       for(i=0; i< NBMOTORS; i++) {
          _motor[i] = constrain(_motor[i], MINPWM, MAXPWM);  // last cap if still needed after up and bottom cap
#ifdef LOGSERIAL          
          PRINTi2("motor last cap",i,_motor[i])
#endif          
       }
#ifndef LOGSERIAL
    if ((tick%MOTORLOGFREQ) == 0 ) { // record every 5 times ie 100 ms at 50Hz
          motor_record_block.motor_record[motor_t].throttle = (uint8_t)throttle;
          motor_record_block.motor_record[motor_t].motor0 = (uint8_t)_motor[0];
          motor_record_block.motor_record[motor_t].motor1 = (uint8_t)_motor[1];
          motor_record_block.motor_record[motor_t].motor2 = (uint8_t)_motor[2];
          motor_record_block.motor_record[motor_t].motor3 = (uint8_t)_motor[3];
          motor_record_block.motor_record[motor_t].tick = tick;                    
          motor_t++;
          if (motor_t == MOTORLOGDATASIZE) { // need to dump
             count = logFile.write((const uint8_t *)&motor_record_block, 512);
             if (count != 512) PRINTi2("bad count written: ",tick,count)
             motor_t = 0;                                             
          }
    } 
#endif         
    } // end if ((minMotor > 0) || (maxMotor > 0))
      
  } 

  MotorESC_writeMotors();
}