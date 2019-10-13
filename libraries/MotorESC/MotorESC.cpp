#include <Arduino.h>
#include <Servo.h>
#include <MotorESC.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
//#define LOGTRACE   // Enable trace
#include <log.h>
File logFile; 


Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

struct motor_record_type  // 9 bytes
{
     uint8_t throttle;
     uint8_t motor0;
     uint8_t motor1;
     uint8_t motor2;
     uint8_t motor3;
     uint32_t tick;     
};
#define MOTORLOGDATASIZE 56 //multiple of block size 512 (struct 9 bytes * 56 = 504 + 8 bytes start/stop)
struct motor_record_block_type {  //(struct 9 bytes
     const uint8_t startMotorLog[2]={0xFB,0xFC};
     motor_record_type motor_record[MOTORLOGDATASIZE];
     const uint8_t stopMotorLog[6] ={0xFD,0xFE,0xFE,0xFE,0xFE,0xFE};  
};
motor_record_block_type motor_record_block;
int motor_t = 0;  

MotorESCClass::MotorESCClass()
{
}
        
void MotorESCClass::MotorESC_init(int16_t value, int8_t motor)
{ 
  PRINTs(">Start MotorESC_init")

  pinMode(LED_PIN, OUTPUT);
  
  switch (motor) {
  case -1:
      Motor1.attach(Motor1Pin, STOPPWM, MAXPWM); 
      Motor2.attach(Motor2Pin, STOPPWM, MAXPWM); 
      Motor3.attach(Motor3Pin, STOPPWM, MAXPWM); 
      Motor4.attach(Motor4Pin, STOPPWM, MAXPWM); 
      
      MotorESC_writeAllMotors(value);
    break;
  case 0:
    Motor1.attach(Motor1Pin, STOPPWM, MAXPWM);
    MotorESC_writeOneMotor (0, value);
    break;
  case 1:
    Motor2.attach(Motor2Pin, STOPPWM, MAXPWM);
    MotorESC_writeOneMotor (1, value);
    break;
  case 2:
    Motor3.attach(Motor3Pin, STOPPWM, MAXPWM);
    MotorESC_writeOneMotor (2, value);
    break;
   case 3:
    Motor4.attach(Motor4Pin, STOPPWM, MAXPWM);
    MotorESC_writeOneMotor (3, value);
    break;       
  }
    
  digitalWrite(LED_PIN, HIGH);  // turn on Led for 15s
  if (value == STOPPWM) delay(15*1000); /* 15s to connect the ESC to power */
  digitalWrite(LED_PIN, LOW);  // turn on Led
  
  PRINTs("<End MotorESC_init")
}

void MotorESCClass::MotorESC_init()
{ 
    MotorESC_init(STOPPWM, -1);
}    
     
void MotorESCClass::MotorESC_writeMotors ()
{ 
  Motor1.writeMicroseconds(_motor[0]);
  Motor2.writeMicroseconds(_motor[1]);    
  Motor3.writeMicroseconds(_motor[2]);
  Motor4.writeMicroseconds(_motor[3]); 
}
  
/**************************************************************************************/
/************          Writes the command to one Motor               ******************/
/**************************************************************************************/
void MotorESCClass::MotorESC_writeOneMotor(uint8_t no, int16_t value) {   // Sends commands to one motor
  _motor[no]=value;
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

/**************************************************************************************/
/************          Run Motors according PID                      ******************/
/**************************************************************************************/
void MotorESCClass::MotorESC_RunMotors(int16_t ESC_command[4], uint32_t tick)
{
  int16_t maxMotor = 0;
  int16_t minMotor = 0;
  int i;
  int count = -1;
  
  if (ESC_command[THROTTLE] == 0) 
  { 
     for(i=0; i< NBMOTORS; i++) _motor[i] = STOPPWM; //stop
#ifdef LOGSDCARD     
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
#endif	 
  }
  else
  {   
     int16_t throttle = map(ESC_command[THROTTLE], MINPPM, MAXPPM, MINPWMTHRO, MAXPWMTHRO);  // MINPWMTHRO,  MAXPWMTHRO to give room for PID adjustment
     
     #define PIDMIX(X,Y,Z) ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z + throttle
    _motor[0] = PIDMIX(-1,-1,-1); //Front Left
    _motor[1] = PIDMIX(+1,-1,+1); //Front Right
    _motor[2] = PIDMIX(+1,+1,-1); //Rear Right
    _motor[3] = PIDMIX(-1,+1,+1); //Rear Left

    for(i=0; i< NBMOTORS; i++) {
#ifdef LOGSERIAL
       PRINTi2("motor",i,_motor[i]) 
#endif
       if((_motor[i]-MAXPWM) > maxMotor) maxMotor = _motor[i]-MAXPWM;
       if((MINPWM - _motor[i]) > minMotor) minMotor = MINPWM - _motor[i];
    }
#ifdef LOGSDCARD
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
#ifdef LOGSDCARD
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
#endif         
    } // end if ((minMotor > 0) || (maxMotor > 0))
      
  } 

  MotorESC_writeMotors();
}