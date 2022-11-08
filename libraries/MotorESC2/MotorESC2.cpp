#include <Arduino.h>
#include <MotorESC2.h>
#include <Servo.h>

// Logging mode
#define  LOGSERIAL
//#define LOGSDCARD  // log to SD Card
//#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
#include <log.h>
//File logFile; 


Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

struct motor_record_type  // 14 bytes
{
     int16_t throttle;
     int16_t motor0;
     int16_t motor1;
     int16_t motor2;
     int16_t motor3;
     uint32_t tick;     
};
#define MOTORLOGDATASIZE 36 //multiple of block size 512 (struct 14 bytes * 36 = 504 + 8 bytes start/stop)
struct motor_record_block_type {  //(struct 14 bytes
     const uint8_t startMotorLog[2]={0xFB,0xFC};
     motor_record_type motor_record[MOTORLOGDATASIZE];
     const uint8_t stopMotorLog[6] ={0xFD,0xFE,0xFE,0xFE,0xFE,0xFE};  
};
motor_record_block_type motor_record_block;
int motor_t = 0;  

    
MotorESC2Class::MotorESC2Class()
{
}
        
void MotorESC2Class::MotorESC_init()
{ 
  
  PRINTs(">Start MotorESC_init")
  
  Motor1.attach(Motor1Pin, MINPWM, MAXPWM); 
  Motor2.attach(Motor2Pin, MINPWM, MAXPWM); 
  Motor3.attach(Motor3Pin, MINPWM, MAXPWM); 
  Motor4.attach(Motor4Pin, MINPWM, MAXPWM);

#ifdef LOGSERIAL
  uint8_t it = 0;
  it = Motor1.itimer(); 
  PRINT("Timer Motor 1: ", it)
  it = Motor2.itimer(); 
  PRINT("Timer Motor 2: ", it)
  it = Motor3.itimer(); 
  PRINT("Timer Motor 3: ", it)
  it = Motor4.itimer(); 
  PRINT("Timer Motor 4: ", it)  
#endif 
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // turn on Led
  
  MotorESC_runMotors(-1, MINPWM);

  delay (15*1000);  //delay 15s to connect battery
  
  digitalWrite(LED_PIN, LOW);   // turn off Led
  
  PRINTs("<End MotorESC_init")

} 

/**************************************************************************************/
/************          Send the PWM command to ESC                   ******************/
/**************************************************************************************/

void MotorESC2Class::MotorESC_sendPWMtoESC()
{ 
  Motor1.writeMicroseconds(_motor[0]);
  Motor2.writeMicroseconds(_motor[1]);    
  Motor3.writeMicroseconds(_motor[2]);
  Motor4.writeMicroseconds(_motor[3]); 
}  
     
/**************************************************************************************/
/************          Run one or all Motors                         ******************/
/**************************************************************************************/
void MotorESC2Class::MotorESC_runMotors(int8_t no, int16_t value)    // Sends same commands to one or all Motors 
{ 
 
  for (int i=0;i<NBMOTORS;i++) {
    if ((no == i) || (no == -1)) _motor[i]=value;
    else                         _motor[i]=MINPWM;
  }
  
MotorESC_sendPWMtoESC();
}

/**************************************************************************************/
/************          Run Motors according PID                      ******************/
/**************************************************************************************/
void MotorESC2Class::MotorESC_MixPID(int16_t ESC_command[4], uint32_t tick)
{
  int i;
  int count = -1;
  
  if (ESC_command[THROTTLE] == 0) 
  { 
     for(i=0; i< NBMOTORS; i++) _motor[i] = MINPWM; //stop
#ifdef LOGSDCARD     
	 if (motor_t > 0) { // force dump
             motor_record_block.motor_record[motor_t].tick = tick;
             motor_record_block.motor_record[motor_t].throttle = 0;
             motor_record_block.motor_record[motor_t].motor0 = _motor[0];
             motor_record_block.motor_record[motor_t].motor1 = _motor[1];
             motor_record_block.motor_record[motor_t].motor2 = _motor[2];
             motor_record_block.motor_record[motor_t].motor3 = _motor[3]; 
             motor_t++; 
             for(int z=motor_t; z< MOTORLOGDATASIZE; z++) motor_record_block.motor_record[z].tick = 0;// reset end tab
             count = logFile.write((const uint8_t *)&motor_record_block, 512);
             if (count != 512) PRINTi2("bad count written MOTOR following force dump: ",tick,count) 
             Serial.print("written MOTOR following force dump, tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(count);                
             motor_t = 0;                                           
     }
#endif	 
  }
  else
  {   
     if (ESC_command[THROTTLE] > MAXPWMTHRO) ESC_command[THROTTLE] = MAXPWMTHRO;
#ifdef LOGSERIAL
       PRINT("throttle", ESC_command[THROTTLE])
#endif  

     // assure consistency between the impact of the throttle and of the PID
     int16_t pidmix = 0;
     #define PIDMIX(X,Y,Z) ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z + ESC_command[THROTTLE]
     
     pidmix = PIDMIX(-1,+1,+1); //Front Right
     _motor[0] = constrain(pidmix, MINPWM, MAXPWM); 
#ifdef LOGSERIAL
       PRINTi2("motor",0,_motor[0]) 
       PRINT("pidmix",pidmix) 
#endif     

     pidmix =PIDMIX(-1,-1,-1); //Rear Right
     _motor[1] = constrain(pidmix, MINPWM, MAXPWM);  
#ifdef LOGSERIAL
       PRINTi2("motor",1,_motor[1]) 
       PRINT("pidmix",pidmix) 
#endif         

     pidmix = PIDMIX(+1,-1,+1); //Rear Left
     _motor[2] = constrain(pidmix, MINPWM, MAXPWM);    
#ifdef LOGSERIAL
       PRINTi2("motor",2,_motor[2]) 
       PRINT("pidmix",pidmix) 
#endif         

     pidmix = PIDMIX(+1,+1,-1); //Front Left
     _motor[3] = constrain(pidmix, MINPWM, MAXPWM); 
 #ifdef LOGSERIAL
       PRINTi2("motor",3,_motor[3]) 
       PRINT("pidmix",pidmix) 
#endif        

#ifdef LOGSDCARD
    if ((tick%MOTORLOGFREQ) == 0 ) { // record every 5 times ie 100 ms at 50Hz
          motor_record_block.motor_record[motor_t].throttle = ESC_command[THROTTLE];
          motor_record_block.motor_record[motor_t].motor0 = _motor[0];
          motor_record_block.motor_record[motor_t].motor1 = _motor[1];
          motor_record_block.motor_record[motor_t].motor2 = _motor[2];
          motor_record_block.motor_record[motor_t].motor3 = _motor[3];
          motor_record_block.motor_record[motor_t].tick = tick;                    
          motor_t++;
          if (motor_t == MOTORLOGDATASIZE) { // need to dump
             count = logFile.write((const uint8_t *)&motor_record_block, 512);
             if (count != 512) PRINTi2("bad count written MOTOR following need to dump",tick,count)
             Serial.print("written MOTOR following need to dump, tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(count);            
             motor_t = 0;                                             
          }
    } 
#endif
      
  }
  
  MotorESC_sendPWMtoESC (); 
}
