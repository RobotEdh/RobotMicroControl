#include <Arduino.h>
#include <MotorESC2.h>

//#define SERVO 
#ifdef SERVO 
#include <Servo.h>
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;
#endif

#define DEBUGLEVEL1
#ifdef  DEBUGLEVEL0
 #define  DEBUGLEVEL1
#endif
 
// Logging mode
#define  LOGSERIAL
//#define LOGSDCARD  // log to SD Card
#include <log.h>

#ifdef  LOGSDCARD
#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
extern File logFile;
extern int countwrite;  

#define MOTORLOGFREQ 1 //record every 5 ticks ie 100 ms at 50Hz

struct motor_record_type  // 12 bytes
{
     uint16_t throttle;
     uint16_t motor0;
     uint16_t motor1;
     uint16_t motor2;
     uint16_t motor3;
     uint16_t tick;     
};
#define MOTORLOGDATANB 42 //multiple of block size 512 (struct 12 bytes * 42 = 504 + 8 bytes start/stop = 512)
struct motor_record_block_type {  //(struct 8 bytes
     const uint8_t startMotorLog[2]={0xFB,0xFC};
     motor_record_type motor_record[MOTORLOGDATANB  ];
     const uint8_t stopMotorLog[6] ={0xFD,0xFE,0xFE,0xFE,0xFE,0xFE};  
};
motor_record_block_type motor_record_block;
int motor_t = 0;  
#endif



    
MotorESC2Class::MotorESC2Class()
{
}
     
void MotorESC2Class::MotorESC_init()
{ 
  
  PRINTs(">Start MotorESC_init")

#ifdef SERVO  
  Motor1.attach(Motor1Pin, MINPWM, MAXPWM); 
  Motor2.attach(Motor2Pin, MINPWM, MAXPWM); 
  Motor3.attach(Motor3Pin, MINPWM, MAXPWM); 
  Motor4.attach(Motor4Pin, MINPWM, MAXPWM);
#else
  PORTL &= B00001111;   //L4 pin 45 ... L7 pin 42 set to 0
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
#ifdef SERVO  
  Motor1.writeMicroseconds(_motor[0]);
  Motor2.writeMicroseconds(_motor[1]);    
  Motor3.writeMicroseconds(_motor[2]);
  Motor4.writeMicroseconds(_motor[3]); 
  
#else  
  MotorESC_pulsePWM();  
#endif    
}  

/**************************************************************************************/
/************          Send the PWM command to ESC                   ******************/
/**************************************************************************************/

void MotorESC2Class::MotorESC_pulsePWM()
{
 
#ifndef SERVO  

  uint32_t current_time, ESC_pulse_start_time, ESC_pulse_end_time[4];
  
  ESC_pulse_start_time = micros();  // This number will overflow (go back to zero), after approximately 70 minutes.
                                    // On 16 MHz Arduino boards, this function has a resolution of four microseconds                                      

  PORTL |= B11110000; // all outputs connected to the ESCs are set to "1"

  ESC_pulse_end_time[0] = ESC_pulse_start_time + _motor[0];
  ESC_pulse_end_time[1] = ESC_pulse_start_time + _motor[1];
  ESC_pulse_end_time[2] = ESC_pulse_start_time + _motor[2];
  ESC_pulse_end_time[3] = ESC_pulse_start_time + _motor[3];
  
  while(PORTL >=16)
  {
        current_time = micros();
        if(ESC_pulse_end_time[0] <= current_time) PORTL &= B11101111; // Port L4 pin 45
        if(ESC_pulse_end_time[1] <= current_time) PORTL &= B11011111; // Port L5 pin 44
        if(ESC_pulse_end_time[2] <= current_time) PORTL &= B10111111; // Port L6 pin 43
        if(ESC_pulse_end_time[3] <= current_time) PORTL &= B01111111; // Port L7 pin 42   
 }
 delay(2);               // waiting 2 milliseconds before the next loop, i.e. a PWM frequency of about 250HZ
  
#endif    
}  
     
/**************************************************************************************/
/************          Run one or all Motors                         ******************/
/**************************************************************************************/
void MotorESC2Class::MotorESC_runMotors(int8_t no, uint16_t value)    // Sends same commands to one or all Motors 
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
void MotorESC2Class::MotorESC_MixPID(int16_t ESC_command[4], uint16_t tick)
{
  uint16_t overflow[4];
  uint16_t maxMotor = 0;

  if (ESC_command[THROTTLE] == 0) 
  { 
     for(int i=0; i< NBMOTORS; i++) _motor[i] = MINPWM; //stop
#ifdef LOGSDCARD     
	 if (motor_t > 0) { // force dump
             motor_record_block.motor_record[motor_t].throttle = 0;
             motor_record_block.motor_record[motor_t].motor0 = _motor[0];
             motor_record_block.motor_record[motor_t].motor1 = _motor[1];
             motor_record_block.motor_record[motor_t].motor2 = _motor[2];
             motor_record_block.motor_record[motor_t].motor3 = _motor[3]; 
             motor_record_block.motor_record[motor_t].tick = tick;             
             motor_t++; 
             for(int z=motor_t; z< MOTORLOGDATANB; z++) motor_record_block.motor_record[z].tick = 0;// reset end tab
             countwrite = logFile.write((const uint8_t *)&motor_record_block, 512);
             motor_t = 0; 
#ifdef LOGSERIAL             
             if (countwrite != 512) PRINTi2("bad count written MOTOR following force dump: ",tick,countwrite) 
             Serial.print("written MOTOR following force dump, tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(countwrite);                
#endif	                                                       
     }
#endif	 
  }
  else
  {   
     if (ESC_command[THROTTLE] > MAXPWMTHRO) ESC_command[THROTTLE] = MAXPWMTHRO;

     // assure consistency between the impact of the throttle and of the PID
     int16_t pidmix = 0;
     #define PIDMIX(X,Y,Z) ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z + ESC_command[THROTTLE]
     
     pidmix = PIDMIX(-1,+1,+1); //Front Right
     overflow[0] = max(0, pidmix - MAXPWM);
     _motor[0] = constrain(pidmix, MINPWM, MAXPWM); 
#ifdef DEBUGLEVEL0
#ifdef LOGSERIAL
       PRINTi2("motor",0,_motor[0]) 
       PRINT("pidmix",pidmix) 
#endif     
#endif
 
     pidmix =PIDMIX(-1,-1,-1); //Rear Right
     overflow[1] = max(0, pidmix - MAXPWM);
     _motor[1] = constrain(pidmix, MINPWM, MAXPWM);  
#ifdef DEBUGLEVEL0
#ifdef LOGSERIAL
       PRINTi2("motor",1,_motor[1]) 
       PRINT("pidmix",pidmix) 
#endif     
#endif         

     pidmix = PIDMIX(+1,-1,+1); //Rear Left
     overflow[2] = max(0, pidmix - MAXPWM);;
     _motor[2] = constrain(pidmix, MINPWM, MAXPWM);    
#ifdef DEBUGLEVEL0
#ifdef LOGSERIAL
       PRINTi2("motor",2,_motor[2]) 
       PRINT("pidmix",pidmix) 
#endif     
#endif       

     pidmix = PIDMIX(+1,+1,-1); //Front Left
     overflow[3] = max(0, pidmix - MAXPWM);
     _motor[3] = constrain(pidmix, MINPWM, MAXPWM); 
#ifdef DEBUGLEVEL0
#ifdef LOGSERIAL
       PRINTi2("motor",3,_motor[3]) 
       PRINT("pidmix",pidmix) 
#endif     
#endif       

     for(int k=0, maxMotor=0; k< NBMOTORS; k++) {
       if(overflow[k] > maxMotor) maxMotor = overflow[k];
     }
       
     if (maxMotor > 0) {      
       for(int l=0; l< NBMOTORS; l++) {
          _motor[l] = _motor[l] - maxMotor;
       }
#ifdef DEBUGLEVEL1
#ifdef LOGSERIAL
       PRINT("maxMotor",maxMotor) 
#endif     
#endif        
     } 
        
#ifdef DEBUGLEVEL0 
     delay(10000);
#endif 

 
#ifdef LOGSDCARD
    if ((tick%MOTORLOGFREQ) == 0 ) { // record every 5 times ie 100 ms at 50Hz
          motor_record_block.motor_record[motor_t].throttle = (uint16_t)ESC_command[THROTTLE];
          motor_record_block.motor_record[motor_t].motor0 = _motor[0];
          motor_record_block.motor_record[motor_t].motor1 = _motor[1];
          motor_record_block.motor_record[motor_t].motor2 = _motor[2];
          motor_record_block.motor_record[motor_t].motor3 = _motor[3];
          motor_record_block.motor_record[motor_t].tick = tick;                    
          motor_t++;
          if (motor_t == MOTORLOGDATANB) { // need to dump
             countwrite = logFile.write((const uint8_t *)&motor_record_block, 512);
             motor_t = 0; 
#ifdef LOGSERIAL             
             if (countwrite != 512) PRINTi2("bad count written MOTOR following need to dump",tick,countwrite)
             Serial.print("written MOTOR following need to dump, tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(countwrite);            
#endif                                                        
          }
    } 
#endif
      
  }
  
  MotorESC_sendPWMtoESC(); 
}

