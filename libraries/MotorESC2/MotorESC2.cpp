#include <Arduino.h>

//#define SERVO 
#define HARDPWM 
//#define SOFTPWM 

#include <MotorESC2.h>

#ifdef SERVO 
#include <Servo.h>
Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;
#elif defined HARDPWM 
#include <PWM.h>
PWMClass PWM;
#endif

//#define DEBUGLEVEL0
//#define DEBUGLEVEL1
#ifdef  DEBUGLEVEL0
 #define  DEBUGLEVEL1
#endif
 
// Logging mode
#define  LOGSERIAL
//#define LOGSDCARD  // log to SD Card
//#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
#include <log.h>

#ifdef  LOGSDCARD
extern File logFile;
extern int countwrite;  

#define MOTORLOGFREQ 500 //record every 500 ticks ie 10s at 50Hz

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
#elif defined  HARDPWM
  PWM.PWMInit();  
#else
  DDRx |=  B00001111;   //Px0...Px3 set to 1 for output
  PORTx &= B11110000;  // Px0...Px3 Turned off
#endif
  
  PRINTs("<End MotorESC_init")
} 

void MotorESC2Class::MotorESC_power(int led, uint16_t duration)
{ 
  
  PRINTs(">Start MotorESC_power")
  
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);  // turn on led

  MotorESC_updateMotors(-1, MINPWM); // Update all motors
  
#ifdef SERVO   
  Motor1.writeMicroseconds(_motor[0]);
  Motor2.writeMicroseconds(_motor[1]);    
  Motor3.writeMicroseconds(_motor[2]);
  Motor4.writeMicroseconds(_motor[3]);

  // blink duration times the led during 1s; Time to connect the battery.
  for (uint16_t i=0;i<duration;i++){
        digitalWrite(led, HIGH);  // turn on led
        delay(500);
        digitalWrite(led, LOW);   // turn off led
        delay(500);  
  } 
#elif defined HARDPWM 
  PWM.writeESC(0, _motor[0]);
  PWM.writeESC(1, _motor[1]);
  PWM.writeESC(2, _motor[2]);
  PWM.writeESC(3, _motor[3]);
#else
  MotorESC_pulsePWM((uint32_t)duration*1000);  // PWM during duration seconds. Time to connect the battery.
#endif    
  digitalWrite(led, LOW);   // turn off Led
  
  PRINTs("<End MotorESC_power")
} 


/**************************************************************************************/
/************          Send the PWM command to ESC                   ******************/
/**************************************************************************************/

void MotorESC2Class::MotorESC_pulsePWM(uint32_t duration)
{
#ifdef SOFTPWM   
  uint32_t startTime = millis();
  
  if (duration == 0) { 
     MotorESC_pulsePWM();
     return;
  }
  
  while((millis() - startTime) < duration) {
          MotorESC_pulsePWM();
  }
  return;
#endif   
}

void MotorESC2Class::MotorESC_pulsePWM()
{
 
#ifdef SOFTPWM 
  uint32_t current_time, ESC_pulse_start_time, ESC_pulse_end_time[4];
  
  ESC_pulse_start_time = micros();  // This number will overflow (go back to zero), after approximately 70 minutes.
                                    // On 16 MHz Arduino boards, this function has a resolution of four microseconds                                      
  
  PORTx |= B00001111;  // Px0...Px3 Turned on

  ESC_pulse_end_time[0] = ESC_pulse_start_time + (uint32_t)_motor[0];
  ESC_pulse_end_time[1] = ESC_pulse_start_time + (uint32_t)_motor[1];
  ESC_pulse_end_time[2] = ESC_pulse_start_time + (uint32_t)_motor[2];
  ESC_pulse_end_time[3] = ESC_pulse_start_time + (uint32_t)_motor[3];

  while(PORTx&=B00001111)
  {
        current_time = micros();
        if(ESC_pulse_end_time[0] <= current_time) PORTx &= B11111110; // Px0 Turned off
        if(ESC_pulse_end_time[1] <= current_time) PORTx &= B11111101; // Px1 Turned off
        if(ESC_pulse_end_time[2] <= current_time) PORTx &= B11111011; // Px2 Turned off
        if(ESC_pulse_end_time[3] <= current_time) PORTx &= B11110111; // Px3 Turned off   
 }
 delay(5); // waiting 5 milliseconds before the next loop, i.e. a PWM frequency of about 200HZ
  
#endif    
}  
     
/**************************************************************************************/
/************          Run one or all Motors                         ******************/
/**************************************************************************************/
void MotorESC2Class::MotorESC_updateMotors(int8_t n, uint16_t value)    // Define same commands to one or all Motors 
{ 
 
  if (n == -1) {for (int i=0;i<NBMOTORS;i++) {_motor[i]=value;}}
  else                                       {_motor[n]=value;}
 
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
       if((uint16_t)overflow[k] > (uint16_t)maxMotor) maxMotor = overflow[k];
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
    if ((tick%MOTORLOGFREQ) == 0 ) { // record every MOTORLOGFREQ ticks
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
  
#ifdef SERVO   
  Motor1.writeMicroseconds(_motor[0]);
  Motor2.writeMicroseconds(_motor[1]);    
  Motor3.writeMicroseconds(_motor[2]);
  Motor4.writeMicroseconds(_motor[3]);
#elif defined HARDPWM
  PWM.writeESC(0, _motor[0]);
  PWM.writeESC(1, _motor[1]);
  PWM.writeESC(2, _motor[2]);
  PWM.writeESC(3, _motor[3]);
#else
  MotorESC_pulsePWM();
#endif 
}

