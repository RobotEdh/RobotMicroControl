#include <Arduino.h>
#include <MotorESC.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
#define LOGTRACE   // Enable trace
#include <log.h>
File logFile;   

MotorESCClass::MotorESCClass()
{
}
        
void MotorESCClass::MotorESC_init()
{ 
  PRINTs("Start MotorESC_init")

  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
  MotorESC_writeAllMotors(MINPWM);
    
  PRINTs(">Start Init ESC")
  PRINTs(">15 s to connect the ESC to power")
  delay(15*1000); /* 15 s to connect the ESC to power */
  
  PRINTs(">10s Run all motors")
  MotorESC_writeAllMotors((MINPWM+MAXPWM)/2);
  delay(10*1000);
  
  PRINTs(">Stop all motors")  
  MotorESC_writeAllMotors(MINPWM);

  PRINTs("<End MotorESC_init")
}

void MotorESCClass::MotorESC_test()
{
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC_writeAllMotors(MINPWM);   // stop
  PRINTs("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way")

  for(int i=0; i< NBMOTORS; i++)
  {
      PRINTi("szMotors",i,szMotors[i])
      MotorESC_writeOneMotor(i, (MINPWM+MAXPWM)/2);
      delay(5*1000);
  }
  MotorESC_writeAllMotors(MINPWM);  // stop
  PRINTs("END TESTCASE 1")

/* END TESTCASE 1 */


/* START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided  */
  PRINTs("START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided")

  MotorESC_writeAllMotors((MINPWM+MAXPWM)/2);
  delay(5*1000); 
 
  MotorESC_writeAllMotors(MINPWM); // stop
  PRINTs("END TESTCASE 2")
 
/* END TESTCASE 2 */
  
  PRINTs("End OK ESC Init")   
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
    _motor[i]=MINPWM;  // stop all motors...
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
void MotorESCClass::MotorESC_RunMotors(int16_t ESC_command[4])
{
  int16_t maxMotor = 0;
  int16_t minMotor = 0;
  int i;
  
  if (ESC_command[THROTTLE] == 0) 
  { 
     for(i=0; i< NBMOTORS; i++) _motor[i] = MINPWM;
  }
  else
  {   
     int16_t throttle = map(ESC_command[THROTTLE], MINPPM, MAXPPM, MINPWM, MAXPWM);
     throttle = constrain(throttle, MINPWM, MAXPWMTHRO);  // to give room for PID ajustement
     PRINT("throttle: ",throttle)
     
     #define PIDMIX(X,Y,Z) ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z
    _motor[0] = PIDMIX(-1,+1,+1); //Front Left
    _motor[1] = PIDMIX(+1,+1,-1); //Front Right
    _motor[2] = PIDMIX(+1,-1,+1); //Rear Right
    _motor[3] = PIDMIX(-1,-1,-1); //Rear Left

    for(i=0; i< NBMOTORS; i++) {
       PRINTi("szMotors",i,szMotors[i])
       _motor[i] = map(_motor[i], -90, 90, -(MAXPWM-MINPWM)/2, (MAXPWM-MINPWM)/2);
       PRINTi(">_motor mapped ",i,_motor[i])
       _motor[i] = _motor[i] + throttle;
       if((_motor[i]-MAXPWM) > maxMotor) maxMotor = _motor[i]-MAXPWM;
       if((MINPWM - _motor[i]) > minMotor) minMotor = MINPWM - _motor[i];
    }
    
    if (maxMotor > 0) {
       PRINT(">maxMotor: ",maxMotor)
       for(i=0; i< NBMOTORS; i++) {
          _motor[i] = _motor[i] - maxMotor;
          if((MINPWM - _motor[i]) > minMotor) minMotor = MINPWM - _motor[i];
       }
    }
    
    if (minMotor > 0) {
       PRINT(">minMotor: ",minMotor) 
       for(i=0; i< NBMOTORS; i++) {
          _motor[i] = _motor[i] + minMotor;
       }  
    }
    
    if ((minMotor > 0) || (maxMotor > 0)) {
       for(i=0; i< NBMOTORS; i++) {
          PRINTi("szMotors",i,szMotors[i])
          _motor[i] = constrain(_motor[i], MINPWM, MAXPWM);  // last cap if still needed after up and bottom cap
          PRINTi(">_motor last cap ",i,_motor[i])
       }  
    }  
      
  } 

  MotorESC_writeMotors();
}