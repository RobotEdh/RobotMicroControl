#include <Arduino.h>
#include <MotorESC.h>


MotorESCClass::MotorESCClass()
{
}
        
void MotorESCClass::MotorESC_init()
{ 
  Serial.println("Start MotorESC_init");

  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
  MotorESC_writeAllMotors(MINPWM);
    
  Serial.println(">Start Init ESC");
  Serial.println(">15 s to connect the ESC to power");
  delay(15*1000); /* 15 s to connect the ESC to power */
  
  Serial.println(">10s Run all motors");  
  MotorESC_writeAllMotors((MINPWM+MAXPWM)/2);
  delay(10*1000);
  
  Serial.println(">Stop all motors");  
  MotorESC_writeAllMotors(MINPWM);

  Serial.println("<End MotorESC_init");
}

void MotorESCClass::MotorESC_test()
{
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC_writeAllMotors(MINPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC_writeOneMotor(i, (MINPWM+MAXPWM)/2);
      delay(5*1000);
  }
  MotorESC_writeAllMotors(MINPWM);  // stop
  Serial.println("END TESTCASE 1");

/* END TESTCASE 1 */


/* START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided  */
  Serial.println("START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided");

  MotorESC_writeAllMotors((MINPWM+MAXPWM)/2);
  delay(5*1000); 
 
 MotorESC_writeAllMotors(MINPWM); // stop
 Serial.println("END TESTCASE 2");
 
/* END TESTCASE 2 */

  MotorESC_writeAllMotors(MINPWM);
  
  Serial.println("End OK ESC Init");   
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
  int i;
  
  if (ESC_command[THROTTLE] == 0) 
  { 
     for(i=0; i< NBMOTORS; i++) _motor[i] = MINPWM;
  }
  else
  {   
     #define PIDMIX(X,Y,Z) ESC_command[THROTTLE] + ESC_command[ROLL]*X + ESC_command[PITCH]*Y + ESC_command[YAW]*Z 
    _motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    _motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    _motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    _motor[3] = PIDMIX(+1,-1,-1); //FRONT_L

    for(i=0; i< NBMOTORS; i++) {
       _motor[i] = map(_motor[i], MINPPM, MAXPPM, MINPWM, MAXPWM);
       _motor[i] = constrain(_motor[i], MINPWM, MAXPWM);
    }
  } 
 
  MotorESC_writeMotors();
}
