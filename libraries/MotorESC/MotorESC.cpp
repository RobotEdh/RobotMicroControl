#include <Arduino.h>
#include <MotorESC.h>


MotorESCClass::MotorESCClass()
{
}
        
void MotorESCClass::MotorESC_init()
{
  const char* sz_blade[] = {"REAR_RIGHT","FRONT_RIGHT","REAR_LEFT","FRONT_LEFT"};
  
  Serial.println("Start ESC Init");

  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
  
  Serial.println(">Start Init ESC");
  Serial.println(">10 s to connect the ESC to power");
  MotorESC_writeAllMotors(MINCOMMAND);
  delay(10*1000); /* 10 s to connect the ESC to power */
  MotorESC_writeAllMotors((MINCOMMAND+ MAXCOMMAND)/2);
  delay(5*1000);
  MotorESC_writeAllMotors(MINCOMMAND);

  Serial.println("<End Init ESC");

/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< 4; i++)
  {
      Serial.println(sz_blade[i]);
      MotorESC_writeOneMotor(i, (MINCOMMAND+ MAXCOMMAND)/2);
      delay(5*1000);
  }
  MotorESC_writeAllMotors(MINCOMMAND);
  
  Serial.println("END TESTCASE 1");

/* END TESTCASE 1 */


/* START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided  */
  Serial.println("START TESTCASE 2: Spin all the motors together for 5s judging how much lift is provided");

  MotorESC_writeAllMotors((MINCOMMAND+ MAXCOMMAND)/2);
  delay(5*1000); 
  MotorESC_writeAllMotors(MINCOMMAND);

  Serial.println("END TESTCASE 2");
 
/* END TESTCASE 2 */

  MotorESC_writeAllMotors(MINCOMMAND);
  
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
  for (uint8_t i =0;i<4;i++) {
    _motor[i]=0;  // reset speed for all motors...
  }
  _motor[no]=value; // ...except for the motor selected
   
  MotorESC_writeMotors();
}


/**************************************************************************************/
/************          Writes the command to all Motors              ******************/
/**************************************************************************************/
void MotorESCClass::MotorESC_writeAllMotors(int16_t value)    // Sends same commands to all motors
{ 
  for (uint8_t i =0;i<4;i++) {
    _motor[i]=value;
  }
  MotorESC_writeMotors();
}


void MotorESCClass::MotorESC_RunMotors(int16_t roll, int16_t pitch,int16_t yaw, int16_t throttle)
{
  int16_t maxMotor;
  int i;
  
  if (throttle < MINCHECK) 
  { 
     for(i=0; i< 4; i++) _motor[i] = MINCOMMAND;
  }
  else
  {   
#define PIDMIX(X,Y,Z) roll*X + pitch*Y + yaw*Z + throttle
    _motor[0] = PIDMIX(-1,+1,-1); //REAR_R
    _motor[1] = PIDMIX(-1,-1,+1); //FRONT_R
    _motor[2] = PIDMIX(+1,+1,+1); //REAR_L
    _motor[3] = PIDMIX(+1,-1,-1); //FRONT_L

    maxMotor=0;
    for(i=0; i< 4; i++) if (_motor[i]>maxMotor) maxMotor=_motor[i];
    
    for(i=0; i< 4; i++) {
      if (maxMotor > MAXCOMMAND) // this is a way to still have good gyro corrections if at least one motor reaches its max.
        _motor[i] -= maxMotor - MAXCOMMAND;
    
      _motor[i] = constrain(_motor[i], MINCOMMAND, MAXCOMMAND);
    }
 } 
 
  MotorESC_writeMotors();
}
