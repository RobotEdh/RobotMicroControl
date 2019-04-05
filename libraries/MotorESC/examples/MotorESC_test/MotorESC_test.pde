#include <MotorESC.h>
 
MotorESCClass MotorESC;     

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Serial.println("Start Init" );
  
  MotorESC.MotorESC_init();
  
  Serial.println("End Init" ); 

}


void loop()
{ 
   
  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC.MotorESC_writeAllMotors(STOPPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC.MotorESC_writeOneMotor(i, (MINPWM+MAXPWM)/2);
      delay(10*1000);
  }
  MotorESC.MotorESC_writeAllMotors(STOPPWM);  // stop
  Serial.println("END TESTCASE 1");
  delay(1*1000);

/* END TESTCASE 1 */

/* START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM */
  Serial.print("START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM: ");Serial.println(MINPWM);
  
  for(int pwm=MINPWM-2; pwm< MINPWM+2; pwm++)
  {
      Serial.print("PWM: ");Serial.println(pwm);
      MotorESC.MotorESC_writeAllMotors(pwm);
      delay(5000);
  }
  MotorESC.MotorESC_writeAllMotors(STOPPWM);  // stop
  Serial.println("END TESTCASE 2");
  delay(1*1000);
 
/* END TESTCASE 2 */

/* START TESTCASE 3: Spin all the motors together for 10s at mid PWM = (MINPWM+MAXPWM)/2 judging how much lift is provided */
   Serial.print("START TESTCASE 3: Spin all the motors together for 10s at mid PWM = (MINPWM+MAXPWM)/2 judging how much lift is provided");;Serial.println((MINPWM+MAXPWM)/2);

  MotorESC.MotorESC_writeAllMotors((MINPWM+MAXPWM)/2);
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 3");
  delay(1*1000);
 
/* END TESTCASE 3 */
  
   Serial.println("End OK ESC Tests");   
   delay(10*1000);    
}