#include <MotorESC.h>
 
MotorESCClass MotorESC;  

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Serial.println("Start Init" );
  
  MotorESC.MotorESC_init();
  delay(10*1000);
  
  Serial.println("End Init" ); 

}


void loop()
{ 
 int delta = 20;
 const char *szMotors[] = {"Front Left", "Front Right", "Rear Right", "Rear Left"};

  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC.MotorESC_writeAllMotors(STOPPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC.MotorESC_writeOneMotor(i, (MINPWM+MAXPWM)/2);
      if (i>0) MotorESC.MotorESC_writeOneMotor(i-1, STOPPWM);
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

/* START TESTCASE 3: Front up */
   Serial.print("START TESTCASE 3: Front up, throttle: ");Serial.print((MINPWM+MAXPWM)/2); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,((MINPWM+MAXPWM)/2) + delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,((MINPWM+MAXPWM)/2) + delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,((MINPWM+MAXPWM)/2) - delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,((MINPWM+MAXPWM)/2) - delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 3");
  delay(1*1000);
 /* END TESTCASE 3 */

/* START TESTCASE 4: Rear up */
   Serial.print("START TESTCASE 4: Rear up, throttle: ");Serial.print((MINPWM+MAXPWM)/2); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,((MINPWM+MAXPWM)/2) - delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,((MINPWM+MAXPWM)/2) - delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,((MINPWM+MAXPWM)/2) + delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,((MINPWM+MAXPWM)/2) + delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 4");
  delay(1*1000);
 /* END TESTCASE 4 */

/* START TESTCASE 5: Left up */
   Serial.print("START TESTCASE 5: Left up, throttle: ");Serial.print((MINPWM+MAXPWM)/2); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,((MINPWM+MAXPWM)/2) + delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,((MINPWM+MAXPWM)/2) - delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,((MINPWM+MAXPWM)/2) - delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,((MINPWM+MAXPWM)/2) + delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 5");
  delay(1*1000);
/* END TESTCASE 5 */

/* START TESTCASE 6: Right up */
   Serial.print("START TESTCASE 6: Right up, throttle: ");Serial.print((MINPWM+MAXPWM)/2); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,((MINPWM+MAXPWM)/2) - delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,((MINPWM+MAXPWM)/2) + delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,((MINPWM+MAXPWM)/2) + delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,((MINPWM+MAXPWM)/2) - delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 6");
  delay(1*1000);
 /* END TESTCASE 6 */
  
   Serial.println("End OK ESC Tests");   
   delay(10*1000);    
}
