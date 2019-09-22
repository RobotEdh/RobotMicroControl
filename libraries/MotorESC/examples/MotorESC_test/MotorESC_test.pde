#include <MotorESC.h>
 
MotorESCClass MotorESC;  

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Serial.println("Start Init" );
  
  MotorESC.MotorESC_init();
  delay(10*1000);
  
  Serial.println("End Init" ); 
  
  displayInstructions();

}

void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("A : Send max throttle to motor 1 only");
    Serial.println("B : Send max throttle to motor 2 only"); 
    Serial.println("C : Send max throttle to motor 3 only"); 
    Serial.println("D : Send max throttle to motor 4 only");  
    Serial.println("0 : Send min throttle all motors");
    Serial.println("1 : Send max throttle all motors");
    Serial.println("2 : Run test function");
    
}

void test()
{ 
 int throttle = 1300;
 int delta = 50;
 const char *szMotors[] = {"Front Left", "Front Right", "Rear Right", "Rear Left"};

  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC.MotorESC_writeAllMotors(STOPPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC.MotorESC_writeOneMotor(i, throttle);
      if (i>0) MotorESC.MotorESC_writeOneMotor(i-1, STOPPWM);
      delay(10*1000);
  }
  MotorESC.MotorESC_writeAllMotors(STOPPWM);  // stop
  Serial.println("END TESTCASE 1");
  delay(1*1000);
/* END TESTCASE 1 */

/* START TESTCASE 2: Spin all the motors together near to minimum speed STOPPWM */
  Serial.print("START TESTCASE 2: Spin all the motors together near to minimum speed STOPPWM: ");Serial.println(STOPPWM);
  
  for(int pwm=STOPPWM-20; pwm< STOPPWM+20; pwm+=20)
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
   Serial.print("START TESTCASE 3: Front up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,throttle + delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,throttle + delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,throttle - delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,throttle - delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 3");
  delay(1*1000);
 /* END TESTCASE 3 */

/* START TESTCASE 4: Rear up */
   Serial.print("START TESTCASE 4: Rear up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,throttle - delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,throttle - delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,throttle + delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,throttle + delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 4");
  delay(1*1000);
 /* END TESTCASE 4 */

/* START TESTCASE 5: Left up */
   Serial.print("START TESTCASE 5: Left up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,throttle + delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,throttle - delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,throttle - delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,throttle + delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 5");
  delay(1*1000);
/* END TESTCASE 5 */

/* START TESTCASE 6: Right up */
   Serial.print("START TESTCASE 6: Right up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC.MotorESC_writeOneMotor(0,throttle - delta); //Front Left
  MotorESC.MotorESC_writeOneMotor(1,throttle + delta); //Front Right
  MotorESC.MotorESC_writeOneMotor(2,throttle + delta); //Rear Right
  MotorESC.MotorESC_writeOneMotor(3,throttle - delta); //Rear Left
  delay(10*1000); 
 
  MotorESC.MotorESC_writeAllMotors(STOPPWM); // stop
  Serial.println("END TESTCASE 6");
  delay(1*1000);
 /* END TESTCASE 6 */
  
   Serial.println("End OK ESC Tests");   
   delay(10*1000);    
}


void loop() {
    if (Serial.available()) {
        char data = Serial.read();

        switch (data) {     
            
            // A
            case 65 : Serial.print("Sending maximum throttle motor1");
                      MotorESC.MotorESC_writeAllMotors(STOPPWM);
                      MotorESC.MotorESC_writeOneMotor(1,MAXPWM); 
            break; 
                     
            // B
            case 66 : Serial.print("Sending maximum throttle motor2");
                      MotorESC.MotorESC_writeAllMotors(STOPPWM);
                      MotorESC.MotorESC_writeOneMotor(2,MAXPWM); 
            break; 
            
            // C
            case 67 : Serial.print("Sending maximum throttle motor3");
                      MotorESC.MotorESC_writeAllMotors(STOPPWM);
                      MotorESC.MotorESC_writeOneMotor(3,MAXPWM); 
            break;
            
            // D
            case 68 : Serial.print("Sending maximum throttle motor4");
                      MotorESC.MotorESC_writeAllMotors(STOPPWM);
                      MotorESC.MotorESC_writeOneMotor(4,MAXPWM); 
            break;
                        
            // 0
            case 48 : Serial.println("Sending minimum throttle");
                      MotorESC.MotorESC_writeAllMotors(STOPPWM);
            break;

            // 1
            case 49 : Serial.println("Sending maximum throttle");
                      MotorESC.MotorESC_writeAllMotors(MAXPWM); 
            break;

            // 2
            case 50 : Serial.print("Running test");
                      test();
            break;
        }
    }
    

}