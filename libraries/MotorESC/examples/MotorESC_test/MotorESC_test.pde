#include <MotorESC.h>
 
MotorESCClass MotorESC;  

void setup()
{
  
  Serial.begin(9600); // initialize serial port

  displayInstructions();

}

void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("A : Init all motors (start Throttle stick = MINPWM)");
    Serial.println("B : Program/Calib all motors (start Throttle stick = MAXPWM)");
    Serial.println("C : Program/Calib motor 1 (start Throttle stick = MAXPWM)"); 
    Serial.println("D : Program/Calib motor 2 (start Throttle stick = MAXPWM)"); 
    Serial.println("E : Program/Calib motor 3 (start Throttle stick = MAXPWM)"); 
    Serial.println("F : Program/Calib motor 4 (start Throttle stick = MAXPWM)");   
    Serial.println("0 : Send MINPWM all motors");
    Serial.println("1 : Send MAXPWM all motors");
    Serial.println("2 : Send value entered all motors");
    Serial.println("9 : Run tests all motors");
}

void test()
{ 
 int throttle = 1300;
 int delta = 50;
 const char *szMotors[] = {"Front Left", "Front Right", "Rear Right", "Rear Left"};

  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC.MotorESC_writeAllMotors(MINPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC.MotorESC_writeOneMotor(i, throttle);
      if (i>0) MotorESC.MotorESC_writeOneMotor(i-1, MINPWM);
      delay(10*1000);
  }
  MotorESC.MotorESC_writeAllMotors(MINPWM);  // stop
  Serial.println("END TESTCASE 1");
  delay(1*1000);
/* END TESTCASE 1 */

/* START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM */
  Serial.print("START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM: ");Serial.println(MINPWM);
  
  for(int pwm=MINPWM; pwm<MINPWM+200; pwm+=20)
  {
      Serial.print("PWM: ");Serial.println(pwm);
      MotorESC.MotorESC_writeAllMotors(pwm);
      delay(5*1000);
  }
  MotorESC.MotorESC_writeAllMotors(MINPWM);  // stop
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
 
  MotorESC.MotorESC_writeAllMotors(MINPWM); // stop
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
 
  MotorESC.MotorESC_writeAllMotors(MINPWM); // stop
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
 
  MotorESC.MotorESC_writeAllMotors(MINPWM); // stop
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
 
  MotorESC.MotorESC_writeAllMotors(MINPWM); // stop
  Serial.println("END TESTCASE 6");
  delay(1*1000);
 /* END TESTCASE 6 */
  
   Serial.println("End OK ESC Tests");     
}


void loop() {
    char s = '0';
    int indx = 0;
    char strArray[100];
    int num = 0;
    
    if (Serial.available()) {
    
       char data = Serial.read();
 
        switch (data) {     
            
            // A
            case 65 : Serial.println("Init (start Throttle stick = MINPWM)");
                      Serial.println("15s to connect ESC to power");
                      
                      MotorESC.MotorESC_init();
                      
                      Serial.println("End Init" ); 
            break; 
                     
            // B
            case 66 : Serial.println("Program/Calib all motors (start Throttle stick = MAXPWM)");
                                        
                      MotorESC.MotorESC_init(MAXPWM, -1);
                      
                      Serial.println("End Program/Calib");  
            break; 
            // C
            case 67 : Serial.println("Program/Calib motor 1 (start Throttle stick = MAXPWM)");
                                        
                      MotorESC.MotorESC_init(MAXPWM, 0);
                      
                      Serial.println("End Program/Calib ");  
            break;            
            // D
            case 68 : Serial.println("Program/Calib motor 2 (start Throttle stick = MAXPWM)");
                                        
                      MotorESC.MotorESC_init(MAXPWM, 1);
                      
                      Serial.println("End Program/Calib");  
            break;
            // E
            case 69 : Serial.println("Program/Calib motor 3 (start Throttle stick = MAXPWM)");
                                        
                      MotorESC.MotorESC_init(MAXPWM, 2);
                      
                      Serial.println("End Program/Calib");  
            break;            
            // F
            case 70 : Serial.println("Program/Calib motor 4 (start Throttle stick = MAXPWM)");
                                        
                      MotorESC.MotorESC_init(MAXPWM, 3);
                      
                      Serial.println("End Program/Calib");  
            break; 
                                       
            // 0
            case 48 : Serial.println("Send MINPWM");
                      MotorESC.MotorESC_writeAllMotors(MINPWM);
            break;

            // 1
            case 49 : Serial.println("Send MAXPWM");
                      MotorESC.MotorESC_writeAllMotors(MAXPWM); 
            break;
            
            // 2
            case 50 : Serial.println("Enter the value to send between 1000ms and 2000ms: ");  
                   indx = 0;       
                   while ((Serial.available()) || (indx < 4+1 )){
                         if (Serial.available()) {
                             s = Serial.read();	//read Serial 
                             strArray[indx] = s;  
                             indx++; 
                             }    
                    } // end while
                    strArray[indx] = '\0';
                    num = atoi(strArray);
                    if (num < 1000) num = 1000;
                    if (num > 2000) num = 2000;
                    Serial.print("Send "); Serial.println(num);  
                    MotorESC.MotorESC_writeAllMotors(num);
            break; 
           
            // 9
            case 57 : Serial.println("Running tests");
                      test();
                      Serial.println("End tests");
            break;
           
            // other
            default: 
                 if (data > 31) Serial.println("Bad value");

            break;          
        } // end switch
    } // end if
    

}