#include <MotorESC2.h>
 
MotorESC2Class MotorESC2;  

void setup()
{
  
  Serial.begin(115200); // initialize serial port

  displayInstructions();
  
}

void displayInstructions()
{  
    Serial.println("READY - PLEASE SEND INSTRUCTIONS AS FOLLOWING :");
    Serial.println("A : Run startup procedure");
    Serial.println("0 : Send MINPWM");
    Serial.println("1 : Send MIDPWM");
    Serial.println("2 : Send MAXPWM");
    Serial.println("3 : Send value entered");
    Serial.println("9 : Run tests");
}

 
void test()
{ 
 int throttle = 1300;
 int delta = 50;
 const char *szMotors[] = {"Front Right", "Rear Right", "Rear Left", "Front Left"};

  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC2.MotorESC_runMotors(-1, MINPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC2.MotorESC_runMotors(i, throttle);
      if (i>0) MotorESC2.MotorESC_runMotors(i, MINPWM);
      delay(10*1000);
  }
  MotorESC2.MotorESC_runMotors(-1, MINPWM);  // stop
  Serial.println("END TESTCASE 1");
  delay(1*1000);
/* END TESTCASE 1 */

/* START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM */
  Serial.print("START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM: ");Serial.println(MINPWM);
  
  for(int pwm=MINPWM; pwm<MINPWM+200; pwm+=20)
  {
      Serial.print("PWM: ");Serial.println(pwm);
      MotorESC2.MotorESC_runMotors(-1, pwm);
      delay(5*1000);
  }
  MotorESC2.MotorESC_runMotors(-1, MINPWM);  // stop
  Serial.println("END TESTCASE 2");
  delay(1*1000);
 /* END TESTCASE 2 */

/* START TESTCASE 3: Front up */
   Serial.print("START TESTCASE 3: Front up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC2.MotorESC_runMotors(0,throttle + delta); //Front Right
  MotorESC2.MotorESC_runMotors(1,throttle - delta); //Rear Right
  MotorESC2.MotorESC_runMotors(2,throttle - delta); //Rear Left
  MotorESC2.MotorESC_runMotors(3,throttle + delta); //Front Left
  delay(10*1000); 
 
  MotorESC2.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 3");
  delay(1*1000);
 /* END TESTCASE 3 */

/* START TESTCASE 4: Rear up */
   Serial.print("START TESTCASE 4: Rear up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC2.MotorESC_runMotors(0,throttle - delta); //Front Right
  MotorESC2.MotorESC_runMotors(1,throttle + delta); //Rear Right
  MotorESC2.MotorESC_runMotors(2,throttle + delta); //Rear Left
  MotorESC2.MotorESC_runMotors(3,throttle - delta); //Front Left
  delay(10*1000); 
 
  MotorESC2.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 4");
  delay(1*1000);
 /* END TESTCASE 4 */

/* START TESTCASE 5: Left up */
   Serial.print("START TESTCASE 5: Left up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC2.MotorESC_runMotors(0,throttle - delta); //Front Right
  MotorESC2.MotorESC_runMotors(1,throttle - delta); //Rear Right
  MotorESC2.MotorESC_runMotors(2,throttle + delta); //Rear Left
  MotorESC2.MotorESC_runMotors(3,throttle + delta); //Front Left
  delay(10*1000); 
 
  MotorESC2.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 5");
  delay(1*1000);
/* END TESTCASE 5 */

/* START TESTCASE 6: Right up */
   Serial.print("START TESTCASE 6: Right up, throttle: ");Serial.print(throttle); Serial.print(" delta: ");Serial.println(delta);

  MotorESC2.MotorESC_runMotors(0,throttle + delta); //Front Right
  MotorESC2.MotorESC_runMotors(1,throttle + delta); //Rear Right
  MotorESC2.MotorESC_runMotors(2,throttle - delta); //Rear Left
  MotorESC2.MotorESC_runMotors(3,throttle - delta); //Front Left
  delay(10*1000); 
 
  MotorESC2.MotorESC_runMotors(-1, MINPWM); // stop
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
    int16_t value = 0;
        
    if (Serial.available()) {
    
       char data = Serial.read();
 
        switch (data) {     
             // A
            case 65 : Serial.println("Running startup procedure");          
                      MotorESC2.MotorESC_init();                     
                      Serial.println("End startup procedure" );   
            break;
                      
            
            // 0
            case 48 : Serial.print("Send MINPWM = ");Serial.println(MINPWM);
                      MotorESC2.MotorESC_runMotors(-1, MINPWM);
                      Serial.println("End Send MINPWM" );   
            break;

            // 1
            case 49 : value = (MINPWM+MAXPWM)/2;
                      Serial.print("Send MIDPWM = ");Serial.println(value);
                      MotorESC2.MotorESC_runMotors(-1, value);
                      Serial.println("End Send MIDPWM" );      
            break;
            
            // 2
            case 50 : Serial.print("Send MAXPWM = ");Serial.println(MAXPWM);
                      MotorESC2.MotorESC_runMotors(-1, MAXPWM); 
                      Serial.println("End Send MAXPWM" );   
            break;
            
            // 3
            case 51 : Serial.print("Enter the value to send between ");Serial.print(MINPWM);Serial.print(" and ");Serial.println(MAXPWM); 
                   indx = 0;       
                   while ((Serial.available()) || (indx < 4 )){
                         if (Serial.available()) {
                             s = Serial.read();	//read Serial 
                             strArray[indx] = s;  
                             indx++; 
                             }    
                    } // end while
                    strArray[indx] = '\0';
                    num = atoi(strArray);
                    if (num < MINPWM) num = MINPWM;
                    if (num > MAXPWM) num = MAXPWM;
                    
                    Serial.print("Send "); Serial.println(num);  
                    MotorESC2.MotorESC_runMotors(-1, num);
                    Serial.println("End Send value");      
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