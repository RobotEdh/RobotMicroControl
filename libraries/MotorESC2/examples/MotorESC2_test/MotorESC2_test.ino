#include <MotorESC2.h>
 
MotorESC2Class MotorESC;  

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
 int delta = 100;
 const char *szMotors[] = {"Front Right", "Rear Right", "Rear Left", "Front Left"};
 int16_t ESC_command[NBMOTORS];
 uint32_t tick = 0;

  Serial.println("Beging ESC Tests");
    
/* START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way  */
  MotorESC.MotorESC_runMotors(-1, MINPWM);   // stop
  Serial.println("START TESTCASE 1: spin up each blade individually for 10s each and check they all turn the right way");

  for(int i=0; i< NBMOTORS; i++)
  {
      Serial.println(szMotors[i]);
      MotorESC.MotorESC_runMotors(i, throttle);
      MotorESC.MotorESC_pulsePWM(10*1000);  // duration 10s

      MotorESC.MotorESC_runMotors(-1, MINPWM);  // stop
      Serial.println("END TESTCASE 1");
      MotorESC.MotorESC_pulsePWM(1*1000);  // duration 1s
  }     
/* END TESTCASE 1 */

/* START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM */
  Serial.print("START TESTCASE 2: Spin all the motors together near to minimum speed MINPWM: ");Serial.println(MINPWM);
  
  for(int pwm=MINPWM; pwm<MINPWM+200; pwm+=20)
  {
      Serial.print("PWM: ");Serial.println(pwm);
      MotorESC.MotorESC_runMotors(-1, pwm);
      MotorESC.MotorESC_pulsePWM(5*1000);  // duration 5s
  }
  MotorESC.MotorESC_runMotors(-1, MINPWM);  // stop
  Serial.println("END TESTCASE 2");
  MotorESC.MotorESC_pulsePWM(1*1000);  // duration 1s
 /* END TESTCASE 2 */

/* START TESTCASE 3: Front up / increase pitch*/
  Serial.print("START TESTCASE 3: Front up, throttle: ");Serial.print(throttle); Serial.print(" pitch delta: ");Serial.println(delta);
   
  ESC_command[THROTTLE] = throttle;
  ESC_command[ROLL]     = 0;
  ESC_command[PITCH]    = delta;
  ESC_command[YAW]      = 0;
           
  MotorESC.MotorESC_MixPID(ESC_command, tick);
  MotorESC.MotorESC_pulsePWM(10*1000);  // duration 10s
 
  MotorESC.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 3");
  MotorESC.MotorESC_pulsePWM(1*1000);  // duration 1s
 /* END TESTCASE 3 */

/* START TESTCASE 4: Right up / increase roll*/
   Serial.print("START TESTCASE 4: Right up, throttle: ");Serial.print(throttle); Serial.print(" roll delta: ");Serial.println(delta);

  ESC_command[THROTTLE] = throttle;
  ESC_command[ROLL]     = delta;
  ESC_command[PITCH]    = 0;
  ESC_command[YAW]      = 0;
           
  MotorESC.MotorESC_MixPID(ESC_command, tick);
  MotorESC.MotorESC_pulsePWM(10*1000);  // duration 10s
 
  MotorESC.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 4");
  MotorESC.MotorESC_pulsePWM(1*1000);  // duration 1s
 /* END TESTCASE 4 */

/* START TESTCASE 5: Turn right / increase Yaw */
   Serial.print("START TESTCASE 5: Turn right, throttle: ");Serial.print(throttle); Serial.print(" yaw delta: ");Serial.println(delta);

  ESC_command[THROTTLE] = throttle;
  ESC_command[ROLL]     = 0;
  ESC_command[PITCH]    = 0;
  ESC_command[YAW]      = delta;
           
  MotorESC.MotorESC_MixPID(ESC_command, tick);
  MotorESC.MotorESC_pulsePWM(10*1000);  // duration 10s 
 
  MotorESC.MotorESC_runMotors(-1, MINPWM); // stop
  Serial.println("END TESTCASE 5");
  MotorESC.MotorESC_pulsePWM(1*1000);  // duration 1s
/* END TESTCASE 5 */


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
                      MotorESC.MotorESC_init(LED_BUILTIN);                     
                      Serial.println("End startup procedure" );   
            break;
                      
            
            // 0
            case 48 : Serial.print("Send MINPWM = ");Serial.println(MINPWM);
                      MotorESC.MotorESC_runMotors(-1, MINPWM);
                      Serial.println("End Send MINPWM" );   
            break;

            // 1
            case 49 : value = (MINPWM+MAXPWM)/2;
                      Serial.print("Send MIDPWM = ");Serial.println(value);
                      MotorESC.MotorESC_runMotors(-1, value);
                      Serial.println("End Send MIDPWM" );      
            break;
            
            // 2
            case 50 : Serial.print("Send MAXPWM = ");Serial.println(MAXPWM);
                      MotorESC.MotorESC_runMotors(-1, MAXPWM); 
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
                    MotorESC.MotorESC_runMotors(-1, num);
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