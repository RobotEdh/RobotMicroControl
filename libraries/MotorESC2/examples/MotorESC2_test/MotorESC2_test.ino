#include <MotorESC2.h>
 
MotorESC2Class MotorESC;  

void setup()
{
  
  Serial.begin(115200); // initialize serial port

  displayInstructions();  
}

void displayInstructions()
{  
    Serial.println("");
    Serial.println("PLEASE SEND INSTRUCTIONS AS FOLLOWING:");
    Serial.println("A : Run Test ALL motors at same time");
    Serial.println("B : Run Test ALL motors in sequence");
    Serial.println("0 : Run Test motor 0 Front Right");
    Serial.println("1 : Run Test motor 1 Rear Right");
    Serial.println("2 : Run Test motor 2 Rear Left");
    Serial.println("3 : Run Test motor 3 Front Left");
    Serial.println("9 : Calibrate");
}

void calibrate()
{ 
   Serial.println("");
   Serial.println("1. Plug your Arduino to your computer with USB cable, open terminal, then type 1 to send max throttle to every ESC to enter programming mode");
   Serial.println("2. Power up your ESCs. You must hear beep1 beep2 beep3 tones meaning the power supply is OK");
   Serial.println("3. After 2sec, beep beep tone emits, meaning the throttle highest point has been correctly confirmed");
   Serial.println("4. Type 0 to send min throttle");
   Serial.println("5. Several beep tones emits, which means the quantity of the lithium battery cells (3 beeps for a 3 cells LiPo)");
   Serial.println("6. A long beep tone emits meaning the throttle lowest point has been correctly confirmed");
   Serial.println("7. Type 2 to launch test function. This will send min to max throttle to ESCs to test them");
   Serial.println("");
   Serial.println("0 : Send minimum throttle");
   Serial.println("1 : Send maximum throttle");
   Serial.println("9 : Exit");
    
   int exit = 0;
   MotorESC.MotorESC_updateMotors(-1, MINPWM); 
   
   while (!exit) {
      if (Serial.available()) {
    
        char data = Serial.read();
 
        switch (data) {     
             // 0
             case 48: Serial.println("Sending minimum throttle");  
                     MotorESC.MotorESC_updateMotors(-1, MINPWM);       
             break;
            
             // 1
             case 49: Serial.println("Sending maximum throttle");
                     MotorESC.MotorESC_updateMotors(-1, MAXPWM);          
             break;             
            
            // 9
            case 57: Serial.println("End calibrate");
                     exit = 1;
            break;
                                            
            // other
            default: 
                 if (data > 31) Serial.println("Bad value");
            break;          
        } // end switch
        
        MotorESC.MotorESC_pulsePWM(1*1000); //1s
         
    } // end if
    MotorESC.MotorESC_pulsePWM(1*1000); //1s
      
  }  // while
  displayInstructions();
}

void runTestVar(int n)
{    
       Serial.print("Send to motor ");Serial.print(n);Serial.print(" from MINPWM = ");Serial.print(MINPWM);Serial.print(" to MAXPWM = ");Serial.print(MAXPWM);
       for (int value = MINPWM; value <= MAXPWM; value += 10) {
           Serial.print("value sent: ");Serial.println(value);
           MotorESC.MotorESC_updateMotors(n, value);       
           MotorESC.MotorESC_pulsePWM(200); //200ms
       }   
       Serial.println("End Send MINPWM to MAXPWM");  
}

void runTest(int n)
{ 
    Serial.println("15s to connect power");          
    MotorESC.MotorESC_init();
    MotorESC.MotorESC_power(LED_BUILTIN, 15);  // 15s to connect          
    Serial.println("End startup procedure" );   
                     
    int value = (MINPWM+MAXPWM)/2;
    Serial.print("Send during 10s to motor ");Serial.print(n);Serial.print(" MIDPWM = ");Serial.println(value);
    MotorESC.MotorESC_updateMotors(n, value);
    MotorESC.MotorESC_pulsePWM(10*1000); //10s
    Serial.println("End Send MIDPWM");      
                     
    Serial.print("Send during 10s to motor ");Serial.print(n);Serial.print(" MAXPWM = ");Serial.println(MAXPWM);
    MotorESC.MotorESC_updateMotors(n, MAXPWM); 
    MotorESC.MotorESC_pulsePWM(10*1000); //10s
    Serial.println("End Send MAXPWM");
    
    runTestVar(n);
          
    MotorESC.MotorESC_updateMotors(n, MINPWM); 
    MotorESC.MotorESC_pulsePWM(1*1000); //1s                      
    
    Serial.print("End Test motor ");Serial.println(n);
    Serial.println("****************"); 
    displayInstructions();
}

void runTestSequence()
{ 
    Serial.println("15s to connect power");          
    MotorESC.MotorESC_init();
    MotorESC.MotorESC_power(LED_BUILTIN, 15);  // 15s to connect          
    Serial.println("End startup procedure" );   
    
    for (int n=0; n<NBMOTORS; n++) {                 
       int value = (MINPWM+MAXPWM)/2;
       Serial.print("Send during 10s to motor ");Serial.print(n);Serial.print(" MIDPWM = ");Serial.println(value);
       MotorESC.MotorESC_updateMotors(n, value);
       MotorESC.MotorESC_pulsePWM(10*1000); //10s
       Serial.println("End Send MIDPWM");      
                     
       Serial.print("Send during 10s to motor ");Serial.print(n);Serial.print(" MAXPWM = ");Serial.println(MAXPWM);
       MotorESC.MotorESC_updateMotors(n, MAXPWM); 
       MotorESC.MotorESC_pulsePWM(10*1000); //10s
       Serial.println("End Send MAXPWM");
       
       runTestVar(n);
      
       MotorESC.MotorESC_updateMotors(n, MINPWM); 
       MotorESC.MotorESC_pulsePWM(1*1000); //1s                      
    }
     
    Serial.print("End Test Sequence motors");
    Serial.println("************************");
    displayInstructions(); 
}



void loop() {
        
    if (Serial.available()) {
    
        char data = Serial.read();
 
        switch (data) {     
             // A
             case 65: runTest(-1);          
             break;
            
             // B
             case 66: runTestSequence();          
             break;             

            // 0
            case 48: runTest(0);          
            break;
            
            // 1
            case 49: runTest(1);
            break;
             
            // 2
            case 50: runTest(2);
            break;
            
            // 3
            case 51: runTest(3);
            break;
            
            // 9
            case 57: calibrate();
            break;
                                            
            // other
            default: 
                 if (data > 31) Serial.println("Bad value");
            break;          
        } // end switch
    } // end if
    
}