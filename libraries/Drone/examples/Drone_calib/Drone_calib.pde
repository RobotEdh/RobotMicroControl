#include <Arduino.h> 
#include <Wire.h>       // I2C protocol
#include <RC.h>         // Radio Command
#include <MotorESC.h>   // Motor ESC
    
RCClass       RC;                 // The Radio Command class
MotorESCClass MotorESC;           // The Motor ESC Class

void setup()
{
  uint8_t servoIndex = 0; 
  
  Wire.begin(); // initialize I2C
   
  Serial.begin(9600); // initialize serial port
  Serial.println("Start init"); 

  servoIndex = MotorESC.MotorESC_init(MAXPWM, -1, 0);  // Init all motors with MAXPWM and no delay
  Serial.print("servoIndex: "); Serial.println(servoIndex);  // display the number of "servos"
  MotorESC.MotorESC_itimer();  // Display the timers used
  
  RC.RC_init(); 
  
  Serial.println("End init");
  
}


void loop()
{
    int16_t RC_command[NBCHANNELS];
	int16_t value = -1;

    // Get RC commands
    RC.RC_getCommands(RC_command); 
             
    // call MotorESC
	if (RC_command[THROTTLE] == 0)              value = MINPWM;               // min value PWM
	else if (RC_command[THROTTLE] < 0.8*MAXPPM) value = (MAXPWM+MINPWM)/2;    // mid value PWM
	else                                        value = MAXPWM;               // max value PWM
    
    Serial.print("Throttle received: ");Serial.println(RC_command[THROTTLE]); 
	Serial.print("Value set to the motors: ");Serial.println(value); 
    MotorESC.MotorESC_writeAllMotors(value);
}