#include <Arduino.h> 
#include <Wire.h>       // I2C protocol
#include <RC.h>         // Radio Command
#include <MotorESC.h>   // Motor ESC
    
RCClass       RC;                 // The Radio Command class
MotorESCClass MotorESC;           // The Motor ESC Class

void setup()
{
  Wire.begin(); // initialize I2C
   
  Serial.begin(9600); // initialize serial port
  Serial.println("Start init"); 
  
  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
  MotorESC.MotorESC_writeAllMotors(MAXPWM);  
  
  Serial.println("you have 15 s to connect the ESC to power..."); 
  digitalWrite(LED_PIN, HIGH);  // turn on Led for 15s
  delay(15*1000); /* 15 s to connect the ESC to power */
  digitalWrite(LED_PIN, LOW);  // turn on Led
  Serial.println("... done"); 
   
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
	if (RC_command[THROTTLE] == 0) value = MINPWM-2;   // min value PWM
	else                           value = MAXPWM; // max value PWM
    
    Serial.print("Throttle received: ");Serial.println(RC_command[THROTTLE]); 
	Serial.print("Value set to the motors: ");Serial.println(value); 
    MotorESC.MotorESC_writeAllMotors(value);
}