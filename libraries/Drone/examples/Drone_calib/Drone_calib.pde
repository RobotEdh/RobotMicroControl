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
  Serial.print("Start init"); 
  
  pinMode(Motor1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor3Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(Motor4Pin, OUTPUT);  // set the analogig pin as output for PWM
      
  RC.RC_init(); 
  
  Serial.print("End init");
  
}


void loop()
{
    int16_t RC_command[NBCHANNELS];
    int16_t ESC_command[NBMOTORS]; 

    // Get RC commands
    RC.RC_getCommands(RC_command); // int16_t range [-90;+90]for ROLL, PITCH and range [-90;+90] for YAW
             
    // call MotorESC
    ESC_command[THROTTLE] = RC_command[THROTTLE];
    ESC_command[ROLL] = 0;
    ESC_command[PITCH] = 0; 
    ESC_command[YAW] = 0;
    
    Serial.print("ESC_command[THROTTLE] : ");Serial.println(ESC_command[THROTTLE]); 
    MotorESC.MotorESC_RunMotors(ESC_command,0);
}
