
#include <motor.h>




  
void setup()
{ 
  Serial.begin(9600); // initialize serial port
  // H bridge setup
  pinMode(InMotorRight1Pin, OUTPUT);      // set the pin as output
  pinMode(EnableMotorRight1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(InMotorRight2Pin, OUTPUT);      // set the pin as output
  pinMode(EnableMotorRight2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(InMotorLeft1Pin, OUTPUT);       // set the pin as output
  pinMode(EnableMotorLeft1Pin, OUTPUT);   // set the analogig pin as output for PWM
  pinMode(InMotorLeft2Pin, OUTPUT);       // set the pin as output
  pinMode(EnableMotorLeft2Pin, OUTPUT);   // set the analogig pin as output for PWM
  
  Serial.println("H bridge setup done");  


}


void loop()
{

  Serial.println(" --> start_forward motor"); 
  start_forward();
  delay(5000); //make it readable 
   
  Serial.println(" --> stop");  
  stop();
  delay(5000); //make it readable 


  Serial.println(" --> start_backward motor"); 
  start_backward();
  delay(5000); //make it readable 

  Serial.println(" --> stop");  
  stop();
  delay(5000); //make it readable 

}

