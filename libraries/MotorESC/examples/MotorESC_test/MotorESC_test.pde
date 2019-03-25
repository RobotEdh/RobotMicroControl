#include <MotorESC.h>
 
MotorESCClass MotorESC;     

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  Serial.println("Start Init" );
  
  MotorESC.MotorESC_init();
  
  Serial.println("End Init" ); 

}


void loop()
{ 
  MotorESC.MotorESC_test();
}