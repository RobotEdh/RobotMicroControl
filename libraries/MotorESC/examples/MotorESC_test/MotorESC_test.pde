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
  for(int i=130; i< 150; i++)
  {
      Serial.print("i: ");Serial.println(i);
      MotorESC.MotorESC_writeAllMotors(i);
      delay(500);
  }
   
  MotorESC.MotorESC_test();
}