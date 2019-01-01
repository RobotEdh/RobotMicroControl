#include <MotorESC.h>
 
MotorESCClass MotorESC;     

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  
  
 
  MotorESC.MotorESC_init();Serial.println("End Init" ); 

}


void loop()
{
  for(int i=0; i< 255; i++)
  {
      Serial.print("i: ");Serial.println(i);
      MotorESC.MotorESC_writeAllMotors(i);
      delay(1000);
  }
   
  MotorESC.MotorESC_test();
}