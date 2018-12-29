
#include <MotorESC.h>
 


MotorESCClass MotorESC;     

void setup()
{
  
  Serial.begin(9600); // initialize serial port
  MotorESC.MotorESC_init();
 
  Serial.println("End Init" ); 

}


void loop()
{
  long starttime;
  
  starttime = millis();
  
  MotorESC.MotorESC_RunMotors(1000,1000,1000,1000);

  Serial.print("elasptime: "); Serial.println(millis() - starttime);

}

