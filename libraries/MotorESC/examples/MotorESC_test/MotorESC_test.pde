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
  int16_t ESC_command[4];
  
  starttime = millis();
  
  ESC_command[THROTTLE] = 1000;
  ESC_command[ROLL] = 1000;
  ESC_command[PITCH] = 1000;
  ESC_command[YAW] = 1000;
        
  MotorESC.MotorESC_RunMotors(ESC_command);

  Serial.print("elasptime: "); Serial.println(millis() - starttime);

}

