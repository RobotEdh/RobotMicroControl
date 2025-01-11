#include <PWM.h>

PWMClass PWM;  

void setup()
{  
  Serial.begin(115200); // initialize serial port

  PWM.PWMInit(); 
  
  Serial.println("End PWMInit");
}

void loop() {
 
#ifdef MEASURE_ON  
  Serial.print("Nb_tick_Low: ");Serial.println(PWM.get_Nb_tick_Low());
  Serial.print("TrimLow: ");Serial.println(usToTicks((double)PWM.get_Nb_tick_Low()/16.0));
  Serial.print("Nb_tick_High: ");Serial.println(PWM.get_Nb_tick_High());
  Serial.print("TrimHigh: ");Serial.println(usToTicks((double)PWM.get_Nb_tick_High()/16.0));
  delay(10000);
#endif 

  for (int position = 0; position <=180; position ++){ 
        PWM.writeServo(0,position);  
        delay(50); 
  }

  for (int position = 180; position >=0; position --){ 
        PWM.writeServo(0,position);  
        delay(50);  
   }  
   
   PWM.writeServo(0,180);
   delay(5000);
   PWM.writeServo(0,0);
   delay(5000);


}