#include <SharpIR.h>


SharpIRClass SharpIR;

void setup() {

  Serial.begin(9600);
  
  SharpIR.SharpIR_init((long)1080); // initialize the default pin A1 connected to the Sharp IR GP2Y0A21Y

//    > 1080 is the int for the GP2Y0A21Y and 
//    > 20150 is the int for GP2Y0A02YK and 
//    > 100500 is the long for GP2Y0A710K0F
}

void loop() {
  delay(2000);   

  unsigned long time1=millis();  // takes the time before the loop on the library begins

  int dis=SharpIR.SharpIR_distance();  // this returns the distance to the object you're measuring
  
  unsigned long time2=millis()-time1;  // the following gives you the time taken to get the measurement

  Serial.print("distance: ");  // returns it to the serial monitor
  Serial.println(dis);
  
  Serial.print("Time taken (ms): "); Serial.println(time2);  

}