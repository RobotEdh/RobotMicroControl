/* This example shows how to get single-shot range
 measurements from the VL53L0X. The sensor can optionally be
 configured with different ranging profiles, as described in
 the VL53L0X API user manual, to get better performance for
 a certain application. This code is based on the four
 "SingleRanging" examples in the VL53L0X API.

 The range readings are in units of mm. */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0XClass  VL53L0X;
VL53L0XClass  VL53L0X2;
VL53L0XClass  VL53L0X3;

// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE


// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY
#define MEDIUM_ACCURACY


void setup()
{
  uint16_t reg16;
  uint8_t reg8;
  uint8_t address;
  uint16_t d;
  uint8_t status;
  bool a;
  
  Serial.begin(9600);
  Wire.begin();
  
  Serial.println("Init all ToF VL53L0X");
  
  //1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
  pinMode(35, OUTPUT); 
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  digitalWrite (35, LOW);
  digitalWrite (37, LOW);
  digitalWrite (39, LOW);
  
  delay(10); 
  
  pinMode(35, INPUT);  // XSHUT high avoiding high voltage 
  pinMode(37, INPUT);  // XSHUT high avoiding high voltage 
  pinMode(39, INPUT);  // XSHUT high avoiding high voltage 
 
  //2. Keep sensor #1 awake by keeping XSHUT pin high, Put all other sensors into shutdown by pulling XSHUT pins low
  pinMode(37, OUTPUT);
  pinMode(39, OUTPUT);
  digitalWrite (37, LOW);
  digitalWrite (39, LOW);
 
  //3. Init sensor #1  and change its adress to 0x30
  Serial.println("Init ToF VL53L0X 1");
  a = VL53L0X.init((uint8_t)0x30,(uint8_t)35);
  if (a)
  {
     Serial.println("Call init OK");
  }
  else
  {
     status = VL53L0X.VL53L0X_getStatus();
     Serial.print("Call init KO, error: ");Serial.println(status);
  }
  
  VL53L0X.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  VL53L0X.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  VL53L0X.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  VL53L0X.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  VL53L0X.setMeasurementTimingBudget(200000);
#elif defined MEDIUM_ACCURACY
  // set timing budget to 33 ms
  VL53L0X.setMeasurementTimingBudget(33000);  
#endif
  
  reg16 = VL53L0X.getModelId();
  status = VL53L0X.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getModelId KO, I2C error: ");Serial.println(status);
  }
  else
  {
     Serial.print("ModelId: ");Serial.println(reg16);
  }   
  reg8   = VL53L0X.getRevisionId();
  status = VL53L0X.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getRevisionId KO, I2C error: ");Serial.println(status);     
  }
  else
  {
     Serial.print("RevisionId: ");Serial.println(reg8);     
     
     address = VL53L0X.VL53L0X_getAddress();
     Serial.print("VL53L0X Address: 0x");Serial.println(address,HEX);  
     delay(2000); 
       
     d = VL53L0X.VL53L0X_readMillimeters();
     Serial.print("Distance in mm (max 1200mm): ");Serial.println(d);
     Serial.println("Init ToF VL53L0X OK");     
  } 

  //4. bring sensor #2 out of reset by setting its XSHUT pin high and Init sensor #2 and change its adress to 0x31
  Serial.println("Init ToF VL53L0X 2");
  a = VL53L0X2.init((uint8_t)0x31,(uint8_t)37);
  if (a)
  {
     Serial.println("Call init OK");
  }
  else
  {
     status = VL53L0X2.VL53L0X_getStatus();
     Serial.print("Call init KO, error: ");Serial.println(status);
  }  
    
  VL53L0X2.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  VL53L0X2.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  VL53L0X2.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X2.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  VL53L0X2.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  VL53L0X2.setMeasurementTimingBudget(200000);
#elif defined MEDIUM_ACCURACY
  // set timing budget to 33 ms
  VL53L0X2.setMeasurementTimingBudget(33000);  
#endif

  reg16 = VL53L0X2.getModelId();
  status = VL53L0X2.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getModelId KO, I2C error: ");Serial.println(status);
  }
  else
  {
     Serial.print("ModelId: ");Serial.println(reg16);
  }   
  reg8   = VL53L0X2.getRevisionId();
  status = VL53L0X2.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getRevisionId KO, I2C error: ");Serial.println(status);     
  }
  else
  {
     Serial.print("RevisionId: ");Serial.println(reg8);     
     
     address = VL53L0X2.VL53L0X_getAddress();
     Serial.print("VL53L0X2 Address: 0x");Serial.println(address,HEX);  
     delay(2000); 
       
     d = VL53L0X2.VL53L0X_readMillimeters();
     Serial.print("Distance in mm (max 1200mm): ");Serial.println(d);
     Serial.println("Init ToF VL53L0X2 OK");     
  }  

  //5. bring sensor #3 out of reset by setting its XSHUT pin high and Init sensor #3 and change its adress to 0x32
  Serial.println("Init ToF VL53L0X 3");

  a = VL53L0X3.init((uint8_t)0x32,(uint8_t)39);
  if (a)
  {
     Serial.println("Call init OK");
  }
  else
  {
     status = VL53L0X3.VL53L0X_getStatus();
     Serial.print("Call init KO, error: ");Serial.println(status);
  } 
  
  VL53L0X3.setTimeout(500);

#if defined LONG_RANGE
  // lower the return signal rate limit (default is 0.25 MCPS)
  VL53L0X3.setSignalRateLimit(0.1);
  // increase laser pulse periods (defaults are 14 and 10 PCLKs)
  VL53L0X3.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
  VL53L0X3.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
#endif

#if defined HIGH_SPEED
  // reduce timing budget to 20 ms (default is about 33 ms)
  VL53L0X3.setMeasurementTimingBudget(20000);
#elif defined HIGH_ACCURACY
  // increase timing budget to 200 ms
  VL53L0X3.setMeasurementTimingBudget(200000);
#elif defined MEDIUM_ACCURACY
  // set timing budget to 33 ms
  VL53L0X3.setMeasurementTimingBudget(33000);  
#endif

  reg16 = VL53L0X3.getModelId();
  status = VL53L0X3.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getModelId KO, I2C error: ");Serial.println(status);
  }
  else
  {
     Serial.print("ModelId: ");Serial.println(reg16);
  }   
  reg8   = VL53L0X3.getRevisionId();
  status = VL53L0X3.VL53L0X_getStatus();
  if (status > 0)
  {
     Serial.print("getRevisionId KO, I2C error: ");Serial.println(status);     
  }
  else
  {
     Serial.print("RevisionId: ");Serial.println(reg8);     
     
     address = VL53L0X3.VL53L0X_getAddress();
     Serial.print("VL53L0X3 Address: 0x");Serial.println(address,HEX);  
     delay(2000); 
       
     d = VL53L0X3.VL53L0X_readMillimeters();
     Serial.print("Distance in mm (max 1200mm): ");Serial.println(d);
     Serial.println("Init ToF VL53L0X3 OK");     
  }  
  
}
void loop() {

   uint16_t d;
   long starttime, elapstime;   
   
   starttime =millis();  
   d = VL53L0X.VL53L0X_readMillimeters();
   elapstime = millis() - starttime;
   Serial.print("VL53L0X  - Distance in mm (max 1200mm): ");Serial.print(d);  Serial.print(" - elapse time time (ms): ");Serial.println(elapstime);
   
   starttime =millis();  
   d = VL53L0X2.VL53L0X_readMillimeters();
   elapstime = millis() - starttime;
   Serial.print("VL53L0X2  - Distance in mm (max 1200mm): ");Serial.print(d);  Serial.print(" - elapse time (ms): ");Serial.println(elapstime);  
   
   starttime =millis();  
   d = VL53L0X3.VL53L0X_readMillimeters();
   elapstime = millis() - starttime;
   Serial.print("VL53L0X3  - Distance in mm (max 1200mm): ");Serial.print(d);  Serial.print(" - elapse time (ms): ");Serial.println(elapstime); 
}