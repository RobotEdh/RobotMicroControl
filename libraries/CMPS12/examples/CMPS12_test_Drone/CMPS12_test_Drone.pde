#include <Arduino.h>
#include <CMPS12.h>

CMPS12Class CMPS12;
double YawInit;

void setup()
{
  uint8_t status;
   
  Serial.begin(9600); // initialize serial port
  
  Wire.begin(); // initialize I2C

  uint8_t calib = CMPS12.CMPS12_init(false); // initialize CMPS12, Do not calibration
  
  if (calib == 0)
  {
       Serial.println("Init OK");
  }
  else 
  {           
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        Serial.print("Calibrate KO, calibrate status: 0b");Serial.println(calib,BIN);    
     }
     else 
     {           
        Serial.print("Init KO, I2C error: ");Serial.println(status);  
     }
  }
  
  YawInit = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
        Serial.print("CMPS12_getCompassHighResolution KO, I2C error: ");Serial.println(status);
  }
  else
  {           
        Serial.print("YawInit: ");Serial.println(YawInit);
  }
      
}

void loop()
{
  uint8_t status = 0; 

  int16_t angle[3]; // Roll & Pitch & Yaw measured

  
  // Read IMU
  angle[0] = (int16_t)CMPS12.CMPS12_getRoll ();  // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
        Serial.print("CMPS12_getRoll KO, I2C error: ");Serial.println(status);  
  }
  else
  {           
        Serial.print("angle[0]: ");Serial.println(angle[0]);  
  }
  
  angle[1] = (int16_t)CMPS12.CMPS12_getPitch (); // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
         Serial.print("CMPS12_getPitch KO, I2C error: ");Serial.println(status);  
  }
  else
  {           
        Serial.print("angle[1]: ");Serial.println(angle[1]);  
  }
  
  angle[2] = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
         Serial.print("CMPS12_getCompassHighResolution KO, I2C error: ");Serial.println(status); 
  }
  else
  {           
        Serial.print("angle[2]: ");Serial.println(angle[2]);  
  }  
  
  angle[2] = angle[2] - YawInit;
  Serial.print("angle[2]: ");Serial.println(angle[2]);
  
  if (angle[2] > 180.0)       angle[2] =  angle[2] - 360.0;
  else if (angle[2] < -180.0) angle[2] =  360.0 + angle[2]; 
  Serial.print("angle[2]: ");Serial.println(angle[2]);
  
  #define PIDMIX(X,Y,Z) (int16_t)angle[0]*X + (int16_t)angle[1]*Y + (int16_t)angle[2]*Z
  int16_t motor[4];
  const char szMotors[4][20]={    
    "FRONT_LEFT",
    "FRONT_RIGHT",
    "REAR_RIGHT",
    "REAR_LEFT"
  };
  motor[0] = PIDMIX(-1,-1,+1); //Front Left
  motor[1] = PIDMIX(+1,-1,-1); //Front Right
  motor[2] = PIDMIX(+1,+1,+1); //Rear Right
  motor[3] = PIDMIX(-1,+1,-1); //Rear Left
    
  for(int i=0; i< 4; i++) {
      Serial.print(szMotors[i]);Serial.print(": ");Serial.println(motor[i]); 
  }
  delay (5000);
  
 }