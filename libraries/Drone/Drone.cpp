#include <CMPS12.h>     // Compas
#include <RC.h>
#include <MotorESC.h>
#include <Drone.h>

// Logging mode
#define  LOGSERIAL
//#define LOGSDCARD  // log to SD Card
//#define LOGTRACE   // Enable trace
#include <log.h>
//extern File logFile;   
   
CMPS12Class CMPS12;               // The Compass class    
RCClass       RC;                 // The Radio Command class
MotorESCClass MotorESC;           // The Motor ESC Class


DroneClass::DroneClass(void) {
}

void DroneClass::Drone_init() {
   
  uint8_t status = 0; 
  PRINTbegin
  
  PRINTs(">Start Init Drone")

 // initialize the Compass CMPS12
  PRINTs("Init Compass CMPS12")
  uint8_t calib = CMPS12.CMPS12_init();
  if(calib == 0) 
  { 
     PRINTs("Init compass OK")        
  }
  else
  {  
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        PRINTb("Calibrate CMPS12 KO, calibrate status: ",calib) 
     }
     else 
     {           
        PRINT("Init CMPS12 KO, I2C error: ",status)
     }    
  }
  PRINTs(" ")
  PRINTs("Init RC")
  RC.RC_init();
  PRINTs("Init RC OK")
  
  PRINTs(" ")
  PRINTs("Init Motor ESC")
  MotorESC.MotorESC_init();
  PRINTs("Init Motor ESC OK")
  
  PRINTs("<End OK Init Drone")
}


void DroneClass::Drone_main() {
  
  uint8_t status = 0; 
    
  uint32_t currentTime, currentTimePID;
  static uint32_t rcTime = 0;
  static uint32_t lastTimePID = millis();
 
  int16_t RC_command[NBCHANNELS];
  int16_t RC_commandRP[2]; // commands Roll & Pitch
  int16_t angle[2]; // Roll & Pitch measured
  int16_t ESC_command[4];
   
  double sampleTime = 0.0;
  double anglePID[2] = {0.0,0.0};;
  double error       = 0.0;
  double delta_error[2] = {0.0,0.0};
  static double last_error[2] = {0.0,0.0};
  static double last_delta_error[2] = {0.0,0.0};
  static double sum_error[2] = {0.0,0.0};
  const char szAngles[2][20]={"Roll","Pitch"};
  
  // Get RC commands every 20 ms
  currentTime = millis();
  if ((currentTime > rcTime )|| (rcTime  == 0)) { // 50Hz: PPM frequency of the RC, no change happen within 20ms except first time
    rcTime = currentTime + 20;
    
    RC.RC_getCommands(RC_command); // int16_t range [-90;+90]            for ROLL, PITCH, YAW
                                   //               0 or [MINPPM;MAXPPM] for THROTTLE
                                   //               0/1                  for AUX1, AUX2
  }
  
  // Read IMU  
  angle[0] = (int16_t)CMPS12.CMPS12_getRoll ();  // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getRoll KO, I2C error: ",status)
  }
  
  angle[1] = (int16_t)CMPS12.CMPS12_getPitch (); // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getPitch KO, I2C error: ",status)
  }

   
  // Compute PID for ROLL & PITCH
  RC_commandRP[0]=RC_command[ROLL];
  RC_commandRP[1]=RC_command[PITCH];
  
  currentTimePID = millis();
  sampleTime = (double)(lastTimePID - currentTimePID);
  lastTimePID = currentTimePID;
  for (int i=0;i<2;i++) {	
    error =  (double)(RC_commandRP[i] - angle[i]);
    
    sum_error[i] += error;
    sum_error[i] = constrain(sum_error[i],-_IMax,_IMax); // cap Integral
    
    delta_error[i] = (error - last_error[i]);
    /// Low pass filter cut frequency for derivative calculation, cuts out the high frequency noise that can drive the controller crazy
    delta_error[i] = last_delta_error[i] + (sampleTime / ( _filter + sampleTime)) * (delta_error[i] - last_delta_error[i]);

    last_error[i] = error;
    last_delta_error[i] = delta_error[i];
    
    anglePID[i] = (_Kp[i]*error) + (_Ki[i]*sum_error[i]*sampleTime) + (_Kd[i]*delta_error[i]/sampleTime);
    
    PRINT("For ",szAngles[i])
    PRINTi("->angle",i,angle[i])
    PRINTi("->rcCommand",i,RC_commandRP[i])
    PRINTi("->error",i,error)
    PRINTi("->sum_error",i,sum_error[i])
    PRINTi("->delta_error",i,delta_error[i])
    PRINTi("->anglePID",i,anglePID[i])
  }

  // call MotorESC
  ESC_command[THROTTLE] = RC_command[THROTTLE];
  ESC_command[ROLL]     = (int16_t)anglePID[0];
  ESC_command[PITCH]    = (int16_t)anglePID[1];
  ESC_command[YAW]      = 0; 
  MotorESC.MotorESC_RunMotors(ESC_command);
}