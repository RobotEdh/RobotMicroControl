#include <CMPS12.h>     // Compas
#include <RC.h>
#include <MotorESC.h>
#include <Drone.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
#define LOGTRACE   // Enable trace
#include <log.h>
extern File logFile;   
   
CMPS12Class CMPS12;               // The Compass class    
RCClass       RC;                 // The Radio Command class
MotorESCClass MotorESC;           // The Motor ESC Class

    
uint32_t currentTime;
static uint32_t PIDTime = 0;
static uint32_t lastTime = 0;
double sampleTime = 0.0;
 
int16_t RC_command[NBCHANNELS];
int16_t ESC_command[4];
double anglePID[2] = {0.0,0.0};

  
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
  
  PRINTflush
}


void DroneClass::Drone_main() {
  
  // Compute PID according the sample period
  currentTime = millis();
  if ((currentTime > PIDTime )||(PIDTime  == 0)) { 
     PIDTime = currentTime + samplePeriod;
     if (lastTime > 0) {
        sampleTime = (double)(currentTime - lastTime);
        PRINT("sampleTime (ms): ",sampleTime)
        
        Drone_pid();
        
        // call MotorESC
        ESC_command[THROTTLE] = RC_command[THROTTLE];
        ESC_command[ROLL]     = (int16_t)anglePID[0];
        ESC_command[PITCH]    = (int16_t)anglePID[1];
        ESC_command[YAW]      = 0; 
        MotorESC.MotorESC_RunMotors(ESC_command);
        
        PRINTflush
     }       
     lastTime = currentTime;   
  }
}

void DroneClass::Drone_pid() {
  
  uint8_t status = 0; 

  int16_t RC_commandRP[2]; // commands Roll & Pitch
  int16_t angle[2]; // Roll & Pitch measured
   
  double error       = 0.0;
  double delta_error[2] = {0.0,0.0};
  static double last_error[2] = {0.0,0.0};
  static double last_delta_error[2] = {0.0,0.0};
  static double sum_error[2] = {0.0,0.0};
  const char szAngles[2][20]={"Roll","Pitch"};

  // Get RC commands
  RC.RC_getCommands(RC_command); // int16_t range [-90;+90]            for ROLL, PITCH, YAW
  
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
    PRINT("->error: ",error)
    PRINTi("->sum_error",i,sum_error[i])
    PRINTi("->delta_error",i,delta_error[i])
    PRINTi("->anglePID",i,anglePID[i])
  }  // end for
}