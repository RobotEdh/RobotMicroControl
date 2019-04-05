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
DS1307Class DS1307;               // The RTC class  

    
uint32_t currentTime;
static uint32_t PIDTime = 0;
static uint32_t lastTime = 0;
double sampleTime = 0.0;
 
int16_t RC_command[NBCHANNELS];
int16_t ESC_command[NBMOTORS];
double YawInit = 0.0;
double anglePID[3] = {0.0,0.0,0.0};

void print_time()
{
  uint8_t status = 0;
  DateTime_t now; 
   
   status = DS1307.DS1307_read_current_datetime(&now);
   if (status > 0)
   {
      PRINT("DS1307_read_current_datetime KO, I2C error: ",status)
   }
   else
   {
      PRINTd
      PRINTt
   } 
}
  
DroneClass::DroneClass(void) {
}

void DroneClass::Drone_init() {
   
  uint8_t status = 0; 
  PRINTbegin
  
   // initialize RTC
  PRINTs(" ")
  status = DS1307.DS1307_init();
  if (status == ERROR_RTC_STOPPED)
  {
     PRINTs("RTC stopped") 
  }  
  else if (status > 0)
  {   
     PRINT("Init DS1307 KO, I2C error:",status) 
  }
  else
  {
     uint8_t address = DS1307.DS1307_getAddress();
     PRINTx("Init RTC DS1307 OK, address: ",address)
 
     print_time();    
  }
   
  PRINTs(" ")  
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
  
  YawInit = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getCompassHighResolution KO, I2C error: ",status)
  }
  else
  {           
        PRINT("YawInit: ",YawInit)
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
  
  print_time(); 
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
        ESC_command[YAW]      = (int16_t)anglePID[2];
        MotorESC.MotorESC_RunMotors(ESC_command);
        
        PRINTflush
     }       
     lastTime = currentTime;   
  }
}

void DroneClass::Drone_pid() {
  
  uint8_t status = 0; 

  double RC_commandRP[3]; // commands Roll & Pitch & Yaw
  double angle[3]; // Roll & Pitch & Yaw measured
   
  double error = 0.0;
  double delta_error[3] = {0.0,0.0,0.0};
  static double last_error[3] = {0.0,0.0,0.0};
  static double last_delta_error[3] = {0.0,0.0,0.0};
  static double sum_error[3] = {0.0,0.0,0.0};
  const char szAngles[3][20]={"Roll","Pitch","Yaw"};

  // Get RC commands
  RC.RC_getCommands(RC_command); // int16_t range [-90;+90]for ROLL, PITCH and range [-90;+90] for YAW
  
  // Read IMU
  angle[0] = (double)CMPS12.CMPS12_getRoll ();  // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getRoll KO, I2C error: ",status)
  }
  
  angle[1] = (double)CMPS12.CMPS12_getPitch (); // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getPitch KO, I2C error: ",status)
  }
       
  angle[2] = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINT("CMPS12_getCompassHighResolution KO, I2C error: ",status)
  }  
  
  // Compute PID for ROLL & PITCH
  //RC_commandRP[0]= (double)RC_command[ROLL];
  //RC_commandRP[1]= (double)RC_command[PITCH];
  //RC_commandRP[2]= (double)RC_command[YAW] + YawInit;
  RC_commandRP[0]=0.0;
  RC_commandRP[1]=0.0;
  RC_commandRP[3]=YawInit;
  if (RC_commandRP[3] > 359.9)    RC_commandRP[3] =  RC_commandRP[3] - 359.9;
  else if (RC_commandRP[3] < 0.0) RC_commandRP[3] =  359.9 + RC_commandRP[3];  
    
  if (RC_command[THROTTLE] == 0) // reset PID
  {           
     for (int j=0;j<3;j++) {
         sum_error[j] = 0.0; 
         last_error[j] = 0.0;
         last_delta_error[j] = 0.0;
     }   
  }
      
  for (int i=0;i<3;i++) {
    error =  angle[i] - RC_commandRP[i];
    
    sum_error[i] += error;
    sum_error[i] = constrain(sum_error[i],-_IMax,_IMax); // cap Integral
    
    delta_error[i] = (error - last_error[i]);
    /// Low pass filter cut frequency for derivative calculation, cuts out the high frequency noise that can drive the controller crazy
    delta_error[i] = last_delta_error[i] + (sampleTime / ( _filter + sampleTime)) * (delta_error[i] - last_delta_error[i]);

    last_error[i] = error;
    last_delta_error[i] = delta_error[i];
    
    anglePID[i] = (_Kp[i]*error) + (_Ki[i]*sum_error[i]*sampleTime) + (_Kd[i]*delta_error[i]/sampleTime);
    
    PRINT("For|",szAngles[i])
    PRINTi2("angle",i,angle[i])
    PRINTi2("rcCommand",i,RC_commandRP[i])
    PRINT("error|",error)
    PRINTi2("sum_error",i,sum_error[i])
    PRINTi2("delta_error",i,delta_error[i])
    PRINTi2("anglePID",i,anglePID[i])
  }  // end for
}
   