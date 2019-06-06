#include <Drone.h>

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
#include <log.h>
extern File logFile;   
   
CMPS12Class CMPS12;               // The Compass class    
RCClass       RC;                 // The Radio Command class
MotorESCClass MotorESC;           // The Motor ESC Class
DS1307Class DS1307;               // The RTC class  

    
struct PID_record_type  // 12 bytes
{
     uint8_t angleType;
     int8_t angle;
     int8_t RC_commandRP;
     int8_t error;
     int8_t sum_error;
     int8_t delta_error;
     int8_t anglePID;
     uint8_t sampleTime;
     uint32_t tick;
     
};
#define PIDLOGDATASIZE 42
struct PID_record_block_type {  //(struct 12 bytes * 42 = 504 + 8 bytes start/stop)
     const uint8_t startPIDLog[2]={0xFA,0xFB};
     PID_record_type PID_record[PIDLOGDATASIZE];
     const uint8_t stopPIDLog[6] ={0xFC,0xFD,0xFD,0xFD,0xFD,0xFD}; 
};
PID_record_block_type PID_record_block;
int PID_t = 0;

uint32_t tick = 1;
uint32_t PIDTime = 0;
uint32_t lastTime = 0;
double sampleTime = 0.0; 
int16_t RC_command[NBCHANNELS];
double YawInit = 0.0;
double anglePID[3] = {0.0,0.0,0.0};

bool init_OK = true;
int go = -1;

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

uint8_t DroneClass::Yaw_init() {
  
  uint8_t status = 0; 
  double SumYawInit = 0.0;
  
  for (int i=0;i<3;i++) {
     YawInit = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
     if (status > 0)
     {           
        PRINT("CMPS12_getCompassHighResolution KO, I2C error: ",status)
        return status;
     }
     SumYawInit += YawInit;
  }
  
  YawInit = SumYawInit/3.0;
  PRINT("YawInit: ",YawInit)
  
  return 0;
  
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
   
  PRINTs(">Start Init Drone")

 // initialize the Compass CMPS12
  PRINTs("Start Init Compass CMPS12")
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
        init_OK = false;
     }    
  }
  
  status = Yaw_init ();
  if (status > 0)
  {           
     PRINT("Yaw_init KO, Error: ",status)
     init_OK = false;
  }
        
  PRINTs("Start Init RC")
  RC.RC_init();
  PRINTs("Init RC OK")
  
  PRINTs("Start Init Motor ESC")
  MotorESC.MotorESC_init();
  PRINTs("Init Motor ESC OK")
  
  PRINTs("<End OK Init Drone")
  
  print_time(); 
}


void DroneClass::Drone_main() {
    
  int16_t ESC_command[NBMOTORS];
   
  // Compute PID according the sample period
  uint32_t currentTime = millis();
  if ((currentTime >= PIDTime )||(PIDTime == 0)) { 
     PIDTime = currentTime + samplePeriod;
     if (lastTime > 0) {
        sampleTime = (double)(currentTime - lastTime);
        
        Drone_pid();
        
        // call MotorESC
        if (!init_OK ) {
           ESC_command[THROTTLE] = 0;
           ESC_command[ROLL]     = 0;
           ESC_command[PITCH]    = 0;
           ESC_command[YAW]      = 0;
           PRINTs(" init KO => ESC_command = 0")
        }
        else       
        {
           ESC_command[THROTTLE] = RC_command[THROTTLE];
           ESC_command[ROLL]     = (int16_t)anglePID[0];
           ESC_command[PITCH]    = (int16_t)anglePID[1];
           ESC_command[YAW]      = (int16_t)anglePID[2];
        }   
        MotorESC.MotorESC_RunMotors(ESC_command, tick);
        
     }       
     lastTime = currentTime;
     if (go > -1)tick++;   
  }
}

void DroneClass::Drone_pid() {
  
  uint8_t status = 0; 
  int count = -1;

  double RC_commandRP[3]; // commands Roll & Pitch & Yaw
  double angle[3]; // Roll & Pitch & Yaw measured
   
  double error = 0.0;
  double delta_error[3] = {0.0,0.0,0.0};
  static double last_error[3] = {0.0,0.0,0.0};
  static double last_delta_error[3] = {0.0,0.0,0.0};
  static double sum_error[3] = {0.0,0.0,0.0};

  // Get RC commands
  RC.RC_getCommands(RC_command); // int16_t range [-45;+45] for ROLL, PITCH and range [-90;+90] for YAW
  
  if ((RC_command[THROTTLE] > 0) && (go != 1)) // reset PID and Yaw init if before starting
  {           
     go = 1; 
     for (int j=0;j<3;j++) {
         sum_error[j] = 0.0; 
         last_error[j] = 0.0;
         last_delta_error[j] = 0.0;
     } 
     
     status = Yaw_init ();
     if (status > 0)
     {           
        PRINT("Yaw_init KO, Error: ",status)
        init_OK = false;
     }
     else PRINTi2("Yaw_init: ",tick,YawInit) 
  }
  else if ((RC_command[THROTTLE] == 0) && (go != -1))  // already started
  { 
     if (PID_t > 0) { // force dump
          for(int z=PID_t; z< PIDLOGDATASIZE+1; z++) PID_record_block.PID_record[PID_t].tick = 0;// reset end tab 
          count = logFile.write((const uint8_t *)&PID_record_block,  512);  
          if (count != 512) PRINTi2("bad count written: ",tick,count)
          PID_t = 0;                                            
     }
     
     go = 0; 
     return;
  } 
  else if (go == -1)
  {
      return;  // not started yet
  } 
   
  // Read CMPS12
  angle[0] = (double)CMPS12.CMPS12_getRoll ();  // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINTi2("CMPS12_getRoll KO, I2C error: ",tick,status)
  }
  
  angle[1] = (double)CMPS12.CMPS12_getPitch (); // signed byte giving angle in degrees from the horizontal plane (+/- 90 degrees)
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINTi2("CMPS12_getPitch KO, I2C error: ",tick,status)
  }
       
  angle[2] = CMPS12.CMPS12_getCompassHighResolution();  //  0-359 degrees
  status = CMPS12.CMPS12_getStatus();
  if (status > 0)
  {           
     PRINTi2("CMPS12_getCompassHighResolution KO, I2C error: ",tick,status)
  }  
  
  // Compute PID for ROLL & PITCH
  //RC_commandRP[0]= (double)RC_command[ROLL];
  //RC_commandRP[1]= (double)RC_command[PITCH];
  //RC_commandRP[2]= (double)RC_command[YAW] - YawInit;
  RC_commandRP[0]=0.0;
  RC_commandRP[1]=0.0;
  RC_commandRP[2]=0.0;
      
  for (int i=0;i<3;i++) {
     
    if (i == 2) {  // only for Yaw
       //subtract YawInit and convert range: 0-359 degrees  to -180, +180
       if (angle[2] - YawInit > 180.0)       angle[2] += - YawInit - 360.0;   
       else if (angle[2] - YawInit < -180.0) angle[2] += - YawInit + 360.0;
       else                                  angle[2] += - YawInit;  
    }
    
    error = angle[i] - RC_commandRP[i]; 
    
    sum_error[i] += error;
    
    // cap Integral
    sum_error[i] = constrain(sum_error[i],-_IMax,_IMax); 
    
    delta_error[i] = (error - last_error[i]);
    
    /// Low pass filter cut frequency for derivative calculation, cuts out the high frequency noise that can drive the controller crazy
    delta_error[i] = last_delta_error[i] + (samplePeriod / ( _filter + samplePeriod)) * (delta_error[i] - last_delta_error[i]);
    
    last_error[i] = error;
    last_delta_error[i] = delta_error[i];
    
    anglePID[i] = (_Kp[i]*error) + (_Ki[i]*sum_error[i]*samplePeriod) + (_Kd[i]*delta_error[i]/samplePeriod);
    
  #ifdef LOGSERIAL  
    PRINTi2("angle",i,angle[i])
    PRINTi2("rcCommand",i,RC_commandRP[i])
    PRINT("error|",error)
    PRINTi2("sum_error",i,sum_error[i])
    PRINTi2("delta_error",i,delta_error[i])
    PRINTi2("anglePID",i,anglePID[i])
  #else  
    if ((tick%PIDLOGFREQ) == 0 ) { // record every 5 ticks ie 100 ms at 50Hz
       PID_record_block.PID_record[PID_t].angleType = (uint8_t)i;
       PID_record_block.PID_record[PID_t].angle = (int8_t)angle[i];
       PID_record_block.PID_record[PID_t].RC_commandRP = (int8_t)RC_commandRP[i];
       PID_record_block.PID_record[PID_t].error = (int8_t)error;
       PID_record_block.PID_record[PID_t].sum_error = (int8_t)sum_error[i];
       PID_record_block.PID_record[PID_t].delta_error = (int8_t)delta_error[i];
       PID_record_block.PID_record[PID_t].anglePID = (int8_t)anglePID[i]; 
       PID_record_block.PID_record[PID_t].sampleTime = (uint8_t)sampleTime;
       PID_record_block.PID_record[PID_t].tick = tick;
       PID_t++;
       if (PID_t == PIDLOGDATASIZE) { // need to dump 
          count = logFile.write((const uint8_t *)&PID_record_block, 512);
          if (count != 512) PRINTi2("bad count written: ",tick,count)
          PID_t = 0;                                           
       }
    }
#endif    
  }  // end for
}
   