#include <Drone2.h>

// Logging mode
#define  LOGSERIAL
//#define LOGSDCARD  // log to SD Card
//#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
#include <log.h>
//extern File logFile;   
   
MPU6050Class   MPU6050;          // The MPU6050 class    
RCClass        RC;               // The Radio Command class
MotorESC2Class MotorESC;         // The Motor ESC Class
DS1307Class    DS1307;           // The RTC class  

    
struct PID_record_type  // 12 bytes
{
     uint8_t angleType;
     int8_t gyro;
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
double RC_angularspeedcommand[NBCHANNELS];
double YawInit = 0.0;
double anglePID[3] = {0.0,0.0,0.0};

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
  
Drone2Class::Drone2Class(void) {
}


void Drone2Class::Drone_init() {
   
  uint8_t status = 0; 
  
  // initialize serial port
  Serial.begin(115200);
  PRINTbegin
  
  // initialize I2C
  Wire.begin(); 
  Wire.setClock(400000); //set I2C SCL to High Speed Mode of 400kHz
  
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

  // initialize the MPU6050
  PRINTs("Start Init MPU6050")
  status = MPU6050.MPU6050_init();
  if (status > 0)
  {   
     PRINT("Init MPU6050 KO:",status) 
  }
  else
  {
     PRINTs("Init MPU6050 OK")
 
     print_time();    
  }
  
  // calibrate the MPU6050
  PRINTs("Start Init MPU6050")
  status = MPU6050.MPU6050_calibrate();
  if (status > 0)
  {   
     PRINT("Calibrate MPU6050 KO:",status) 
  }
  else
  {
     PRINTs("Calibrate MPU6050 OK")
 
     print_time();    
  }
  status =  MPU6050.MPU6050_checkDeviceID();
  if (status > 0) {
     PRINT("Wrong MPU6050 device ID:",status)
     PRINTx(" Should be: ",MPU6050_ID) 
  }
  else
  {
     PRINTx("MPU6050_checkDeviceID OK : ",MPU6050_ID) 
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


void Drone2Class::Drone_main() {
    
  uint8_t status = 0;
  int16_t ESC_command[NBMOTORS];
   
  // Compute PID according the sample period
  uint32_t currentTime = millis();
  if ((currentTime >= PIDTime )||(PIDTime == 0)) { 
     PIDTime = currentTime + samplePeriod;
     if (lastTime > 0) {
        sampleTime = (double)(currentTime - lastTime);
        
        status = Drone_pid();
        
        // call MotorESC
        if ((status == 0) && (go > -1)) {
           ESC_command[THROTTLE] = RC_angularspeedcommand[THROTTLE];
           ESC_command[ROLL]     = (int16_t)anglePID[0];
           ESC_command[PITCH]    = (int16_t)anglePID[1];
           ESC_command[YAW]      = (int16_t)anglePID[2];
           
           MotorESC.MotorESC_MixPID(ESC_command, tick);
        }   
        
     }       
     lastTime = currentTime;
     if (go > -1)tick++;   
  }
}

uint8_t Drone2Class::Drone_pid() {
  
  uint8_t status = 0; 

  double angle[2]; // angle Roll & Pitch computed
  double gyro[3]; // gyro Roll & Pitch computed
  
  double error = 0.0;
  double delta_error[3] = {0.0,0.0,0.0};
  static double last_error[3] = {0.0,0.0,0.0};
  static double last_delta_error[3] = {0.0,0.0,0.0};
  static double sum_error[3] = {0.0,0.0,0.0};
  
  int count = -1; 

  // Get RC commands
  RC.RC_getAngularSpeedCommands(RC_angularspeedcommand); 
  
  if ((RC_angularspeedcommand[THROTTLE] > 0.0) && (go != 1)) // reset PID before starting
  {           
     go = 1; 
     for (int j=0;j<3;j++) {
         sum_error[j] = 0.0; 
         last_error[j] = 0.0;
         last_delta_error[j] = 0.0;
     } 
     
  }
  else if ((RC_angularspeedcommand[THROTTLE] == 0.0) && (go != -1))  // already started
  { 
#ifdef LOGSDCARD
     if (PID_t > 0) { // force dump
          for(int z=PID_t; z<PIDLOGDATASIZE; z++) PID_record_block.PID_record[z].tick = 0;// reset end tab 
          count = logFile.write((const uint8_t *)&PID_record_block,  512);  
          if (count != 512) PRINTi2("bad count written PID following force dump: ",tick,count)
          Serial.print("written PID following force dump,tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(count);
          PID_t = 0;                                            
     }
#endif     
     go = 0;
     for (int j=0;j<3;j++) {
         sum_error[j] = 0.0; 
         last_error[j] = 0.0;
         last_delta_error[j] = 0.0;
         anglePID[j] = 0.0;
     } 
     return 0;
  } 
  else if (go == -1)
  {
     return 0;  // not started yet
  } 
   
  status = MPU6050.MPU6050_compute_angle_gyro(samplePeriod, angle, gyro);
  if (status > 0) return status;
      
  for (int i=0;i<3;i++) {
    if ((i == PITCH)|| (i == ROLL)) RC_angularspeedcommand[i] -=  angle[i] * coeff_stabilisation;

    error = RC_angularspeedcommand[i] - gyro[i]; 
    
    sum_error[i] += error;
    
    // cap Integral
    sum_error[i] = constrain(sum_error[i],-_IMax,_IMax); 
    
    delta_error[i] = (error - last_error[i]);
    
    /// Low pass filter cut frequency for derivative calculation, cuts out the high frequency noise that can drive the controller crazy
    delta_error[i] = last_delta_error[i] + (samplePeriod / ( _filter + samplePeriod)) * (delta_error[i] - last_delta_error[i]);
    
    last_error[i] = error;
    last_delta_error[i] = delta_error[i];
    
    anglePID[i] = (_Kp[i]*error) + (_Ki[i]*sum_error[i]*samplePeriod) + (_Kd[i]*delta_error[i]/samplePeriod);
    anglePID[i] = constrain(anglePID[i],-_PIDMax,_PIDMax); 

    
  #ifdef LOGSERIAL  
    PRINTi2("gyro",i,gyro[i])
    PRINTi2("rcCommand",i,RC_angularspeedcommand[i])
    PRINT("error|",error)
    PRINTi2("sum_error",i,sum_error[i])
    PRINTi2("delta_error",i,delta_error[i])
    PRINTi2("anglePID",i,anglePID[i])
  #else  
    if ((tick%PIDLOGFREQ) == 0 ) { // record every PIDLOGFREQ ticks
       PID_record_block.PID_record[PID_t].angleType = (uint8_t)i;
       PID_record_block.PID_record[PID_t].gyro = (int8_t)gyro[i];
       PID_record_block.PID_record[PID_t].RC_commandRP = (int8_t)RC_angularspeedcommand[i];
       PID_record_block.PID_record[PID_t].error = (int8_t)error;
       PID_record_block.PID_record[PID_t].sum_error = (int8_t)sum_error[i];
       PID_record_block.PID_record[PID_t].delta_error = (int8_t)delta_error[i];
       PID_record_block.PID_record[PID_t].anglePID = (int8_t)anglePID[i]; 
       PID_record_block.PID_record[PID_t].sampleTime = (uint8_t)sampleTime;
       PID_record_block.PID_record[PID_t].tick = tick;
       PID_t++;
       if (PID_t == PIDLOGDATASIZE) { // need to dump 
          count = logFile.write((const uint8_t *)&PID_record_block, 512);
          if (count != 512) PRINTi2("bad count written PID following need to dump: ",tick,count)
          Serial.print("written PID following need to dump, tick: ");Serial.print(tick);Serial.print(" ,count: ");Serial.println(count);
          PID_t = 0;                                           
       }
    }
#endif    
  }  // end for
  
  return 0;
}
   
