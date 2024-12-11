#include <Drone3.h>

#define DEBUGLEVEL1
double rolldebug = 0.0;
double incrementdebug = 5.0;
#ifdef  DEBUGLEVEL0
 #define  DEBUGLEVEL1
#endif


// Logging mode
//#define  LOGSERIAL

#define LOGSDCARD  // log to SD Card
#ifdef  LOGSDCARD
#define AUTOFLUSH // auto flush following each write
//#define LOGTRACE   // Enable trace
#include <log.h> //SPI CS=10 
File logFile; 
int countwrite = -1; 

#define PIDLOGFREQ 1 //record every 1 ticks ie 20 ms at 50Hz

struct PID_record_type  
{
     uint8_t  index;
     uint8_t  dt;      //seconds (ms*1000)
     uint16_t countESC;      
     int16_t  instruction;  
     int16_t  angle;  //Degree*10
     int16_t  error;    //Error*10
     int16_t  lastError; //lastError*10
     int16_t  prop;      //prop*10
     int16_t  integ;     //integ*10
     int16_t  derivative;  //derivative*10
     int16_t  lastDerivative;  //lastDerivative*10
     int16_t  anglePID; //Degree*10
     uint16_t tick;
     
};
#define PIDLOGDATANB 21
struct PID_record_block_type {  //(struct 24 bytes * 21 = 504 + 8 bytes start/stop = 512)
     const uint8_t startPIDLog[2]={0xFA,0xFB}; 
     PID_record_type PID_record[PIDLOGDATANB];
     const uint8_t stopPIDLog[6] ={0xFC,0xFD,0xFD,0xFD,0xFD,0xFD}; 
};
PID_record_block_type PID_record_block;
uint8_t PID_t = 0;  
#endif
   
ICM20948Class  ICM20948;         // The ICM20948 class    
RCClass        RC;               // The Radio Command class
MotorESC2Class MotorESC;         // The Motor ESC Class

//#define  RTC
#ifdef RTC
DS1307Class    DS1307;           // The RTC class  
#endif
    
void print_time()
{
#ifdef RTC  
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
#endif    
}
  
Drone3Class::Drone3Class(void) {
}


void Drone3Class::Drone_init() {
   
  uint8_t status = 0; 
  
  // initialize serial port
  Serial.begin(115200);
  PRINTbegin

#ifdef RTC  
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
     delay(60000);
  }
  else
  {
     uint8_t address = DS1307.DS1307_getAddress();
     PRINTx("Init RTC DS1307 OK, address: ",address)
 
     print_time();    
  }
#endif 
   
  PRINTs(">Start Init Drone")
  
  status = Drone_init_ICM20948();
        
  PRINTs("Start Init RC")
  RC.RC_init();
  
  PRINTs("Init RC OK")
  
#ifndef DEBUGLEVEL1
  PRINTs("Start Init Motor ESC")
  MotorESC.MotorESC_init();
  PRINTs("Init Motor ESC OK")
#endif
  
  PRINTs("<End OK Init Drone")
  PRINTs("******************************")
  
  print_time(); 
}


void Drone3Class::Drone_main() {
    
  uint8_t status = 0;
  int16_t ESC_command[4];
  double anglePID[3] = {0.0,0.0,0.0};
  int16_t throttle = 0;
  uint32_t previousTime = 0;
  double dt;
  uint16_t tick = 0;
  uint16_t countESC = 0;
  
  while(1){    
  // loop until sample period reached, then compute PID and send command to MotorESC.
   if (previousTime == 0)
      previousTime = millis();
  
   else if ((millis() - previousTime) > _samplePeriod) { 
      dt = (double)(millis() - previousTime)/1000.0;

#ifdef DEBUGLEVEL1
      if (dt > ((double)(_samplePeriod+2)/1000.0)) {PRINT("dt: ",dt) PRINT("count call ESC: ",countESC)}
      if (countESC == 0) PRINT("count call ESC: ",countESC)
#endif  
      previousTime = millis();
     
      status = Drone_pid(dt, anglePID, throttle, tick, countESC);      
      if ((status == 0) && (_go != NOT_STARTED)) {
  // call MotorESC
           ESC_command[THROTTLE] = throttle;
           ESC_command[ROLL]     = (int16_t)anglePID[0];
           ESC_command[PITCH]    = (int16_t)anglePID[1];
           ESC_command[YAW]      = (int16_t)anglePID[2];

           MotorESC.MotorESC_MixPID(ESC_command, tick);
      }
          
      if (_go != NOT_STARTED)tick++;
 
   }
   else if (_go == RUNNING)
   {
      countESC++;
      MotorESC.MotorESC_pulsePWM();
   }
 
 } // end while (1)
}

uint8_t Drone3Class::Drone_pid(double dt, double anglePID[3], int16_t &throttle, uint16_t tick, uint16_t countESC) {
  
  uint8_t status = 0; 
         double instruction[3] = {0.0,0.0,0.0};          
         double angle[3] = {0.0,0.0,0.0};
         double error[3] = {0.0,0.0,0.0};
  static double lastError[3] = {0.0,0.0,0.0};
         double prop[3] = {0.0,0.0,0.0};
  static double integ[3] = {0.0,0.0,0.0};
         double derivative[3] = {0.0,0.0,0.0};
  static double lastDerivative[3] = {0.0,0.0,0.0};
        
  // Get instructions  
  Drone_get_instructions(instruction, throttle);

  if ((throttle > 0.0) && (_go != RUNNING)) // reset PID before starting
  {           
     _go = RUNNING; 
     for (int j=0;j<3;j++) {
         integ[j] = 0.0; 
         lastError[j] = 0.0;
         lastDerivative[j] = 0.0;
         anglePID[j] = 0.0;
     } 
     
  }
  else if ((throttle == 0.0) && (_go != NOT_STARTED))  // already started, stop command received
  { 
#ifdef LOGSDCARD
     if (PID_t > 0) { // force dump
          for(int z=PID_t; z<PIDLOGDATANB; z++) PID_record_block.PID_record[z].tick = 0;// reset end tab 
          countwrite = logFile.write((const uint8_t *)&PID_record_block,  512);  
          if (countwrite != 512) PRINTi2("bad count written PID following force dump: ",tick,countwrite)
          Serial.print("written PID following force dump,tick: ");Serial.print(tick);Serial.print(" ,countwrite: ");Serial.println(countwrite);
          PID_t = 0;                                            
     }
#endif     
     _go = STOPPED; 
     for (int j=0;j<3;j++) { // reset PID before stoping
         integ[j] = 0.0; 
         lastError[j] = 0.0;
         lastDerivative[j] = 0.0;
         anglePID[j] = 0.0;
     } 
     return 0;
  } 
  else if (_go == NOT_STARTED)
  {
     return 0;  // not started yet; no start command received
  } 
 
  // Get angles  
  status = Drone_get_angles(angle);
  if (status > 0) return status;

  for (int i=0;i<3;i++) {

    // compute error
    error[i] = instruction[i] - angle[i]; 
    
    // Compute proportionnal
    prop[i] = _Kp[i] * error[i];
    
    // compute integral
    integ[i] += _Ki[i] * error[i] * dt;
    integ[i] = constrain(integ[i],-_IMax,_IMax); // cap Integral
    
    // compute derivative
    derivative[i] = (error[i] - lastError[i]) / dt;
    derivative[i] = lastDerivative[i] + (dt / (dt + _lowPassFilter)) * (derivative[i] - lastDerivative[i]); // Low pass filter cut frequency for derivative calculation
    
    lastError[i] = error[i];
    lastDerivative[i] = derivative[i];
    derivative[i] = _Kd[i] * derivative[i];

    // compute PID
    anglePID[i] = prop[i] + integ[i] + derivative[i];
    anglePID[i] = constrain(anglePID[i],-_PIDMax,_PIDMax); 
    
#ifdef LOGSERIAL 
#ifdef DEBUGLEVEL0 
    PRINT ("dt: ",dt)
    PRINT ("countESC: ",countESC)
    PRINTi2("instruction",i,instruction[i])
    PRINTi2("angle",i,angle[i])
    PRINTi2("error",i,error[i])
    PRINTi2("prop",i,prop[i])
    PRINTi2("integ",i,integ[i])
    PRINTi2("derivative",i,derivative[i])
    PRINTi2("anglePID",i,anglePID[i])
#endif 
#endif    
#ifdef LOGSDCARD 
    if ((tick%PIDLOGFREQ) == 0 ) { // record every PIDLOGFREQ ticks
       PID_record_block.PID_record[PID_t].index = (uint8_t)i;       
       PID_record_block.PID_record[PID_t].dt = (uint8_t)(dt*1000.0);
       PID_record_block.PID_record[PID_t].countESC = (uint16_t)countESC;
       PID_record_block.PID_record[PID_t].instruction = (int16_t)(instruction[i]*10.0);
       PID_record_block.PID_record[PID_t].angle = (int16_t)(angle[i]*10.0);
       PID_record_block.PID_record[PID_t].error = (int16_t)(error[i]*10.0);
       PID_record_block.PID_record[PID_t].lastError = (int16_t)(lastError[i]*10.0);
       PID_record_block.PID_record[PID_t].prop = (int16_t)(prop[i]*10.0);
       PID_record_block.PID_record[PID_t].integ = (int16_t)(integ[i]*10.0);
       PID_record_block.PID_record[PID_t].derivative = (int16_t)(derivative[i]*10.0); 
       PID_record_block.PID_record[PID_t].lastDerivative = (int16_t)(lastDerivative[i]*10.0);        
       PID_record_block.PID_record[PID_t].anglePID = (int16_t)(anglePID[i]*10.0);
       PID_record_block.PID_record[PID_t].tick = (uint16_t)tick;
       PID_t++;
       if (PID_t == PIDLOGDATANB) { // need to dump 
          countwrite = logFile.write((const uint8_t *)&PID_record_block, 512);
          PID_t = 0;
#ifdef LOGSERIAL           
          if (countwrite != 512) PRINTi2("bad count written PID following need to dump: ",tick,countwrite)
          Serial.print("written PID following need to dump, tick: ");Serial.print(tick);Serial.print(" ,countwrite: ");Serial.println(countwrite);
#endif                                                       
       }
    }
#endif    
  }  // end for

  return 0;
}
   
   

uint8_t Drone3Class::Drone_get_angles(double angle[3])
{
  icm20948_DMP_data_t data;
  double q0, q1, q2, q3;
  uint8_t status = 0;
  static unsigned long count = 0;
  static unsigned long counterr = 0;

#ifdef DEBUGLEVEL1 
    angle[0] = 0;
    angle[1] = 0;
    angle[2] = 0; 
    return 0;
#endif  

  // Read any DMP data waiting in the FIFO
  status = ICM20948.ICM20948_readDMPdataFromFIFO(&data);
  if ((status == DMP_NO_ERR) || (status == DMP_FIFO_NOT_EMPTY)) // Was valid data available?
  { 
#ifdef DEBUGLEVEL0 
     PRINTx("Received data! Header: ",data.header) // Print the header in HEX so we can see what data is arriving in the FIFO
#endif     
    if ((data.header & DMP_header_bitmap_Quat6) > 0)
    {
      
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
      
      // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
      // Orientation of Axes of Sensitivity and Polarity of Rotation
      // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
      //
      // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
      // must be in aircraft reference frame.
      //  
      // We use the Tait Bryan angles (in terms of flight dynamics):
      // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
      //
      // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
      // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
      // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
      //
      // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
      // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
      // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

      // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
      // device is pointing up/down. (Gimbal lock)

      double qw = q0; // See issue #145 - thank you @Gord1
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      // roll (x-axis rotation)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      angle[0] = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      angle[1] = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      angle[2] = atan2(t3, t4) * 180.0 / PI;
     
      count++;
      return DMP_NO_ERR;
      
    }   // end data.header 
    else
    {
      counterr++;  
#ifdef DEBUGLEVEL1 
      PRINTs("ICM20948_readDMPdataFromFIFO error: No Header available")
      PRINT("count: ",count)
      PRINT("counterr: ",counterr)
#endif     
      // Reset FIFO
      ICM20948.ICM20948_resetFIFO();
      
      return DMP_ERR_NO_HEADER_QUAT6;
    }
  }
  counterr++; 
#ifdef DEBUGLEVEL1   
  PRINT("ICM20948_readDMPdataFromFIFO error: ",status)
  PRINT("count Read FIFO OK: ",count)
  PRINT("count Read FIFO ERR: ",counterr)
#endif       
  // Reset FIFO
  ICM20948.ICM20948_resetFIFO();    
    
  return status; 
}

void Drone3Class::Drone_get_instructions(double instruction[3], int16_t &throttle)
{  
  int16_t RC_command[NBCHANNELS];
  
  RC.RC_getCommands(RC_command);  
  
  // throttle = PPM [1100;2000]
  throttle = RC_command[THROTTLE];
  
  // roll, pitch add constrain to range [-45;+45]
  instruction[0] = constrain((double)RC_command[ROLL],-_angleLimit,_angleLimit);
  instruction[1] = constrain((double)RC_command[PITCH],-_angleLimit,_angleLimit);
      
  // yaw range [-90;+90]  
  instruction[2] = (double)RC_command[YAW];

#ifdef DEBUGLEVEL1 
  instruction[0]=rolldebug;
  rolldebug += incrementdebug;
  if ((rolldebug > _angleLimit) || (rolldebug < -_angleLimit)) incrementdebug *= -1.0;
  instruction[1]=0.0;
  instruction[2]=0.0;
  throttle=1500;
#endif  

}

uint8_t Drone3Class::Drone_init_ICM20948()
{
  uint8_t status = 0;

  PRINTs(">Start Init ICM20948")
  
#ifdef SPI_ICM20948  
  ICM20948.ICM20948_initializeSPI(SPI_SLAVESELECTED_PIN);
#endif

#ifdef I2C_ICM20948
  ICM20948.ICM20948_initializeI2C();
#endif
  
  status = ICM20948.ICM20948_startupDefault(false); // don't start Magnetometer
  if (status > 0)
  {
    PRINT("Init ICM20948 KO, startupDefault error:",status)
    return status;   
  }

  status = ICM20948.ICM20948_initializeDMP(false); // don't config Magnetometer
  if (status > 0)
  {
    PRINT("Init ICM20948 KO, initializeDMP error:",status)
    return status;  
  }

  // Enable the DMP Game Rotation Vector sensor (Quat6)
  ICM20948.ICM20948_enableDMPSensor(ICM20948_SENSOR_GAME_ROTATION_VECTOR);
  
  // Configuring DMP to output data at multiple ODRs:
  ICM20948.ICM20948_setDMPDsensorPeriod(DMP_ODR_Reg_Quat6, 0); // Set to the maximum 225 HZ.

  // Enable the FIFO
  ICM20948.ICM20948_enableFIFO(true);

  // Enable the DMP
  ICM20948.ICM20948_enableDMP(true);

  // Reset DMP
  ICM20948.ICM20948_resetDMP(); 
  
  // Reset FIFO
  ICM20948.ICM20948_resetFIFO();
  
  PRINTs("<End OK Init ICM20948")
  
  return 0;
}