#include <MPU6050.h>
#include <RC.h>
#include <MotorESC.h>
#include <Drone.h>
   
MPU6050Class  MPU6050;     
RCClass       RC; 
MotorESCClass MotorESC;  



DroneClass::DroneClass(void) {
}

void DroneClass::Drone_init() {
    
  Serial.begin(9600); // initialize serial 
  
  Serial.println(">Start Init");

  MPU6050.MPU6050_init();

  MPU6050.MPU6050_calibrate();
  double temperature = MPU6050.MPU6050_getTemperature();  
  Serial.print("temperature: "); Serial.println(temperature); 
  
  RC.RC_init();
  
  MotorESC.MotorESC_init();
  
  Serial.println("<End OK Init");
}


void DroneClass::Drone_main() {
  
  uint32_t currentTime ;
  static uint32_t rcTime = 0;
 
  int16_t RC_command[NBCHANNELS];
  double angle[3];
  int16_t ESC_command[4];
    
  int16_t RC_commandRP[2]; 
  double anglePID[2];
  int16_t error       = 0;
  int16_t delta_error = 0;
  static int16_t last_error[2] = {0,0};
  static int32_t sum_error[2]  = {0,0};
  const char szAngles[2][20]={"Roll","Pitch"};
  
  currentTime = micros(); 
  if ((currentTime > rcTime )|| (rcTime  == 0)) { // 50Hz: PPM frequency of the RC, no change happen within 20ms except first time
    rcTime = currentTime + 20000;
    
    RC.RC_getCommands(RC_command); // int16_t range [-180;+180]          for ROLL, PITCH, YAW
                                   //               0 or [MINPPM;MAXPPM] for THROTTLE
                                   //               0/1                  for AUX1, AUX2
  }
  
  //**** Read IMU ****   
  MPU6050.MPU6050_get_roll_pitch_yaw(angle);  // double range [-180;+180] ROLL, PITCH, YAW

   
  // ROLL & PITCH
  RC_commandRP[0]=RC_command[ROLL];
  RC_commandRP[1]=RC_command[PITCH];
  for(int i=0;i<2;i++) {	
    error =  RC_commandRP[i] - (int16_t)(angle[i]);
    sum_error[i] += (int32_t) error; 
    delta_error = (error - last_error[i]);
    last_error[i] = error;
    
    anglePID[i] = (Kp[i]*(double)error) + (Ki[i]*(double)sum_error[i]) + (Kd[i]*(double)delta_error);
   
    Serial.print("angle[");Serial.print(szAngles[i]);Serial.print("]:");Serial.println(angle[i]);
    Serial.print("rcCommand[");Serial.print(szAngles[i]);Serial.print("]:");Serial.println(RC_commandRP[i]);
    Serial.print("sum_error[");Serial.print(szAngles[i]);Serial.print("]:");Serial.println(sum_error[i]);
    Serial.print("error:");Serial.println(error);
    Serial.print("anglePID[");Serial.print(szAngles[i]);Serial.print("]:");Serial.println(anglePID[i]);
  }

  ESC_command[THROTTLE] = RC_command[THROTTLE];
  ESC_command[ROLL]     = (int16_t)anglePID[0];
  ESC_command[PITCH]    = (int16_t)anglePID[1];
  ESC_command[YAW]      = 0; 
  MotorESC.MotorESC_RunMotors(ESC_command);
}
