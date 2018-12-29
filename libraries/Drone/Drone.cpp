#include <MPU6050.h>
#include <RC.h>
#include <MotorESC.h>
#include <Drone.h>
   
MPU6050Class  MPU6050;     
RCClass       RC; 
MotorESCClass MotorESC;  

uint32_t currentTime = 0;
uint32_t previousTime = 0;
uint32_t cycleTime = 0;

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

  int16_t RC_command[NBCHANNELS];
  double angle[2];
  double axisPID[3];
  

  static int16_t last_error[3] = {0,0,0};
  static int32_t sum_error[3] = {0,0,0};

  
  int16_t error = 0;
  int16_t delta_error = 0;
  static uint32_t rcTime  = 0;
  const char* sz_axis[] = {"ROLL","PITCH","YAW","THROTTLE"};
  
  
  if ((currentTime > rcTime )|| (rcTime  == 0)) { // 50Hz: PPM frequency of the RC, no change happen within 20ms except first time
    rcTime = currentTime + 20000;
    
    RC.RC_getCommands(RC_command);
  }
  currentTime = micros();
  cycleTime = currentTime - previousTime;
  previousTime = currentTime ;
  
  //**** Read IMU ****   
  MPU6050.MPU6050_get_roll_pitch_yaw(angle);
   
  // ROLL & PITCH
  for(int i=0;i<2;i++) {
  	
    error =  RC_command[i] - (int16_t)(angle[i]*159.0); // convert c_angle from -pi;+pi to -500;+500
    sum_error[i] += (int32_t) (error * cycleTime);
    
    delta_error = (error - last_error[i])/cycleTime;
    delta_error = ((int32_t) delta_error * ((uint16_t)0xFFFF / (cycleTime>>4)))>>6;
    
    axisPID[i] =  (Kp[i]*error) + (Ki[i]*sum_error[i]) + (Kd[i]*delta_error);
    
    last_error[i] = error;
  
    Serial.print(">MultiWii_loop: c_angle[");Serial.print(sz_axis[i]);Serial.print("]:");Serial.println(angle[i]);
    Serial.print(">MultiWii_loop: rcCommand[");Serial.print(sz_axis[i]);Serial.print("]:");Serial.println(RC_command[i]);
    Serial.print(">MultiWii_loop: sum_error[");Serial.print(sz_axis[i]);Serial.print("]:");Serial.println(sum_error[i]);
    Serial.print(">MultiWii_loop: error:");Serial.println(error);
    Serial.print(">MultiWii_loop: axisPID[");Serial.print(sz_axis[i]);Serial.print("]:");Serial.println(axisPID[i]);
     
  }

  //YAW
  axisPID[YAW] = 0;
  Serial.print(">MultiWii_loop: axisPID[");Serial.print((int)YAW);Serial.print("]:");Serial.println(axisPID[YAW]);

  MotorESC.MotorESC_RunMotors(axisPID[ROLL],axisPID[PITCH],axisPID[YAW],RC_command[THROTTLE]);
}
