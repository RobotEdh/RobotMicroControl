#ifndef DRONE2_h
#define DRONE2_h

#include <MPU6050.h>    // MPU6050
#include <RC.h>         // Radio Command
#include <MotorESC2.h>  // Motor ESC
#include <DS1307.h>     // RTC


#define PIDLOGFREQ 1 //record every 1 ticks ie 20 ms at 50Hz
  
class Drone2Class
{
  public:
    Drone2Class();
    
    void       Drone_init(void);
    uint8_t    Drone_pid(void);
    void       Drone_main(void);

  private:
    const uint32_t samplePeriod = 20;  // 20 ms (50Hz)
    const double _Kp[3] = {1.8 , 1.8 , 4.0};
    const double _Ki[3] = {0.03, 0.03, 0.02};  // values for 50hz (20 ms)
    const double _Kd[3] = {12.6, 12.6, 0.0}; // values for 50hz (20 ms)
    const double _IMax  = 5.0; // Maximum Integral value
    const double _PIDMax = 400.0;
    const double _filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  7.9577e-3
    const double coeff_stabilisation = 3.0;
};

#endif