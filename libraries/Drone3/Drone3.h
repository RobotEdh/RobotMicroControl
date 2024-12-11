#ifndef DRONE3_h
#define DRONE3_h

#include <ICM20948.h>    // IMU ICN20948
#include <RC.h>         // Radio Command
#include <MotorESC2.h>  // Motor ESC
#include <DS1307.h>     // RTC

#define SPI_ICM20948 
#define SPI_SLAVESELECTED_PIN  53

#define NOT_STARTED 0
#define STOPPED  1
#define RUNNING  2 
  
class Drone3Class
{
  public:
    Drone3Class();
    
    void       Drone_init(void);
    uint8_t    Drone_init_ICM20948(void);
    uint8_t    Drone_pid(double dt, double anglePID[3], int16_t &throttle, uint16_t tick, uint16_t countESC);
    void       Drone_main(void);
    uint8_t    Drone_get_angles(double angle[3]);
    void       Drone_get_instructions(double instruction[3], int16_t &throttle);

  private:
    uint8_t        _go = NOT_STARTED;
    const uint32_t _samplePeriod = 20;  // 50hz (20 ms)
    const double   _Kp[3] = {1.8 , 1.8 , 4.0};
    const double   _Ki[3] = {3.0, 3.0, 2.0};  
    const double   _Kd[3] = {0.126, 0.126, 0.0}; 
    const double   _IMax  = 5.0; // Maximum Integral value
    const double   _PIDMax = 400.0;
    const double   _lowPassFilter = 0.0079577; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  0.0079577
    const double   _angleLimit = 30.0;
};



#endif