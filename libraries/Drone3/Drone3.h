#ifndef DRONE3_h
#define DRONE3_h

#include <ICM20948.h>    // IMU ICN20948
#include <RC.h>         // Radio Command
#include <MotorESC2.h>  // Motor ESC
#include <DS1307.h>     // RTC

#define SPI_ICM20948 
#define SPI_SLAVESELECTED_PIN  53

#define LED_INIT    8
#define LED_RUNNING 9

#define NOT_STARTED 0
#define STOPPED  1
#define RUNNING  2 
  
class Drone3Class
{
  public:
    Drone3Class();
    
    uint8_t    Drone_init(void);
    uint8_t    Drone_init_ICM20948(void);
    uint8_t    Drone_compute_offsets(void);
    uint8_t    Drone_pid(double dt, double anglePID[3], int16_t &throttle, uint16_t tick);
    void       Drone_main(void);
    uint8_t    Drone_get_angles(double angle[3]);
    void       Drone_get_instructions(double instruction[3], int16_t &throttle);

  private:
    uint8_t         _go = NOT_STARTED;
    const uint32_t  _samplePeriod = 20;  // 50hz (20 ms)
    const double    _Kp[3] = {1.3 , 1.3 , 4.0};
    const double    _Ki[3] = {0.04/(double)_samplePeriod, 0.04/(double)_samplePeriod, 0.02/(double)_samplePeriod};  
    const double    _Kd[3] = {  18*(double)_samplePeriod,   18*(double)_samplePeriod,  0.0*(double)_samplePeriod};
    const uint16_t  _nbMeasures = 2000;  // 2000 measures to calibrate angles
          double    _angleOffset[3]= {0.0, 0.0, 0.0}; 
    const double    _IMax  = 5.0; // Maximum Integral value
    const double    _PIDMax = 400.0;
    const double    _lowPassFilter = 0.0079577; // Set to  "1 / ( 2 * PI * f_cut )";f_cut = 20 Hz -> _filter =  0.0079577
    const double    _angleLimit = 30.0;
    const double    _safeguard = 20.0;  // safeguard angle variation
};



#endif