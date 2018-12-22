
#ifndef I2C_Scanner_h
#define I2C_Scanner_h

#include <Arduino.h> // used for pin definition

#include <Wire.h>


class I2C_ScannerClass
{
  public:
    
    I2C_ScannerClass ();
    
    void I2C_Scanner_init();
    void I2C_Scanner_scan();

  private:

};

#endif