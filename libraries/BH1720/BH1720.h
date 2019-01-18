#ifndef BH1720_h
#define BH1720_h

#include <Arduino.h>
#include <Wire.h>

#define BH1720_ADDRESS_0   0x23 // pin ADDR < 0.3VCC
#define BH1720_ADDRESS_1   0x5C // pin ADDR > 0.7VCC

#define WIRE_TRANSMIT_SUCESS          0x00 // Wire.endTransmission()- 0:success
#define WIRE_ERROR_TRANSMIT_TOOLONG   0x01 // Wire.endTransmission()- 1:data too long to fit in transmit buffer
#define WIRE_ERROR_TRANSMIT_ADR_NACK  0x02 // Wire.endTransmission()- 2:received NACK on transmit of address
#define WIRE_ERROR_TRANSMIT_DATA_NACK 0x03 // Wire.endTransmission()- 3:received NACK on transmit of data
#define WIRE_TRANSMIT_ERROR_OTHER     0x04 // Wire.endTransmission()- 4:other error
#define WIRE_REQUEST_ERROR            0x80 // Wire.requestFrom()- the number of bytes returned from the slave device != the number of bytes to request

class BH1720Class
{
  public:


    BH1720Class();

    uint8_t BH1720_init(void);
    uint16_t BH1720_readReg16BitHL(uint8_t reg);
    
    double  BH1720_getLux(void);

    uint8_t BH1720_getStatus(void);
    uint8_t BH1720_getLast_nb_receive(void);
    uint8_t BH1720_getAddress(void);
    
  private:
    uint8_t _address;
    uint8_t _last_status;
    uint8_t _last_nb_receive;     
};

#endif
