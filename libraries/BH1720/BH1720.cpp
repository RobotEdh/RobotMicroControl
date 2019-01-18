#include <Arduino.h>
#include <BH1720.h>


BH1720Class::BH1720Class()
{

}

    
uint8_t BH1720Class::BH1720_init()
{
    _last_status = 0;
    _last_nb_receive = 0;
    _address = BH1720_ADDRESS_0; // pin ADDR < 0.3VCC
    
     
  Wire.beginTransmission(_address);
  Wire.write(0x10);  // set Continuously H-Resolution Mode
  
  delay (180);  // Wait to complete 1st H-resolution mode measurement.( max. 180ms. )

  _last_status = Wire.endTransmission();
  
  return _last_status;
}

// Read a 16-bit register low byte high and then low byte
uint16_t BH1720Class::BH1720_readReg16BitHL(uint8_t reg)
{
  uint16_t value;
  
  Wire.beginTransmission(_address);
  Wire.write(reg);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)2);
  if (_last_nb_receive != 2) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
    
  uint8_t msb = Wire.read(); // value high byte
  uint8_t lsb = Wire.read(); // value low byte

  value  = (((uint16_t)msb) << 8) | lsb;
  
  return value;
}


double BH1720Class::BH1720_getLux()
{
  uint16_t lux = BH1720_readReg16BitHL(_address);
  if (_last_status >0) return (double)_last_status;
  
  delay (120);  // The result of continuously measurement mode is updated.( 120ms.typ at H-resolution mode)
  
  return (double)lux/1.2; // H-Resolution Mode: 1lux resolution 120ms
}



uint8_t BH1720Class::BH1720_getStatus()
{
  return _last_status;
}

uint8_t BH1720Class::BH1720_getLast_nb_receive()
{
  return _last_nb_receive;
}

uint8_t BH1720Class::BH1720_getAddress()
{
  return _address;
}