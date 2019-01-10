#include <DS1307.h>
#include <Wire.h>


DS1307Class::DS1307Class(void)
{

}

uint8_t DS1307Class::DS1307_init()
{
    _address = DS1307_ADDRESS;
    
    return DS1307_isRunning(); 
}

// Check if RTC is running
uint8_t DS1307Class::DS1307_isRunning()
{
  uint8_t value;

  Wire.beginTransmission(_address);
  Wire.write(DS1307_TIME_REG);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;
    
  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)1);
  if (_last_nb_receive != 1) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
    
  value = Wire.read() & 0x80; // get CH bit : 0 : runing; 1 not running
  if (value) return ERROR_RTC_STOPPED;
  else       return 0;  
}


// To eliminate any potential race conditions,
// stop the clock before writing the values,
// then restart it after.
uint8_t DS1307Class::DS1307_adjust_current_datetime(DateTime_t *datetime)
{
  Wire.beginTransmission(_address);
  Wire.write(DS1307_TIME_REG); 
  
  Wire.write(0x80); // Stop the clock CH = 1. The seconds will be written last
  Wire.write(DS1307_decimal_to_bcd(datetime->minutes));
  Wire.write(DS1307_decimal_to_bcd(datetime->hours) & 0x3F); // Mode 24h 
  Wire.write(DS1307_decimal_to_bcd(datetime->day_of_week));
  Wire.write(DS1307_decimal_to_bcd(datetime->days));
  Wire.write(DS1307_decimal_to_bcd(datetime->months));
  Wire.write(DS1307_decimal_to_bcd(datetime->year));

  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;
    
  // Now go back and set the seconds, starting the clock back up as a side effect
  Wire.beginTransmission(_address);
  Wire.write(DS1307_TIME_REG); 
  
  Wire.write(DS1307_decimal_to_bcd(datetime->seconds) & 0x7F); // CH = 0 ennable oscillator

  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;
  else return 0;  
}


// Read time register
uint8_t DS1307Class::DS1307_read_current_datetime(DateTime_t *datetime)
{
  Wire.beginTransmission(_address);
  Wire.write(DS1307_TIME_REG);
  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;

  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)7);  // ask for 7 bytes
  if (_last_nb_receive != 7) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
  
  uint8_t raw_seconds = Wire.read();
  /* if bit 7 of second = 1 : RTC is stopped */
  if (raw_seconds & 0x80) return ERROR_RTC_STOPPED;
  
  datetime->seconds = DS1307_bcd_to_decimal(raw_seconds);
  datetime->minutes = DS1307_bcd_to_decimal(Wire.read());
  uint8_t raw_hours = Wire.read();
  if (raw_hours & 64)
  { // Format 12h
    datetime->hours = DS1307_bcd_to_decimal(raw_hours & 31);
    datetime->is_pm = raw_hours & 32;
  }
  else
  { // Format 24h
    datetime->hours = DS1307_bcd_to_decimal(raw_hours & 63);
    datetime->is_pm = 0;
  }
  datetime->day_of_week = DS1307_bcd_to_decimal(Wire.read());
  datetime->days = DS1307_bcd_to_decimal(Wire.read());
  datetime->months = DS1307_bcd_to_decimal(Wire.read());
  datetime->year = DS1307_bcd_to_decimal(Wire.read());
  
  return 0;
}

/** Fonction de lecture de la mémoire non volatile du module RTC (56 octets maximum) */
uint8_t DS1307Class::DS1307_read_nvram_memory(uint8_t address)
{ 
  /* Ne lit pas en dehors de la NVRAM */
  if (address > DS1307_NVRAM_SIZE) 
  {
      _last_status = ERROR_ADDRESS_TOO_HIGH;
      return _last_status;
  }
  /* Début de la transaction I2C */
  Wire.beginTransmission(_address);
  Wire.write(DS1307_NVRAM_BASE + address); // Lecture mémoire NVRAM

  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;
 
  /* Lit un octet depuis la mémoire du module RTC */
  _last_nb_receive = Wire.requestFrom(_address, (uint8_t)1);
  if (_last_nb_receive != 1) {_last_status=WIRE_REQUEST_ERROR; return _last_status;}
  return Wire.read();
}


/** Fonction d'écriture de la mémoire non volatile du module RTC (56 octets maximum) */
uint8_t DS1307Class::DS1307_write_nvram_memory(uint8_t address, uint8_t data)
{
  /* N'écrit pas en dehors de la NVRAM */
  if (address > DS1307_NVRAM_SIZE) 
  {
      _last_status = ERROR_ADDRESS_TOO_HIGH;
      return _last_status;
  }
  
  /* Début de la transaction I2C */
  Wire.beginTransmission(_address);
  Wire.write(DS1307_NVRAM_BASE + address); // Ecriture mémoire NVRAM
  Wire.write(data);

  _last_status = Wire.endTransmission();
  if (_last_status > 0) return _last_status;
  else return 0;  
}

uint8_t DS1307Class::DS1307_getStatus()
{
  return _last_status;
}

uint8_t DS1307Class::DS1307_getLast_nb_receive()
{
  return _last_nb_receive;
}

uint8_t DS1307Class::DS1307_getAddress()
{
  return _address;
}


/** conversion BCD -> decimal */
uint8_t DS1307Class::DS1307_bcd_to_decimal(uint8_t bcd) {
  return (bcd / 16 * 10) + (bcd % 16); 
}

/** conversion decimal -> BCD */
uint8_t DS1307Class::DS1307_decimal_to_bcd(uint8_t decimal) {
  return (decimal / 10 * 16) + (decimal % 10);
}
