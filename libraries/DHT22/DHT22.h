#ifndef _DHT22_H_
#define _DHT22_H_

#include <Arduino.h> // used for pin definition
#include <inttypes.h> // used for uint8_t type


// This should be 40, but the sensor is adding an extra bit at the start
#define DHT22_DATA_BIT_COUNT 41

#define DHT22_ERROR_VALUE -995


typedef enum
{
  DHT_ERROR_NONE = 0,
  DHT_BUS_HUNG,
  DHT_ERROR_NOT_PRESENT,
  DHT_ERROR_ACK_TOO_LONG,
  DHT_ERROR_SYNC_TIMEOUT,
  DHT_ERROR_DATA_TIMEOUT,
  DHT_ERROR_CHECKSUM,
  DHT_ERROR_TOOQUICK
} DHT22_ERROR_t;

const char szDHT_errors[8][20]={    
"No error",
"BUS Hung",
"Not Present",
"ACK time out",
"Sync Timeout",
"Data Timeout",
"Checksum error",
"Polled to quick"
};

class DHT22Class
{
  public:
    DHT22Class();
    
    void DHT22_init(uint8_t pin);
    DHT22_ERROR_t readData();
	short int getHumidityInt();
	short int getTemperatureCInt();
    double getHumidity();
    double getTemperatureC();
    double getTemperatureF();
  
   private:
    uint8_t _bitmask;
    volatile uint8_t *_baseReg;
    unsigned long _lastReadTime;
    short int _lastHumidity;
    short int _lastTemperature;   
    
};


#endif /*_DHT22_H_*/
