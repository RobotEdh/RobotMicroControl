#ifndef IoTBlueServer_h
#define IoTBlueServer_h

#include <Arduino.h>
#include <SPI.h>
#include <BLEPeripheral.h>
#include <BLESerial.h>
#include <IOTSerial.h>

// define pins (varies per shield/board)
#define BLE_REQ   10
#define BLE_RDY   2
#define BLE_RST   9

#define LED_PIN         7  // pin 7 on nRF52832, LED is reverse
#define INTERRUPT_PIN   16

#define SUCCESS         0


class IoTBlueServerClass
{
 public:
    
  IoTBlueServerClass();
  
  int IoTBSbegin (void);
  
  int IoTBSmain(void);
  
  int IoTBSRobotCmd(String command);
  
  int IoTBSReadMsg(uint8_t *msg, uint8_t *msglen, unsigned long timeout);
  
  void IoTBSRobotReply(void);
  void IoTBSSendReply(uint8_t tag, uint16_t value);

 
 private:
   uint8_t _cmdId;
   int16_t  _result;
   uint16_t  _AlertStatus,_PictureNumber,_MotorState,_Direction,_ObstacleStatus,_Distance,_Temperature,_Humidity,_Brightness,_Noise;
   
};

#endif