# RobotMicroControl
Everything for a Robot on Micro Controller as Arduino &amp; ESP8266

There is one folder per device containing the souce code of the driver + a sketch to test it.

In addition:

  There is one specific folder for the main programm that should run on Arduino MEGA25620, it is named Robot.
  This folder contains several sketchs depending on the type of implementation, the more complete is robot_IoT that commnunicate with 2 ESP8266
  
  There is one specific folder for the WIFI Server programm that should run on ESP8266, it is named IoTWiFiServer.
  
  There is one specific folder for the WIFI Client programm that should run on ESP8266, it is named IoTWiFiClient.
  
  There is one specific folder to set up WIFI credentials for ESP8266, it is named ESP8266. WIFI credentials are stored in the flash memory of the ESP (so they are not displayed in Git!)
  
  
  
  NB: It is recommended to modified the size of the Arduino serial buffers SERIAL_TX_BUFFER_SIZE and SERIAL_RX_BUFFER_SIZE from 64 to 128 in order to have the same than in ESP. You can do it by updating the fie HardwareSerial.h located in the Arduino core sources directory. 
