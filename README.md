
 Robot & Motor
 -------------
 The main Robot programs for Arduino.
 There are several sketchs dedicated for testing.
 
 
 Drone & RC & MotorESC
 ----------------------
 The main Drone program for Arduino including the management of the Radio Command (RC) and the Motor controller (Motor ESC)
 This program uses other several lib for MPU, PWM, logging, SDCard.
 There are several sketchs dedicated for testing.
 The latest version is Drone3, using MPU ICM20948 via SPI with DMP activating,  it runs on MEGA2560 and ESP32 with freeRTOS managing tasks on the 2 cores.
 
 
 IoTBlueServer
 -------------
 The main program to commnunicate with an Mobile Android via Bluetooth.
  
   
 IoTWiFiServer
 -------------
 The main program for ESP8266 to commnunicate with a PC Windows via WIFI.
 There is a sketch dedicated for testing.


 IoTWiFiClient
 -------------
 The main program for ESP8266 to send data via WIFI to the Raspberry PI.
 There is a sketch that handles the deep sleep feature (recommanded for low consumption)
 
 
 IoTSerial
 ---------
 This lib handles the Serial protocol and transmission.
  
 
 BLEPeripheral
 -------------
 This lib handles the BLE bluetooth.
 See the dedicated README.md.
 

 ESP8266
--------
 This lib handles the WIFI with ESP8266.
 The examples show how to store and retreive the credential from the Flash memory of the ESP8266
  
 
 BH1720
 ------
 This lib handles the Ambient Light sensor BH1720 via I2C.
 The example shows how to get the Brightness.


 CMPS12
 -------
 This lib handles the compass CMPS12 via I2C.
 One example shows how to diplay the magnometer figures and all the other accel&gyro&temperature data, the other examples show how to calibrate/erase calibrate.


 DHT22
 -----
 This lib handles the Temperature&Humidiy sensor DHT22.
 The example shows how to get the Temperature and the Humidity.
 
 
 DS1307
 ------
 This lib handles the Real Time Clock (RTC) DS1307 via I2C interface.
 One example shows how to adjust the real date and time and a second example shows how to get them.
 

 ICM20948
 --------
 This lib handles the MPU ICM20948 via I2C or SPI interface.
 The DMP and the FIFO are activated

 
 LSY201
 ------
 This lib handles the JPE camera LSY201 via Serial3 interface at 38400 bauds.
 The example shows how to do a picture and store it on a SD Card.
 
 
 SharpIR
 -------
 This lib handles the SharpIR sensors via ADC, including the management of sample.
 The median of a sample of 25 measures is provided as the distance.
 The example shows how to get a distance with the Sharp IR GP2Y0A21Y
 
 
 VL53L0X
 -------
 This lib handles the Time Of Flight sensor VL53L0X via I2C, including the change of address and the management of sample.
 By default the config is MEDIUM_ACCURACY (set timing budget to 33 ms).
 The median of a sample of 5 measures is provided as the distance, so in (5*33 = 165 ms).
 The example shows how to get the distance from 3 VL53L0X by changing the address of each of them.
 

 MyLog
 -----
 This lib manages the logging.
 3 modes are available:
   1. no logging
   2. log Serial of log Serial1
   3. log SD card 
 Macro are defined to hlep logging, inclmuding RTC timestamps and traces.
 

 PWM
 -----
 This lib manages the PWM based on Hardware (Timer interrupt), not Software.
 This is a good alternative (faster and less CPU busy time) to the standard Arduino Servo lib.
 

 TiltPan
 -------
 This lib handles the move of a TiltPan by 2 servo motors via PWM.
 The example shows how to move a TiltPan.
 
 
 Motion
 ------
 This lib handles the infrared motion sensor via digital pin.
 The example shows how to detect motion.
 
 
 LiquidCrystal_I2C
-----------------
 This lib handles the LCD LiquidCrystal via I2C.
 The examples shows how to diplay texts and customs characters on the LCD.
 
 
 SDCard
-------
 This lib handles the SD-Card via SPI.
 
 
 I2C_Scanner
 -----------
 This is a very standard and helpfull tools checking all the devices connected on I2C.
 
 
 Other lib
----------
 + CMPS03: this is the Compass. Not used because I prefer CMPS12 or ICM20948.
 + TEMT6000: this is the Ambient Light sensor. Not used because I prefer BH1720.
 + TMP102: this is the Temperature sensor. Not used because I prefer DHT22.
 + LIS3MDL: this is a lib to study a three-axis magnetic sensor but it didn't worked as exepected.
 + MPU6050: this is a lib to study the MPU-6050 sensor Not used because I prefer CMPS12.
 + Micro: this is a lib to manage an electret Micro but it didn't worked as exepected.
 + Sound: this is a lib to manage a Sound Detector but it didn't worked as exepected.
