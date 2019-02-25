#ifndef ROBOT_h
#define ROBOT_h

#include <Arduino.h> 
#include <Wire.h>       // I2C protocol

#include <motor.h>
#include <VL53L0X.h>           // TOF
#include <CMPS12.h>            // Compas
#include <DHT22.h>             // Temperature& Humidity
#include <Servo.h>             // Servo
#include <TiltPan.h>           // Tilt&Pan
#include <LSY201.h>            // Camera
#include <LiquidCrystal_I2C.h> // LCD
#include <Motion.h>            // Motion
#include <sdcard.h>            // SD Card
#include <SD.h> 
#include <DS1307.h>            // RTC
#include <IOTSerial.h>         // Serial lib to communicate with IOT
#include <I2C_Scanner.h>       // used to scan I2C
#include <BH1720.h>            // Brightness sensor

#define PI_NO_COMM        0
#define PI_ALERT_ONLY     1
#define PI_ALERT_INFOS    2

#define PAYLOAD_SIZE 80

#define NO_ALERT          0
#define ALERT_MOTION      1
#define ALERT_LUX         2
#define ALERT_TEMPERATURE 3
#define ALERT_HUMIDITY    4
#define ALERT_NOISE       5

#define NB_LUX 6
#define MAX_VAR_LUX 100
#define NB_TEMPERATURE 3
#define MAX_VAR_TEMPERATURE 1.5
#define NB_HUMIDITY 3
#define MAX_VAR_HUMIDITY 10
#define NB_NOISE 6
#define MAX_VAR_NOISE 200

#define Led_Green    22  
#define Led_Red      23  
#define Led_Blue     24 
#define BUZZ_PIN     25          
#define DHT22_PIN    38   // Pin used by the Temperature&Humidity sensor pin 38


#define MOTION_PIN    2   // Pin used by the Motion connected to interrupt INT0 on MEGA2560
void IntrMotion();        // interrupt handler Motion

#define IOT_PIN       3   // Pin used by the IOT connected to interrupt INT1 on MEGA2560
void IntrIOT();           // interrupt handler IOT

#define WAKEUP_PIN  36   // Pin used by the MEGA2560 to wake-up the ESp

int freeRam (void);
void WakeUpESP(void); 
void dateTime(uint16_t* date, uint16_t* time);

void print_time(void );
/* Description: print date & time                                             */                                            
/* input:       none                                                          */
/* output:      none                                                          */
/* lib:         DS1307                                                        */

void blink(int led);
/* Description: blink a led                                                   */                                            
/* input:       led                                                           */
/* output:      none                                                          */
/* lib:         digitalWrite                                                  */
/*              delay                                                         */ 


void buzz(int buzzNb);
/* Description: buzz                                                          */                                            
/* input:       buzzNb                                                        */
/*              Number of times buzzing                                       */
/* output:      none                                                          */
/* lib:         digitalWrite                                                  */
/*              delay                                                         */

int check();
/* Description: check                                                         */                                            
/* input:       none                                                          */
/* output:      alert                                                         */
/*                                                                            */

int infos(uint16_t *resp, uint8_t *resplen);
/* Description: get infos                                                     */                                            
/* input:       none                                                          */
/* output:      resp                                                          */
/*                  = response                                                */
/*              resp_len                                                      */
/*                  = response lenght                                         */
/*              return                                                        */
/*                  = SUCCESS is no error otherwise error code                */                               
/* lib:                                                                       */
/*              get_SpeedMotorRight                                           */
/*              get_SpeedMotorLeft                                            */
/*              CMPS03_read                                                   */
/*              GP2Y0A21YK_getDistanceCentimeter                              */

int robot_begin();     
/* Description: initialize everything, must be called during setup            */                                            
/* input:       none                                                          */
/* output:      return                                                        */                             
/*                  = SUCCESS always even if error during initialization      */                                
/* lib:         JPEGCamera.begin                                              */ 
/*              pinMode                                                       */
/*              digitalWrite                                                  */

int robot_command (uint16_t cmd[], uint16_t resp[], uint8_t *resplen);
/* Description: command the robot                                             */                                            
/* input:       cmd                                                           */
/*                  = command and the related parameters                      */
/* output:      resp                                                          */
/*                  = response                                                */
/*              presp_len                                                     */
/*                  = response lenght                                         */
/*              return                                                        */
/*                  = SUCCESS is no error otherwise error code                */                               
/* lib:         stop                                                          */ 
/*              start_forward                                                 */                            
/*              start_forward_test                                            */ 
/*              check_around                                                  */
/*              TiltPan_move                                                  */
/*              turn                                                          */
/*              get_SpeedMotorRight                                           */
/*              get_SpeedMotorLeft                                            */
/*              CMPS03_read                                                   */
/*              GP2Y0A21YK_getDistanceCentimeter                              */
/*              makePicture                                                   */   
/*              go                                                            */  


int robot_IOT(void);    
int robot_Send_Picture (uint8_t n);
void robot_main(void); 
#endif
