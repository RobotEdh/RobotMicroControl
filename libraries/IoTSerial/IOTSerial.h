/*
  IOTSerial.cpp - Library for commmunicating in serial mode with IOT 
  Created by EDH, May 12, 2018.
  Released into the public domain.
  Use SoftwareSerial at 38400 bauds
*/

#ifndef IOTSERIAL_h
#define IOTSERIAL_h

#include <arduino.h>
#include <inttypes.h>

#define SUCCESS              0

#define MSG_SIZE_MAX 255
#define MAX_TAGS     63
#define MAX_PARAMS   63

#define CMD_START         0x01
#define CMD_STOP          0x02
#define CMD_INFOS         0x03
#define CMD_PICTURE       0x04
#define CMD_TURN          0x05
#define CMD_CHECK_AROUND  0x06
#define CMD_MOVE_TILT_PAN 0x07
#define CMD_GO            0x08
#define CMD_ALERT         0x09
#define CMD_CHECK         0x0A
#define CMD_PI            0x0B

// 0: alert status
// 1: picture number
// 2: motor_state
// 3: direction
// 4: distance
// 5: temperature
// 6: brightness
// 7: noise
#define ALERT_STATUS           0
#define NO_PICTURE             1
#define MOTOR_STATE            2
#define DIRECTION              3
#define DISTANCE               4
#define TEMPERATURE            5
#define BRIGHTNESS             6
#define NOISE                  7

#define RESP_SIZE 8
    
const char szField[RESP_SIZE][30]={    
"alert_status",
"no_picture",
"motor_state",
"direction",
"distance",
"temperature",
"brightness",
"noise"
};

const uint8_t CMD          = 1;
const uint8_t RESP_OK      = 2;
const uint8_t RESP_KO      = 3;
const uint8_t INFOS        = 4;
const uint8_t PICTURE      = 5;

const uint8_t TAG_CMD      = 'C';
const uint8_t TAG_CMDID    = 'I';
const uint8_t TAG_PARAM    = 'P';
const uint8_t TAG_RESP     = 'R';
const uint8_t TAG_INFOS    = 'F';
const uint8_t TAG_PICTURE  = 'U';
const uint8_t TAGSYMBOL    = '#';
const uint8_t SBN1 = 0xFA;
const uint8_t SBN2 = 0xFB;
const uint8_t EBN1 = 0xFE;
const uint8_t EBN2 = 0xFF;



class IOTSerialClass
{
	public:
		IOTSerialClass();
			
		int IOTSbegin(int snum);
        /* Description: Initialize the serial port snum                               */                                            
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* output:      none                                                          */ 
        /* lib:       	Serialx.begin                                                 */
			
        void IOTSgetTags(uint8_t *buf, uint8_t *tag, uint16_t *value, uint8_t *nbtags);
        
        int IOTSflush(int snum);
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* output:      none                                                          */ 
        /* lib:         Serialx.available                                             */
        /*              Serialx.read                                                  */
		
		int IOTSRawread(int snum);
        /* Description: Read a raw message                                            */                                            
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* output:      message                                                       */
        /*                  = none                                                    */
        /*              rlen                                                          */
        /*                  = message length received                                 */                                  
        /*              return                                                        */                            
        /*                  = int read                                                */
        /* lib:         Serialx.available                                             */
        /*              Serialx.read                                                  */  
                    		
		int IOTSread(int snum, uint8_t *msg, uint8_t *msglen);
        /* Description: Read a structued message                                      */                                            
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* output:      message                                                       */
        /*                  = message received                                        */
        /*              rlen                                                          */
        /*                  = message length received                                 */                                  
        /*              return                                                        */                            
        /*                  = -10 if error reading the serial buffer                  */
        /*                  = 0 otherwise                                             */ 
        /* lib:         Serialx.available                                             */
        /*              Serialx.read                                                  */
        
        int IOTSRawsend(int snum, uint8_t buf);        
        /* Description: Send a raw message                                            */                                            
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* input:       buf                                                           */ 
        /*                  = buf to send                                             */ 
        /* output:      none                                                          */ 
        /* lib:         Serialx.write                                                 */        
        int IOTSsend(int snum, uint8_t msgtype);                                                                     // Response short
        int IOTSsend(int snum, uint8_t msgtype,                  uint16_t *param, uint8_t paramlen, uint8_t cmdId);  // Response full
        int IOTSsend(int snum, uint8_t msgtype,                  uint16_t *param, uint8_t paramlen);                 // Infos, Picture
        int IOTSsend(int snum, uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId);  // Command
	
	private:
        void IOTSsend0(uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId);
        void IOTSsend1(uint8_t msgtype, uint8_t cmdType, uint16_t *param, uint8_t paramlen, uint8_t cmdId);
        void IOTSsend2(uint8_t msgtype, uint8_t valcmdTypeue1, uint16_t *param, uint8_t paramlen, uint8_t cmdId);       
        /* Description: Send a structured message                                     */                                            
        /* input:       snum                                                          */ 
        /*                  = Serial port number: 0, 1 or 2                           */ 
        /* input:       msgtype                                                       */ 
        /*                  = message msgtype to send                                 */ 
        /* input:       cmdType                                                       */
        /*                  = type of commande                                        */ 
        /* input:       param, paramlen                                               */
        /*                  = params                                                  */                               
        /* input:       cmdId                                                         */
        /*                  = id of the command                                       */                               
        /* output:      none                                                          */ 
        /* lib:         Serialx.write                                                 */
		
};

#endif
