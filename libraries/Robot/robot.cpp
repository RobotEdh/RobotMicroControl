#include <robot.h>         

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
#define LOGTRACE   // Enable trace
#include <log.h>
extern File logFile;                // The loging class

/* classes aleady defined in motor */
extern VL53L0XClass VL53L0Xfront;   // The ToF class for the front direction
extern LiquidCrystal_I2C lcd;       // The LCD class
extern CMPS12Class CMPS12;          // The Compass class

       DHT22Class DHT22;            // The Temperature&Humidity class      
       JPEGCameraClass JPEGCamera;  // The Camera class 
       DS1307Class DS1307;          // The RTC class       
       IOTSerialClass IOTSerial;    // The IOT serial
       I2C_ScannerClass I2C_Scanner;// used to scan I2C
       BH1720Class BH1720;          // The brightness sensor

// data updated during interrupts
volatile int IntIOT = 0; 
volatile int IntMotion = 0; 

int motor_state = STATE_STOP;
int alert_status = 0;
int no_picture = 0;                 // Picture number

int PI_activated = PI_NO_COMM; 
unsigned long PI_freqInfos = 0;  
unsigned long PI_previousTimeInfos = 0;

unsigned long freqCheck = 0;
unsigned long previousTimeCheck = 0;

unsigned long GOtimeout = 10;
 
double tab_temperature[NB_TEMPERATURE] = {0.0};
double avg_temperature = 0.0;

double tab_humidity[NB_HUMIDITY] = {0.0};
double avg_humidity = 0.0;

double tab_lux[NB_LUX] = {0.0};
double avg_lux = 0.0;

int tab_noise[NB_NOISE] = {0};
unsigned long avg_noise = 0;

char timestamp[30];

void IntrIOT()  // IOT interrupt
{
    IntIOT = 1;
    digitalWrite(Led_Blue, HIGH);  // turn on led
}

void IntrMotion()  // Motion interrupt
{
    IntMotion = 1;
    digitalWrite(Led_Red, HIGH);  // turn on led
}

int freeRam ()
{
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}


// call back for file timestamps
void dateTime(uint16_t* date, uint16_t* time) {
  uint8_t status = 0;
  DateTime_t now; 
   
  status = DS1307.DS1307_read_current_datetime(&now);
  if (status > 0)
  {
      PRINT("DS1307_read_current_datetime KO, I2C error: ",status)
  }
  else
  {
    sprintf(timestamp, "%02d:%02d:%02d %2d/%2d/%2d \n", now.hours,now.minutes,now.seconds,now.months,now.days,now.year-2000);
   // return date using FAT_DATE macro to format fields
   *date = FAT_DATE(now.year, now.months, now.days);

   // return time using FAT_TIME macro to format fields
   *time = FAT_TIME(now.hours, now.minutes, now.seconds);
  }
} 

void print_time()
{
  uint8_t status = 0;
  DateTime_t now; 
   
   status = DS1307.DS1307_read_current_datetime(&now);
   if (status > 0)
   {
      PRINT("DS1307_read_current_datetime KO, I2C error: ",status)
   }
   else
   {
      PRINTd
      PRINTt
   } 
}
 
 
void reset_leds()
{
  // turn off all leds
  digitalWrite(Led_Green, LOW);
  digitalWrite(Led_Red, LOW);
  digitalWrite(Led_Blue, LOW);    
}

void blink(int led)
{
  // turn off all leds
  reset_leds();

  // blink 2 times the resquested led during 2s
  for (int i=0;i<2;i++){
        digitalWrite(led, HIGH);  // turn on led
        delay(500);
        digitalWrite(led, LOW);  // turn off led
        delay(500);  
  }          
}

void buzz(int buzzNb)
{  
  for (int i=0;i<buzzNb;i++){
      digitalWrite(BUZZ_PIN, HIGH);
      delay(1);
      digitalWrite(BUZZ_PIN, LOW); 
      delay(200);
   }            
}

 
// Wake up ESP
void WakeUpESP()
{
    int ret = 0;
    
    // send a pulse
    digitalWrite(WAKEUP_PIN, LOW);
    delay(10);
    digitalWrite(WAKEUP_PIN, HIGH);
    PRINTs("Wake up ESP sent, wait 15s for init")
	// wait 15s for ESP init
	delay(15000);
	
	// re-initialize the IOT Serial 1 
	ret = IOTSerial.IOTSend(1); // close the IOT Serial 1 to communicate with IOT WIFClient ESP8266
    ret = IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
    ret = IOTSerial.IOTSflush(1); // flush to start clean
    PRINTs("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK")
}


int robot_begin()
{
  int ret = SUCCESS;
  int ivalue = 0;
  uint8_t status = 0;
  int nberror = 0;
  
  PRINTbegin
  
  // initialize RTC
  PRINTs(" ")
  status = DS1307.DS1307_init();
  if (status == ERROR_RTC_STOPPED)
  {
     PRINTs("RTC stopped")
     nberror++;   
  }  
  else if (status > 0)
  {   
     PRINT("Init DS1307 KO, I2C error:",status)
     nberror++;  
  }
  else
  {
     uint8_t address = DS1307.DS1307_getAddress();
     PRINTx("Init RTC DS1307 OK, address: ",address)
 
     print_time();    
  }
   
  ret = motor_begin(); 
  if (ret != SUCCESS) return ret;
  
  PRINTs("Begin Robot Init")
  PRINTs("****************")
  
  lcd.clear();
  lcd.print("Begin Robot Init");
  
  PRINTs("")
  PRINTs("Led Green")
  pinMode(Led_Green, OUTPUT);    // set the pin as output
  blink(Led_Green); 
  PRINTs("Led Red")
  pinMode(Led_Red, OUTPUT);      // set the pin as output
  blink(Led_Red);
  PRINTs("Led Blue")  
  pinMode(Led_Blue, OUTPUT);     // set the pin as output
  blink(Led_Blue);
  PRINTs("Init Leds OK")
    
  // initialize the buzzer
  PRINTs("")    
  PRINTs("Buzz")
  pinMode(BUZZ_PIN, OUTPUT); 
  buzz(3);   
  PRINTs("Init Buzzer OK")
   
  // initialize the Tilt&Pan servos 
  PRINTs("")   
  PRINTs("Move Tilt&Pan ") 
  TiltPan_begin(HSERVO_Pin, VSERVO_Pin);
  PRINTs("Init Tilt&Pan servos OK")

  // initialize the camera
  PRINTs(" ")
  PRINTs("Begin Init Camera...")
  ret=JPEGCamera.begin();
  if (ret != SUCCESS)
  {  
        PRINT("Error Init Camera, error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("Init Camera KO  ");
        nberror++;
  }        
  else
  {
        PRINTs("Init Camera OK")
        lcd.setCursor(0,1); 
        lcd.print("Init Camera OK  ");
  } 
  delay(5*1000);lcd.clear();    
  
  // initialize the SD-Card 
  PRINTs(" ") 
  if (!SD.begin(SS_CS_Pin)) {
    PRINTs("initialization SD Card KO")
  }
  else {
    PRINTs("initialization SD Card OK") 
  }
     
  ret = initSDCard();
  if (ret != SUCCESS)
  {  
        PRINT("Error Init SD-Card, error: ",ret)
        lcd.print("Init SD-Card KO ");
        nberror++;
  }                                                                    
  else
  {
        PRINTs("Init SD-Card OK")
        lcd.print("Init SD-Card OK ");
  }
    
  // get infos from SD-Card  
  ret=infoSDCard();
  if (ret < 0)
  {  
        PRINT("Error Infos SD-Card, error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("Err Infos SDCard");
  }
  else
  {
        no_picture = ret+1;
        PRINT("no_picture starts at: ",no_picture)
        lcd.setCursor(0,1); 
        lcd.print("Num picture:");lcd.print(no_picture);
  }   
  delay(5*1000);lcd.clear(); 
  
 // set date time callback function
 SdFile::dateTimeCallback(dateTime);  
    
    
  // initialize the Brightness sensor 
  PRINTs(" ")
  status = BH1720.BH1720_init(); // initialize BH1720
  if (status == 0)
  {
     uint8_t address = BH1720.BH1720_getAddress(); 
     PRINTx("Init Brightness BH1720 sensor OK, address: ",address)   
     
     double lux = BH1720.BH1720_getLux();
     status = BH1720.BH1720_getStatus();
     if (status > 0)
     {
        PRINT("Error BH1720_getLux: ",status)
     }
     else
     {
        PRINT("Lux (cloudy indoor:5-50, cloudy outdoor:50-500, sunny indoor:100-1000, sunny outdoor: >10000): ",lux)
     }
     ivalue = (int)lux;
     lcd.print("Lux:");lcd.print(ivalue);lcd.printByte(lcd_pipe);     
  }
  else 
  {           
     PRINT("Init Brightness BH1720 sensor KO, I2C error: ",status)
     nberror++;  
  } 

 
  // initialize the Sound detector 
  PRINTs(" ")
  //Sound.Sound_init(SOUND_PIN); // initialize the pin connected to the detector
  PRINTs("Init Sound Detector  OK")
  
  ivalue = 0; //TODO
  PRINT("Noise: ",ivalue)
  lcd.print("Noise:");lcd.print(ivalue);

   
  // initialize the Temperature&Humidity sensor 
  PRINTs(" ")
  DHT22.DHT22_init(DHT22_PIN);
  PRINTs("Init Temperature&Humidity sensor OK")
  DHT22_ERROR_t errorCode = DHT22.readData();
  if (errorCode == DHT_ERROR_NONE)
  {
      PRINT("Temperature (C): ",DHT22.getTemperatureC())
      PRINT("Humidity (%): ",DHT22.getHumidity()) 
  }
  else
  {
      PRINT("DHT22 KO: ", szDHT_errors[errorCode])
      nberror++;   
  }
  
  
  // set Motion interrupt
  PRINTs(" ")
  PRINTs("Set Motion interrupt")
  pinMode(MOTION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), IntrMotion, RISING);         // set Motion interrupt

  
  // initialize the IOT Serial 1 
  PRINTs(" ")
  ret = IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
  ret = IOTSerial.IOTSflush(1); // flush to start clean
  PRINTs("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK")
  
  // Send sleep to IOT WIFClient 
  PRINTs("Call IOTSsend 1 SLEEP")
  ret = IOTSerial.IOTSsend (1, SLEEP);
  PRINT("Call IOTSsend, ret: ",ret) 
  
  // sets the digital pin WAKEUP_PIN as output and keep high level
  pinMode(WAKEUP_PIN, OUTPUT);          
  digitalWrite(WAKEUP_PIN, HIGH);
  
  // initialize the IOT Serial 2, interrupt setting
  PRINTs(" ")
  ret = IOTSerial.IOTSbegin(2); // initialize the IOT Serial 2 to communicate with IOT Bluetooth server
  ret = IOTSerial.IOTSflush(2); // flush to start clean
  PRINTs("Init IOT Serial 2 to communicate with IOT Bluetooth server OK")
  pinMode(IOT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IOT_PIN), IntrIOT, RISING);         // set IOT interrupt
   
  interrupts(); // enable all interrupts
  PRINT("Init Interrupts OK, IntIOT: ",IntIOT)
  PRINT("Init Interrupts OK, IntMotion: ",IntMotion)
  
  // Check I2C
  PRINTs(" ")
  PRINTs("Check I2C")
  I2C_Scanner.I2C_Scanner_init(); 
  I2C_Scanner.I2C_Scanner_scan(); 
  
  lcd.setCursor(0,1); 
  lcd.print("End   Robot Init");
  delay(5*1000);lcd.clear();  
 
  PRINTs("End Robot Init ")
  if (!nberror)PRINTs("OK")
  else         PRINT("KO nb errors: ",nberror)
  PRINTs("*****************") 
  print_time();  
   
  PRINTflush
  
  return nberror;
  
} 


int infos (uint16_t *resp, uint8_t *resplen)
{    
     uint8_t status = 0;
     
     print_time();
     
     // alert status
     resp[ALERT_STATUS] = (uint16_t)alert_status;
     PRINT("alert status: ",resp[ALERT_STATUS])
     
     // picture number
     resp[NO_PICTURE] = (uint16_t)no_picture;
     PRINT("no_picture: ",resp[NO_PICTURE])
     
     // motor_state
     resp[MOTOR_STATE] = (uint16_t)motor_state;
     PRINT("motor_state: ",resp[MOTOR_STATE])
     
     // direction
     resp[DIRECTION] = (uint16_t)CMPS12.CMPS12_getCompassHighResolution();
     PRINT("direction: ",resp[DIRECTION])
     
     // obstacle_status
     resp[OBSTACLE_STATUS] = (uint16_t)check_around();
     PRINT("obstacle_status: ",resp[OBSTACLE_STATUS])
      
     // distance
     uint16_t distance = VL53L0Xfront.VL53L0X_readMillimeters(); 
     if (distance > 0) resp[DISTANCE] = distance; // in mm
     else              resp[DISTANCE] = 0;
     PRINT("distance front: ",resp[DISTANCE])
     
     // temperature&humidity
     DHT22_ERROR_t errorCode = DHT22.readData();
     if (errorCode == DHT_ERROR_NONE)
     {
         resp[TEMPERATURE] = (uint16_t)(100.0 * DHT22.getTemperatureC());
         PRINT("temperature: ",resp[TEMPERATURE])
         resp[HUMIDITY]    = (uint16_t)(100.0 * DHT22.getHumidity());
         PRINT("humidity: ",resp[HUMIDITY])  
     }
     else
     {
        PRINT("DHT22.readData KO, error: ",szDHT_errors[errorCode])
        resp[TEMPERATURE] = 0;
        resp[HUMIDITY]    = 0;
     }     
     
     // brightness
     double lux = BH1720.BH1720_getLux();
     status = BH1720.BH1720_getStatus();
     if (status > 0)
     {
        PRINT("BH1720_getStatus KO, error: ",status)
        resp[BRIGHTNESS] = 0;
     }
     else
     {
        resp[BRIGHTNESS] = (uint16_t)lux;
        PRINT("brightness: ",resp[BRIGHTNESS])
     }
 
     // noise
     resp[NOISE] =  0;  //TODO
     PRINT("noise: ",resp[NOISE])
     

     *resplen = RESP_SIZE;
 
     return SUCCESS;
}     


int check ()
{
  
  int i = 0;
  uint8_t status = 0;
  
  print_time();

  // Check Motion
  if (IntMotion == 1) {
       blink(Led_Red);   
       PRINTs("Alert Motion")
       IntMotion = 0; //reset alert	
       return ALERT_MOTION;
  }
  else PRINTs("No Motion")
  
  // Check Noise
  int noise = 0; //TODO
  PRINT("noise: ",noise)
  PRINT("previous_noise: ",tab_noise[0])
  PRINT("avg_noise: ",avg_noise)
  PRINT("MAX_VAR_NOISE: ",MAX_VAR_NOISE)
  if (noise != tab_noise[0]) {
      if (MAX_VAR_NOISE < abs(avg_noise - noise) && tab_noise[NB_NOISE-1] != 0) {
          return ALERT_NOISE;
      }    
      avg_noise = 0;
      for (i=NB_NOISE-1;i>0;i--) { 
          tab_noise[i] = tab_noise[i-1];
          avg_noise += tab_noise[i];
      }
      tab_noise[0]=noise;
      avg_noise = (avg_noise+noise)/NB_NOISE;
  }
   
  // Check Temperature & Humidity  Variation
  DHT22_ERROR_t errorCode = DHT22.readData();
  if (errorCode == DHT_ERROR_NONE)
  {
      double temperature = DHT22.getTemperatureC();
      double humidity    = DHT22.getHumidity();
      PRINT("temperature: ",temperature)
      PRINT("previous_temperature: ",tab_temperature[0])
      PRINT("avg_temperature: ",avg_temperature)
      PRINT("MAX_VAR_TEMPERATURE: ",MAX_VAR_TEMPERATURE)
      PRINT("humidity: ",humidity)
      PRINT("previous_humidity: ",tab_humidity[0])
      PRINT("avg_humidity: ",avg_humidity)
      PRINT("MAX_VAR_HUMIDITY: ",MAX_VAR_HUMIDITY)      
  
      if (temperature != tab_temperature[0]) {
         if (MAX_VAR_TEMPERATURE < abs(avg_temperature - temperature) && tab_temperature[NB_TEMPERATURE-1] != 0.0) {
             return ALERT_TEMPERATURE;
         }    
         avg_temperature = 0.0;
         for (i=NB_TEMPERATURE-1;i>0;i--) { 
             tab_temperature[i] = tab_temperature[i-1];
             avg_temperature += tab_temperature[i];
         }
         tab_temperature[0]=temperature;
         avg_temperature = (avg_temperature+temperature)/NB_TEMPERATURE;
     }
     if (humidity != tab_humidity[0]) {
         if (MAX_VAR_HUMIDITY < abs(avg_humidity - humidity) && tab_humidity[NB_HUMIDITY-1] != 0.0) {
             return ALERT_HUMIDITY;
         }    
         avg_humidity = 0.0;
         for (i=NB_HUMIDITY-1;i>0;i--) { 
             tab_humidity[i] = tab_humidity[i-1];
             avg_humidity += tab_humidity[i];
         }
         tab_humidity[0]=humidity;
         avg_humidity = (avg_humidity+humidity)/NB_HUMIDITY;
     }
  }
  else
  {
        PRINT("DHT22.readData KO, error: ",szDHT_errors[errorCode])
  }
         
  // Check Lux Variation
  
  double lux = BH1720.BH1720_getLux();
  status = BH1720.BH1720_getStatus();
  if (status > 0)
  {
     PRINT("BH1720_getStatus KO, error: ",status)
  }
  else
  {
     PRINT("lux: ",lux)
     PRINT("previous_lux: ",tab_lux[0])
     PRINT("avg_lux: ",avg_lux)
     PRINT("MAX_VAR_LUX: ",MAX_VAR_LUX)     
     if (lux != tab_lux[0]) {
        if (MAX_VAR_LUX < abs(avg_lux - lux) && tab_lux[NB_LUX-1] != 0) {
           return ALERT_LUX;
        }    
        avg_lux = 0;
        for (i=NB_LUX-1;i>0;i--) { 
           tab_lux[i] = tab_lux[i-1];
           avg_lux += tab_lux[i];
        }
        tab_lux[0]=lux;
        avg_lux = (avg_lux+lux)/NB_LUX;
     }
  }
      
  return NO_ALERT;              
}        

   

int robot_command (uint16_t cmd[], uint16_t resp[], uint8_t *resplen)
{    
 
 int HPos = 90;
 int VPos = 90;
 unsigned long start = 0;
 uint8_t infolen = 0;
 int checkdir = SUCCESS;
 int motor_state_save = -1;
 int error = -1;
 int ret = SUCCESS;

 lcd.clear();     // clear LCD
 reset_leds();    // turn off all leds
 digitalWrite(Led_Green, HIGH);  // turn on led green

 PRINTx("Begin robot_command, command: ",cmd[0])
 switch ((int)cmd[0]) {
 
 case CMD_STOP:
     PRINTs("CMD_STOP")
     lcd.print("STOP"); 
     
     stop();
     motor_state = STATE_STOP;
     resp[0] = STATE_STOP;
     *resplen = 0+1;
     break; 
  
 case CMD_START:
     PRINTs("CMD_START")
     lcd.print("START"); 
           
     start_forward();        
     motor_state = STATE_GO;
     resp[0] = STATE_GO;
     *resplen = 0+1;
     break; 
 
 case CMD_CHECK_AROUND:
     PRINTs("CMD_CHECK_AROUND")
     lcd.print("CHECK AROUND");
    
     checkdir = check_around();
     
     lcd.setCursor(0,1); 
     if      (checkdir == DIRECTION_MID_LEFT)   lcd.print("MID LEFT");
     else if (checkdir == DIRECTION_MID_RIGHT)  lcd.print("MID RIGHT");
     else if (checkdir == DIRECTION_LEFT)       lcd.print("LEFT");
     else if (checkdir == DIRECTION_RIGHT)      lcd.print("RIGHT");
     else if (checkdir == OBSTACLE_LEFT)        lcd.print("OBSTACLE LEFT");
     else if (checkdir == OBSTACLE_RIGHT)       lcd.print("OBSTACLE RIGHT");
     else if (checkdir == OBSTACLE_LEFT_RIGHT)  lcd.print("OBSTACLE LEFT RIGHT");
     else                                       lcd.print("?");

     resp[0] = (uint16_t)checkdir;
     *resplen = 0+1;
     break;
     
 case CMD_MOVE_TILT_PAN:    
     PRINTs("CMD_MOVE_TILT_PAN")
     if (cmd[2] == 0) HPos = (int)cmd[1] + 90; else HPos = 90 - (int)cmd[1]; 
     if (cmd[4] == 0) VPos = (int)cmd[3] + 90; else VPos = 90 - (int)cmd[3]; 
     PRINT("HPos: ",HPos)
     PRINT("VPos: ",VPos)
     lcd.print("MOVE TILT&PAN");
     lcd.setCursor(0,1); 
     lcd.print("X: ");
     lcd.print(HPos);
     lcd.print(" Y: ");
     lcd.print(VPos);
     
     TiltPan_move(HPos, VPos);
    
     *resplen = 0;
     break; 
                    
 case CMD_TURN:
     PRINTs("CMD_TURN")
     PRINT("angle (deg): ",cmd[1])
     PRINT("angle (sign): ",(cmd[2] != 1) ? ('+'):('-'))
     if (cmd[1] == 180)
     {       
           PRINTs("CMD_TURN_BACK")
           lcd.print("TURN BACK ");  
         
           ret = turnback (10);  // 10s max
           if (ret != SUCCESS){
           	  PRINT("turnback KO, error: ",ret)
           	  lcd.setCursor(0,1); 
           	  lcd.print("turnback error: "); lcd.print(ret);
           	  error = 1;
           }
     }       
     else if (motor_state == STATE_GO)
     { 
           lcd.print("TURN"); lcd.print((cmd[2] != 1) ? ('+'):('-')); lcd.print((int)cmd[1]);lcd.printByte(223); //degree   
         
           ret = turn ((double)((cmd[2] != 1) ? (cmd[1]):(-cmd[1])), 5);  // 5s max        
           if (ret != SUCCESS){
           	  PRINT("turn KO, error: ",ret)
           	  lcd.setCursor(0,1); 
           	  lcd.print(" turn error: "); lcd.print(ret);
           	  error = 1;
           }           
    }
    
    *resplen = 0;
    break;        
     
     
 case CMD_GET_INFOS: 
     PRINTs("CMD_GET_INFOS")

     ret = infos (resp, &infolen);
       
     if (resp[MOTOR_STATE] == STATE_GO) {
         lcd.print("RUNING");
     }    
     else
     {
         lcd.print("STOPPED");
     }
     lcd.setCursor(0,1);   
     lcd.print(resp[TEMPERATURE]); lcd.print((byte)lcd_celcius);lcd.write(lcd_pipe);   
     lcd.print(resp[DISTANCE]); lcd.print("cm");lcd.write(lcd_pipe); 
     lcd.print(resp[DIRECTION]); lcd.printByte(223); //degree    
 
     *resplen = infolen;
     break;
      
 case CMD_PICTURE: 
     PRINTs("CMD_PICTURE")
     no_picture++;
     PRINT("no picture: ",no_picture)
     lcd.print("PICTURE ");
     
     motor_state_save = motor_state;
     if (motor_state == STATE_GO) {
        PRINTs("Stop") 
        stop();
        motor_state = STATE_STOP;
      }
   
     ret = JPEGCamera.makePicture (no_picture);
     if (ret == SUCCESS)
     { 
        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        PRINT("makePicture error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
       
     if (motor_state_save == STATE_GO) {          
        PRINTs("Start")
        start_forward();                     
        motor_state = STATE_GO;
     }
        
     // byte 0: picture number
     resp[0] = no_picture;
     *resplen = 0+1;    
     break;

 case CMD_ALERT: 
     PRINTs("CMD_ALERT")
     lcd.print("Alert"); 
    
     blink(Led_Red);  
     buzz(5); 
     
     // If motor_state == STATE_GO => Stop           
     if (motor_state == STATE_GO) {
        PRINTs("Stop")
        stop();
        motor_state = STATE_STOP;
     }
     
     // get infos
     ret = infos (resp, &infolen);

     // Make 3 pictures left, front and right
     if ((HPos != 90) || (VPos !=90))
     { 
        HPos = 90;
        VPos = 90;
        TiltPan_move(HPos, VPos);
     }
     
     no_picture++;
     PRINT("makePicture, no picture: ",no_picture)
     lcd.print("PICTURE ");
     
     ret = JPEGCamera.makePicture (no_picture);
     if (ret == SUCCESS)
     { 
        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        PRINT("makePicture KO, error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
             
     if (ret == SUCCESS)
     { 
        HPos = 0;
        VPos = 90;
        TiltPan_move(HPos, VPos);

        no_picture++;
        PRINT("makePicture, no picture: ",no_picture)
        lcd.print("PICTURE ");
        
        ret = JPEGCamera.makePicture (no_picture);

        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        PRINT("makePicture KO, error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
      
     if (ret == SUCCESS)
     { 
        HPos = 180;
        VPos = 90;
        TiltPan_move(HPos, VPos);
     
        no_picture++;
        PRINT("makePicture, no picture: ",no_picture)
        lcd.print("PICTURE ");
        
        ret = JPEGCamera.makePicture (no_picture);

        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        PRINT("makePicture KO, error: ",ret)
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }

     resp[NO_PICTURE] = (uint16_t)no_picture;
     *resplen = infolen;
            
     HPos = 90;
     VPos = 90;
     TiltPan_move(HPos, VPos);
              
     break; 
     
 case CMD_CHECK: 
     PRINT("CMD_CHECK - freq (sec): ",cmd[1])
     lcd.print("Check"); ;lcd.print((int)cmd[1]);
     
     freqCheck = (unsigned long)cmd[1] * 1000;
     
     if (freqCheck > 0) alert_status = check();
     
     if (alert_status != 0) {
           PRINT("Alert detected: ",alert_status)
           lcd.setCursor(0,1); 
           lcd.print("Alert: "); lcd.print(alert_status);                
     }
     else
     {
           PRINTs("No alert detected: ")
           lcd.setCursor(0,1); 
           lcd.print(" No Alert");               
     } 
  
     // byte 0: alert
     resp[0] = alert_status;
     *resplen = 0+1;
     break; 
  
 case CMD_GO: 
 case CMD_RUN: 
     PRINT("CMD_GO/RUN, timeout (sec): ",cmd[1])
     lcd.print("GO/RUN ");lcd.print((int)cmd[1]);lcd.print("secs");
     
     GOtimeout = (unsigned long)cmd[1]; 
     
     if (GOtimeout == 0) {    
        PRINTs("stop")
        stop();
        motor_state = STATE_STOP;
     }
     else if (motor_state != STATE_GO)
     {  
           PRINTs("start_forward")
           start_forward();
           motor_state = STATE_GO;
     }
     
     error = -1;
     start = millis();
     while((millis() - start < GOtimeout*1000UL) && (error == -1)) {
          ret = go(GOtimeout);  
     
          if ((ret != SUCCESS) && (ret != OBSTACLE) && (ret != OBSTACLE_LEFT) && (ret != OBSTACLE_RIGHT))
          {
              stop();
              motor_state = STATE_STOP;
     	      error = 1;
     	                   
              PRINT("CMD_GO/RUN, error: ",ret)
              PRINTs("Stop")              
              lcd.setCursor(0,1); 
              lcd.print("error: "); lcd.print(ret);                

          }
          else if ((ret == OBSTACLE) || (ret == OBSTACLE_LEFT) || (ret == OBSTACLE_RIGHT))
          {
              stop();
              motor_state = STATE_STOP;
                 
              PRINT("CMD_GO/RUN, Obstacle: ",ret)
              PRINTs("Stop")
              lcd.setCursor(0,1); 
              if (ret == OBSTACLE_LEFT)        lcd.print("OBSTACLE LEFT");
              else if (ret == OBSTACLE_RIGHT)  lcd.print("OBSTACLE RIGHT");
              else if (ret == OBSTACLE)        lcd.print("OBSTACLE");
              else                             lcd.print("?");
                              
              ret = SUCCESS;            
              checkdir = check_around();
         
              PRINT("check_around, direction: ",checkdir)
              lcd.clear();
              lcd.print("check around");
              lcd.setCursor(0,1); 
              if      (checkdir == DIRECTION_MID_LEFT)   lcd.print("MID LEFT");
              else if (checkdir == DIRECTION_MID_RIGHT)  lcd.print("MID RIGHT");
              else if (checkdir == DIRECTION_LEFT)       lcd.print("LEFT");
              else if (checkdir == DIRECTION_RIGHT)      lcd.print("RIGHT");
              else if (checkdir == OBSTACLE_LEFT)        lcd.print("OBSTACLE LEFT");
              else if (checkdir == OBSTACLE_RIGHT)       lcd.print("OBSTACLE RIGHT");
              else if (checkdir == OBSTACLE_LEFT_RIGHT)  lcd.print("OBSTACLE LEFT RIGHT");
              else                                       lcd.print("?");
         
              if ((checkdir == DIRECTION_MID_LEFT) || (checkdir == DIRECTION_LEFT) || (checkdir == DIRECTION_MID_RIGHT) || (checkdir == DIRECTION_RIGHT)) {
                   start_forward();
                   motor_state = STATE_GO;
                   
                   double alpha = 0.0;
                   if      (checkdir == DIRECTION_MID_LEFT)   alpha = -30.0;
                   else if (checkdir == DIRECTION_MID_RIGHT)  alpha =  30.0;
                   else if (checkdir == DIRECTION_LEFT)       alpha = -45.0;
                   else if (checkdir == DIRECTION_RIGHT)      alpha =  45.0;
                       
                   PRINT("Turn, angle: ",alpha)
                   ret = turn (alpha,  5); // turn during 5s max
                   if (ret != SUCCESS)
                   {
                      stop();
                      motor_state = STATE_STOP;                   	  
                   	  error = 1;
                   	                     	  
                   	  PRINT("Turn KO, error: ",ret)
                      PRINTs("Stop")                                         	  
                   	  lcd.clear();                   	  
                   	  lcd.print("turn");
                   	  lcd.setCursor(0,1);
                   	  lcd.print("error: "); lcd.print(ret); 
                   }
                   else
                   {
                      lcd.clear();                   	  
                   	  lcd.print("turn OK"); 
                   	  PRINT("Turn OK, angle: ",alpha)               	
                   }
              }
              else 
              {
              	   buzz(2);
              	   motor_state = STATE_GO;
              	   PRINTs("Turnback 10 ms")
              	   ret = turnback (10); // turn back during 10s max
                   if (ret != SUCCESS)
                   {
                      stop();
                      motor_state = STATE_STOP;
                      error = 1; 
                      
                      PRINT("Turnback KO, error: ",ret)
                   	  PRINTs("Stop")
                   	  lcd.clear();                   	  
                   	  lcd.print("turnback");
                   	  lcd.setCursor(0,1);
                   	  lcd.print("error: "); lcd.print(ret);                    	                                           	  
                   }
                   else
                   {
                      lcd.clear();                   	  
                   	  lcd.print("turnback OK"); 
                   	  PRINTs("Turnback OK");                  	
                   }                   
              }                 
          }
          else
          {
              	   PRINTs("GO OK")
              	   lcd.clear();                   	  
                   lcd.print("GO OK");
                   error = 0;
          }           
     } // end while (millis() - start < GOtimeout*1000UL) && (error == -1)
     
     if ((int)cmd[0] == CMD_GO) {
        ret = infos (resp, &infolen);
              
        if (error == 0) {
           if (resp[MOTOR_STATE] == STATE_GO) lcd.print("RUNNING");
           else                               lcd.print("STOPPED");
           lcd.setCursor(0,1);   
           lcd.print(resp[TEMPERATURE]); lcd.print((byte)lcd_celcius);lcd.write(lcd_pipe);   
           lcd.print(resp[DISTANCE]); lcd.print("cm");lcd.write(lcd_pipe);
           lcd.print(resp[DIRECTION]); lcd.printByte(223); //degree   
        }  
        *resplen = infolen; 
     }
     else
     { 
        *resplen = 0;       
     }
                 
     break;
 
 case CMD_PI: 
     PRINTs("CMD_PI")
     PRINT("PI_activated: ",cmd[1])
     PRINT("PI freq (s): ",cmd[2])
     lcd.print("PI activated ");lcd.print((int)cmd[1]);
 
     PI_activated = (int)cmd[1];
     if (PI_activated == PI_ALERT_INFOS) {
         PI_freqInfos = (unsigned long)cmd[2]* 1000UL;
         PRINT("PI_freqInfos (ms): ",PI_freqInfos)
     }
       
    *resplen = 0;     
     break;
 
 case CMD_TEST: 
     PRINTs("CMD_TEST")
     
     // reply each value incremented
     resp[ALERT_STATUS] = cmd[1]+1;
     PRINT("alert status: ",resp[ALERT_STATUS])
     
     resp[NO_PICTURE] = cmd[2]+1;
     PRINT("no_picture: ",resp[NO_PICTURE])
     
     resp[MOTOR_STATE] = cmd[3]+1;
     PRINT("motor_state: ",resp[MOTOR_STATE])
     
     resp[DIRECTION] = cmd[4]+1;
     PRINT("direction: ",resp[DIRECTION])

     resp[OBSTACLE_STATUS] = cmd[5]+1;
     PRINT("obstacle_status: ",resp[OBSTACLE_STATUS])

     resp[DISTANCE] = cmd[6]+1;
     PRINT("distance: ",resp[DISTANCE])

     resp[TEMPERATURE] = cmd[7]+1;
     PRINT("temperature: ",resp[TEMPERATURE])
     
     resp[HUMIDITY] = cmd[8]+1;
     PRINT("humidity: ",resp[HUMIDITY]); 

     resp[BRIGHTNESS] = cmd[9]+1;
     PRINT("brightness: ",resp[BRIGHTNESS])
     
     resp[NOISE] = cmd[10]+1;
     PRINT("noise: ",resp[NOISE])


     lcd.setCursor(0,1);   
     lcd.print("TEST");lcd.write(lcd_pipe); 
 
     *resplen = RESP_SIZE;
     break;          
 
 default:
    PRINTs("invalid command")
    lcd.print("invalid command");
    
    *resplen = 0;      
    break;                     
 
 } //end switch
    
    
 if (error == 1) {
    blink(Led_Red);
    blink(Led_Red);
    buzz(7);
    *resplen = 0; 
 }
                        
 reset_leds();    // turn off all leds
 return ret;
}
       
void robot_main ()
{    
 uint16_t mcmd[CMD_SIZE] = {0};
 uint16_t mresp[RESP_SIZE] = {0};
 uint8_t  mresplen = 0;

 unsigned long currentTimeCheck;
 unsigned long PI_currentTimeInfos;
 
 uint8_t n_pict = 0;
 int mem0 = 0;
 int mem = 0;
 int ret = SUCCESS;
 
 mem0 = freeRam();
 PRINT("Free RAM: ",mem0)
  
 while (1) {  // No stop
       
       //check memory
       mem = freeRam();
       if (mem0 < mem){ PRINT("Memory leak from: ",mem0) PRINT(" to: ",mem) }
        
       currentTimeCheck = millis();
       if ( ((currentTimeCheck > previousTimeCheck + freqCheck) || (IntMotion == 1)) && (freqCheck > 0)) {  // check every freqCheck or if motion alert
             previousTimeCheck = currentTimeCheck; 
             PRINT("IntMotion: ",IntMotion)              

             mcmd[0] = (uint16_t)CMD_CHECK;
             mcmd[1] = (uint16_t)freqCheck/1000;
             PRINT("Call robot_command CHECK witk freqCheck (sec): ",mcmd[1])
             ret = robot_command (mcmd, mresp, &mresplen);
 
             PRINT("Call robot_command, ret: ",ret)	
             alert_status = (int)mresp[0];  
             PRINT("Alert: ",alert_status)
             PRINTflush
       }
       
       PI_currentTimeInfos = millis();  
       if (alert_status > 0)
       {
             PRINT("Alert trigerred: ",alert_status)	// robot has detected something...
             PRINTs("***************")
             PRINTs("")
                     
             PRINTs("Call robot_command ALERT");
  	         mcmd[0] = (uint16_t)CMD_ALERT;
             ret = robot_command (mcmd, mresp, &mresplen);  
             PRINT("Call robot_command, ret: ",ret)	
             n_pict = (uint8_t)(mresp[NO_PICTURE]); // Last Picture number
             PRINT("n_pict: ",n_pict)
                           
             // Send Infos + last 3 pictures in WIFI to the PI server if it is activated
             if ((PI_activated == PI_ALERT_ONLY)|| (PI_activated == PI_ALERT_INFOS)) {
                
                WakeUpESP();  // wake up ESP
                
                mresp[NO_PICTURE] = 0; // No picture send
                //Send the Infos message first time quickly
                PRINTs("Call IOTSsend 1 INFOS")
                ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                PRINT("Call IOTSsend, ret: ",ret) 
                                          
                //Send the 3 last pictures
                PRINTs("Call robot_Send_Picture 3 times")
                if (n_pict > 3) ret = robot_Send_Picture(n_pict-2);                 
                if (n_pict > 2) ret = robot_Send_Picture(n_pict-1);                   
                if (n_pict > 1) ret = robot_Send_Picture(n_pict); 
                
                //Send the Infos message again to attach pictures
                mresp[NO_PICTURE] = n_pict;
                PRINTs("Call IOTSsend 1 INFOS")
                ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                PRINT("Call IOTSsend, ret: ",ret)	   
                
                PRINTs("Call IOTSsend 1 SLEEP")
                ret = IOTSerial.IOTSsend (1, SLEEP);
                PRINT("Call IOTSsend, ret: ",ret)        
                           
                alert_status = 0;
             }
             else
             {
                PRINTs("PI communication not activated") 
                alert_status = 0;   
             } 
             PRINTflush                        
       }
       else if (IntIOT > 0) { // IOT wants to do something...
             PRINTs("Request received from IOT, call robot_IOT")	
             PRINTs("*****************************************")
             PRINTs("")
             IntIOT = 0;
 
             ret = robot_IOT(); 
             if (ret != SUCCESS) {
                   PRINT("Call robot_IOT KO, error: ",ret)
                   ret = IOTSerial.IOTSflush(2);
             }
             else
             {
                   PRINTs("robot_IOT OK") 
             }
             PRINTflush 

       }
       else if (PI_activated == PI_ALERT_INFOS)
       {
             if ((PI_freqInfos > 0) && (PI_currentTimeInfos > PI_previousTimeInfos + PI_freqInfos)) {
                   PI_previousTimeInfos = PI_currentTimeInfos;      

                   mcmd[0] = (uint16_t)CMD_GET_INFOS;
                   PRINTs("Call robot_command GET_INFOS")
                   ret = robot_command (mcmd, mresp, &mresplen);
                   PRINT("Call robot_command, ret: ",ret)	
                   
                   WakeUpESP();  // wake up ESP
                   
                   PRINTs("Call IOTSsend 1 INFOS")
                   ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                   PRINT("Call IOTSsend, ret: ",ret) 
                   
                   PRINTs("Call IOTSsend 1 SLEEP")
                   ret = IOTSerial.IOTSsend (1, SLEEP);
                   PRINT("Call IOTSsend, ret: ",ret) 

                   PRINTflush
             }
        
       }   
       else if (motor_state == STATE_GO)
       {       
             mcmd[0] = (uint16_t)CMD_RUN;
             mcmd[1] = (uint16_t)GOtimeout;
             mcmd[2] = (uint16_t)0;   // na             
             PRINT("Call command GO, timout (sec): ",mcmd[1])
             ret = robot_command (mcmd, mresp, &mresplen);
             PRINT("Call robot_command, ret: ",ret)	
             PRINTflush             
       }         
 } // end while 
 
   
}

int robot_IOT ()
{    
 uint8_t msg[MSG_SIZE_MAX];
 uint8_t msg_len = 0;
 uint8_t tag [MAX_TAGS];
 uint16_t value [MAX_TAGS];
 uint8_t nbtags;
 uint16_t cmd[CMD_SIZE];
 uint16_t resp[RESP_SIZE];
 uint8_t resplen = 0;
 uint8_t cmdId = 0;

 int ret = SUCCESS;
 
 PRINTs("Start robot_IOT")

 PRINTs("Call IOTSread 2")
 //Read the message received from IOT
 ret = IOTSerial.IOTSread(2, msg, &msg_len, 60000UL);  // timeout 60s
 PRINT("Call IOTSread 2, ret: ",ret)
 PRINT("msg_len: ",msg_len)

 if (ret != SUCCESS) {
       PRINTs("Call IOTSsend 2 with RESP_KO")  
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -1;
 }
   
 IOTSerial.IOTSgetTags(msg, tag, value, &nbtags);   
 
 PRINT("nbtags: ",nbtags)
 if (nbtags < 1) {
       PRINTs("nbtags < 1, Call IOTSsend 2 with RESP_KO")    
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -2;
 }
 
 PRINTx("tag[0]: ",tag[0])
 if (tag[0] != TAG_CMDID) {
       PRINTs("tag[0] != TAG_CMDID, Call IOTSsend 2 with RESP_KO")      
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -3;
 }

 cmdId = value[0];
 PRINT("cmdId: ",cmdId)
 
 PRINT("tag[1]: ",tag[1]) 
 if (tag[1] == TAG_CMD)
 {
       PRINTs("Tag CMD")
       if (nbtags > MAX_TAGS) {
               PRINTs("nbtags > MAX_TAGS< 1, Call IOTSsend 2 with RESP_KO")    
               ret = IOTSerial.IOTSsend(2, RESP_KO);
               reset_leds();
               return -4;
       }     
             
       for (uint8_t j=0; j<(nbtags-1); j++)
       {
           cmd[j] = value[j+1];
           PRINTj("cmd",j,cmd[j])
       }
 
       // Execute the command received from IOT
       PRINTs("Call robot_command")
       ret = robot_command (cmd, resp, &resplen);  
       PRINT("Call robot_command, ret: ",ret)

       if (ret == SUCCESS) { 
        	 PRINTs("Call IOTSsend 2 with RESP_OK")
        	 PRINT("cmdId",cmdId)    
        	 for (uint8_t k=0; k<resplen; k++)
             {
                PRINTj("resp",k,resp[k])
             }
             ret = IOTSerial.IOTSsend(2, RESP_OK, resp, resplen, cmdId);
       }
       else
       {
        	 PRINTs("Call IOTSsend 2 with RESP_KO")    
             ret = IOTSerial.IOTSsend(2, RESP_KO);
             reset_leds();
             return -5;
       }        
 }
 else {
       PRINTs("no Tag CMD, Call IOTSsend 2 with RESP_KO")    
       IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -6;
 }
   
 reset_leds();   
 PRINTs("End OK robot_IOT") 
 return SUCCESS;                     
}			 



int robot_Send_Picture (uint8_t n)
{    
 uint16_t param[MAX_PARAMS];
 uint8_t paramlen = 0;
 uint8_t msg[MSG_SIZE_MAX];
 uint8_t msg_len = 0;
 uint8_t tag [MAX_TAGS];
 uint16_t value [MAX_TAGS];
 uint8_t nbtags;
 
 char filename[12+1];
 uint16_t nbytes = 0;
 uint8_t buf[PAYLOAD_SIZE];
 
 int ret = SUCCESS;
 
 PRINT("Start robot_Send_Picture, n: ",n)
 
 // Open picture file
 sprintf(filename, "PICT%d.jpg", n);  
 File FilePicture = SD.open(filename, FILE_READ);
 if (!FilePicture) {PRINTs("FilePicture open KO "); return FILE_OPEN_ERROR;}
 else PRINTs("FilePicture open OK ")
 
 //Send the Picture message 
 param[0] = (uint16_t)n;
 param[1] = (uint16_t)FilePicture.size();
 paramlen = 2;

 ret = IOTSerial.IOTSsend(1, PICTURE, param, paramlen); 
 
 // CANNOT use TX with Sparkfunk Thing but CAN with Adafruit HUZZAH 
       
 //Read the message replied to be sure that the client is ready to receive the picture
 ret = IOTSerial.IOTSread(1, msg, &msg_len, 60000UL);  // timeout 60s
 PRINT("Call IOTSread 1, ret: ", ret);
 PRINT("msg_len: ",msg_len)

 if (ret != SUCCESS) {
     PRINTs("error IOTSread")  
     ret = IOTSerial.IOTSflush(1);
     return 0;
 }
 
 //Decode the message
 IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
 PRINT("Call IOTSgetTags, nbtags: ",nbtags)
    
 if (nbtags < 1)          return -1; 
    
 PRINTx("tag[0]: ",tag[0]) 
 if (tag[0]!= TAG_CMDID)   return -2;
 PRINTx("tag[1]: ",tag[1])   
 if (tag[1]!= TAG_RESP)   return -3;
 PRINTx("value[1]: ",value[1])         
 if (value[1] == RESP_KO) return -4;
 if (value[1] != RESP_OK) return -5;

 // RESP_OK received, we are good.
 // read from the file until there's nothing else in it:
 while ((nbytes = FilePicture.read(buf, sizeof(buf))) > 0 && ret == SUCCESS) {
       for (uint16_t i = 0;i<nbytes;i++)
       {
         ret = IOTSerial.IOTSRawsend(1, buf [i]); 
       }
 
 }// while 
  
 //Close file
 FilePicture.close();
 
 //Read the message that indicate that the client has send the picture
 ret = IOTSerial.IOTSread(1, msg, &msg_len, 60000UL);  // timeout 60s
 PRINT("Call IOTSread 1, ret: ", ret);
 PRINT("msg_len: ",msg_len)

 if (ret != SUCCESS) {
     PRINTs("error IOTSread")  
     ret = IOTSerial.IOTSflush(1);
     return 0;
 }
 
 //Decode the message
 IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
 PRINT("Call IOTSgetTags, nbtags: ",nbtags)
    
 if (nbtags < 1)          return -1; 
    
 PRINTx("tag[0]: ",tag[0]) 
 if (tag[0]!= TAG_CMDID)   return -2;
 PRINTx("tag[1]: ",tag[1])   
 if (tag[1]!= TAG_RESP)   return -3;
 PRINTx("value[1]: ",value[1])         
 if (value[1] == RESP_KO) return -4;
 if (value[1] != RESP_OK) return -5;

 // RESP_OK received, we are good.
    
 PRINTs("End OK robot_Send_Picture") 
 return SUCCESS;                     
}			 