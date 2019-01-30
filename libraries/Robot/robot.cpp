#include <robot.h>         

/* classes aleady defined in motor */
extern VL53L0XClass VL53L0X;        // The ToF class
extern LiquidCrystal_I2C lcd;       // The LCD class
extern CMPS12Class CMPS12;          // The Compass class

       DHT22Class DHT22;            // The Temperature&Humidity class      
       JPEGCameraClass JPEGCamera;  // The Camera class 
       DS1307Class DS1307;          // The RTC class       
       IOTSerialClass IOTSerial;    // The IOT serial
       I2C_ScannerClass I2C_Scanner;// used to scan I2C
       BH1720Class BH1720;          // The brightness sensor

// SD variables
extern SdFile root;          // SD Root
extern SdFile FilePicture;   // SD File

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

// Wake up ESP
void WakeUpESP()
{
    int ret = 0;
    
    // send a pulse
    digitalWrite(WAKEUP_PIN, LOW);
    delay(10);
    digitalWrite(WAKEUP_PIN, HIGH);
    Serial.println ("Wake up ESP sent");
	// wait 15s for ESP init
	delay(15000);
	
	// re-initialize the IOT Serial 1 
	ret = IOTSerial.IOTSend(1); // close the IOT Serial 1 to communicate with IOT WIFClient ESP8266
    ret = IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
    ret = IOTSerial.IOTSflush(1); // flush to start clean
    Serial.println ("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK");
}


void print_time()
{
  uint8_t status = 0;
  DateTime_t now; 
   
   status = DS1307.DS1307_read_current_datetime(&now);
   if (status > 0)
   {
      Serial.print("DS1307_read_current_datetime KO, I2C error: ");Serial.println(status);
   }
   else
   {
      Serial.print("Date: ");Serial.print(now.days);Serial.print("/");Serial.print(now.months);Serial.print("/");Serial.println(now.year + 2000);
      Serial.print("Time: ");Serial.print(now.hours);Serial.print(":");Serial.print(now.minutes);Serial.print(":");Serial.println(now.seconds);
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

  // blink the resquested led
  for (int i=0;i<3;i++){
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

int robot_begin()
{
  int ret = SUCCESS;
  int ivalue = 0;
  uint8_t status = 0;
  int nberror = 0;
  
  ret = motor_begin(); 
  if (ret != SUCCESS) return ret;

  Serial.println(F("Begin Robot Init"));
  Serial.println(F("****************"));
  
  lcd.clear();
  lcd.print("Begin Robot Init");
  
  Serial.println(F(""));
  Serial.println(F("Led Green"));
  pinMode(Led_Green, OUTPUT);    // set the pin as output
  blink(Led_Green); 
  Serial.println(F("Led Red"));
  pinMode(Led_Red, OUTPUT);      // set the pin as output
  blink(Led_Red);
  Serial.println(F("Led Blue"));  
  pinMode(Led_Blue, OUTPUT);     // set the pin as output
  blink(Led_Blue);
  Serial.println(F("Init Leds OK"));
    
  // initialize the buzzer
  Serial.println(F(""));    
  Serial.println(F("Buzz"));
  pinMode(BUZZ_PIN, OUTPUT); 
  buzz(3);   
  Serial.println(F("Init Buzzer OK"));
   
  // initialize the Tilt&Pan servos 
  Serial.println(F(""));    
  Serial.println(F("Move Tilt&Pan ")); 
  TiltPan_begin(HSERVO_Pin, VSERVO_Pin);
  Serial.println(F("Init Tilt&Pan servos OK"));

  // initialize the camera
  Serial.println(F(" "));
  Serial.println(F("Begin Init Camera..."));
  ret=JPEGCamera.begin();
  if (ret != SUCCESS)
  {  
        Serial.print(F("Error Init Camera, error: "));
        Serial.println(ret);
        lcd.setCursor(0,1); 
        lcd.print("Init Camera KO  ");
        nberror++;
  }        
  else
  {
        Serial.println(F("Init Camera OK"));
        lcd.setCursor(0,1); 
        lcd.print("Init Camera OK  ");
  } 
  delay(5*1000);lcd.clear();    
  
  // initialize the SD-Card 
  Serial.println(F(" "));    
  ret = initSDCard();
  if (ret != SUCCESS)
  {  
        Serial.print(F("Error Init SD-Card, error: "));
        Serial.println(ret);
        lcd.print("Init SD-Card KO ");
        nberror++;
  }                                                                    
  else
  {
        Serial.println(F("Init SD-Card OK"));
        lcd.print("Init SD-Card OK ");
  }
    
  // get infos from SD-Card  
  ret=infoSDCard();
  if (ret < 0)
  {  
        Serial.print(F("Error Infos SD-Card, error: "));
        Serial.println(ret);
        lcd.setCursor(0,1); 
        lcd.print("Err Infos SDCard");
  }
  else
  {
        no_picture = ret+1;
        Serial.print(F("no_picture starts at: "));
        Serial.println(no_picture);
        lcd.setCursor(0,1); 
        lcd.print("Num picture:");lcd.print(no_picture);
  }   
  delay(5*1000);lcd.clear();  
    
    
  // initialize the Brightness sensor 
  Serial.println(F(" "));  
  status = BH1720.BH1720_init(); // initialize BH1720
  if (status == 0)
  {
     Serial.print(F("Init Brightness BH1720 sensor OK, address: 0x"));
     uint8_t address = BH1720.BH1720_getAddress();
     Serial.println(address,HEX);   
     
     double lux = BH1720.BH1720_getLux();
     status = BH1720.BH1720_getStatus();
     if (status > 0)
     {
        Serial.print(F("Error BH1720_getLux: "));Serial.println(status);
     }
     else
     {
        Serial.print(lux);Serial.println(F(" Lux (cloudy indoor:5-50, cloudy outdoor:50-500, sunny indoor:100-1000, sunny outdoor: >10000)"));
     }
     ivalue = (int)lux;
     lcd.print("Lux:");lcd.print(ivalue);lcd.printByte(lcd_pipe);     
  }
  else 
  {           
     Serial.print(F("Init Brightness BH1720 sensor KO, I2C error: "));Serial.println(status);
     nberror++;  
  } 

 
  // initialize the Sound detector 
  Serial.println(F(" "));
  //Sound.Sound_init(SOUND_PIN); // initialize the pin connected to the detector
  Serial.println(F("Init Sound Detector  OK"));
  
  ivalue = 0; //TODO
  //Serial.print("Value between 0 (no noise) and 1023 (huge noise): ");
  Serial.println(ivalue); 
  lcd.print("Noise:");lcd.print(ivalue);

   
  // initialize the Temperature&Humidity sensor 
  Serial.println(" ");
  DHT22.DHT22_init(DHT22_PIN);
  Serial.println(F("Init Temperature&Humidity sensor OK"));
  DHT22_ERROR_t errorCode = DHT22.readData();
  if (errorCode == DHT_ERROR_NONE)
  {
      Serial.print(DHT22.getTemperatureC());
      Serial.print(F("C "));
      Serial.print(DHT22.getHumidity());
      Serial.println(F("%"));  
  }
  else
  {
      Serial.println( szDHT_errors[errorCode]);
      nberror++;   
  }
  
  
  // set Motion interrupt
  Serial.println(F(" "));
  Serial.println(F("Set Motion interrupt"));
  pinMode(MOTION_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(MOTION_PIN), IntrMotion, RISING);         // set Motion interrupt

  
  // initialize RTC
  Serial.println(F(" "));
  status = DS1307.DS1307_init();
  if (status == ERROR_RTC_STOPPED)
  {
     Serial.println(F("RTC stopped"));
     nberror++;   
  }  
  else if (status > 0)
  {   
     Serial.print(F("Init DS1307 KO, I2C error:")); Serial.println(status); 
     nberror++;  
  }
  else
  {
     uint8_t address = DS1307.DS1307_getAddress();
     Serial.print(F("Init RTC DS1307 OK, address: 0x")); Serial.println(address,HEX);
 
     print_time();    
  }
  
  
  // initialize the IOT Serial 1 
  Serial.println(" ");
  ret = IOTSerial.IOTSbegin(1); // initialize the IOT Serial 1 to communicate with IOT WIFClient ESP8266
  ret = IOTSerial.IOTSflush(1); // flush to start clean
  Serial.println (F("Init IOT Serial 1 to communicate with IOT WIFClient ESP8266 OK"));
  
  // initialize the IOT Serial 2, interrupt setting
  Serial.println(" ");
  ret = IOTSerial.IOTSbegin(2); // initialize the IOT Serial 2 to communicate with IOT WIFServer ESP8266
  ret = IOTSerial.IOTSflush(2); // flush to start clean
  Serial.println (F("Init IOT Serial 2 to communicate with IOT WIFServer ESP8266 OK"));
  pinMode(IOT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IOT_PIN), IntrIOT, RISING);         // set IOT interrupt
 
  // sets the digital pin WAKEUP_PIN as output and keep high level
  pinMode(WAKEUP_PIN, OUTPUT);          
  digitalWrite(WAKEUP_PIN, HIGH);
      
  interrupts(); // enable all interrupts
  Serial.print(F("Init Interrupts OK, IntIOT: "));    Serial.println(IntIOT);
  Serial.print(F("Init Interrupts OK, IntMotion: ")); Serial.println(IntMotion);
  
  // Check I2C
  Serial.println(F(" "));
  Serial.print(F("Check I2C")); 
  I2C_Scanner.I2C_Scanner_init(); 
  I2C_Scanner.I2C_Scanner_scan(); 
  
  lcd.setCursor(0,1); 
  lcd.print("End   Robot Init");
  delay(5*1000);lcd.clear();  
 
  Serial.print(F("End Robot Init "));
  if (!nberror) Serial.println(F("OK"));
  else {Serial.print(F("KO nb errors: "));Serial.println(nberror);}
  Serial.println(F("*****************"));
  Serial.println(F(" "));
 
  return nberror;
  
} 


int infos (uint16_t *resp, uint8_t *resplen)
{    
     uint8_t status = 0;
     
     print_time();
     
     // alert status
     resp[ALERT_STATUS] = (uint16_t)alert_status;
     Serial.print("alert status: ");Serial.println(resp[ALERT_STATUS]);
     
     // picture number
     resp[NO_PICTURE] = (uint16_t)no_picture;
     Serial.print("no_picture: ");Serial.println(resp[NO_PICTURE]);
     
     // motor_state
     resp[MOTOR_STATE] = (uint16_t)motor_state;
     Serial.print("motor_state: ");Serial.println(resp[MOTOR_STATE]);
     
     // direction
     resp[DIRECTION] = (uint16_t)CMPS12.CMPS12_getCompassHighResolution();
     Serial.print("direction: ");Serial.println(resp[DIRECTION]);
     
     // obstacle_status
     resp[OBSTACLE_STATUS] = (uint16_t)check_around();
     Serial.print("obstacle_status: ");Serial.println(resp[OBSTACLE_STATUS]);
      
     // distance
     uint16_t distance = VL53L0X.VL53L0X_readMillimeters(); 
     if (distance > 0) resp[DISTANCE] = distance; // in mm
     else              resp[DISTANCE] = 0;
     Serial.print("distance: ");Serial.println(resp[DISTANCE]);
     
     // temperature&humidity
     DHT22_ERROR_t errorCode = DHT22.readData();
     if (errorCode == DHT_ERROR_NONE)
     {
         resp[TEMPERATURE] = (uint16_t)(100.0 * DHT22.getTemperatureC());
         Serial.print("temperature: ");Serial.println(resp[TEMPERATURE]);
         resp[HUMIDITY]    = (uint16_t)(100.0 * DHT22.getHumidity());
         Serial.print("humidity: ");Serial.println(resp[HUMIDITY]);  
     }
     else
     {
        Serial.print("Error DHT22 readData: ");Serial.println( szDHT_errors[errorCode]);
        resp[TEMPERATURE] = 0;
        resp[HUMIDITY]    = 0;
     }     
     
     // brightness
     double lux = BH1720.BH1720_getLux();
     status = BH1720.BH1720_getStatus();
     if (status > 0)
     {
        Serial.print("Error BH1720_getLux: ");Serial.println(status);
        resp[BRIGHTNESS] = 0;
     }
     else
     {
        resp[BRIGHTNESS] = (uint16_t)lux;
        Serial.print("brightness: ");Serial.println(resp[BRIGHTNESS]);
     }
 
     // noise
     resp[NOISE] =  0;  //TODO
     Serial.print("noise: ");Serial.println(resp[NOISE]);
     

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
       Serial.print("Alert Motion");
       IntMotion = 0; //reset alert	
       return ALERT_MOTION;
  }
  else Serial.println("No Motion");
  
  // Check Noise
  int noise = 0; //TODO
  Serial.print("noise: ");Serial.print(noise);Serial.print(" -- previous_noise: ");Serial.print(tab_noise[0]);Serial.print(" -- avg_noise: ");Serial.print(avg_noise);Serial.print(" -- MAX_VAR_NOISE: "); Serial.println(MAX_VAR_NOISE);
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
      Serial.print("temperature: ");Serial.print(temperature);Serial.print(" -- previous_temperature: ");Serial.print(tab_temperature[0]);Serial.print(" -- avg_temperature: ");Serial.print(avg_temperature);Serial.print(" -- MAX_VAR_TEMPERATURE: ");Serial.println(MAX_VAR_TEMPERATURE);
      Serial.print("humidity: ");   Serial.print(humidity);   Serial.print(" -- previous_humidity: ");   Serial.print(tab_humidity[0]);   Serial.print(" -- avg_humidity: ");   Serial.print(avg_humidity);   Serial.print(" -- MAX_VAR_HUMIDITY: ");      Serial.println(MAX_VAR_HUMIDITY);
  
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
        Serial.print("Error DHT22: " );Serial.println( szDHT_errors[errorCode]);
  }
         
  // Check Lux Variation
  
  double lux = BH1720.BH1720_getLux();
  status = BH1720.BH1720_getStatus();
  if (status > 0)
  {
     Serial.print("Error BH1720_getLux: ");Serial.println(status);
  }
  else
  {
     Serial.print("lux: ");Serial.print(lux);Serial.print(" -- previous_lux: ");Serial.print(tab_lux[0]);Serial.print(" -- avg_lux: ");Serial.print(avg_lux);Serial.print(" -- MAX_VAR_LUX: ");Serial.println(MAX_VAR_LUX);
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

 Serial.print("Start robot_command start, command: 0x"); Serial.println(cmd[0],HEX);
 switch ((int)cmd[0]) {
 
 case CMD_STOP:
     Serial.println("CMD_STOP");
     lcd.print("STOP"); 
     
     stop();
     motor_state = STATE_STOP;
     resp[0] = STATE_STOP;
     *resplen = 0+1;
     break; 
  
 case CMD_START:
     Serial.println("CMD_START");
     lcd.print("START"); 
           
     start_forward();        
     motor_state = STATE_GO;
     resp[0] = STATE_GO;
     *resplen = 0+1;
     break; 
 
 case CMD_CHECK_AROUND:
     Serial.println("CMD_CHECK_AROUND");
     lcd.print("CHECK AROUND");
    
     checkdir = check_around();
     
     lcd.setCursor(0,1); 
     if      (checkdir == DIRECTION_LEFT)     lcd.print("LEFT");
     else if (checkdir == DIRECTION_RIGHT)    lcd.print("RIGHT");
     else if (checkdir == OBSTACLE)           lcd.print("OBSTACLE");        
     else if (checkdir == OBSTACLE_LEFT)      lcd.print("OBSTACLE LEFT");
     else if (checkdir == OBSTACLE_RIGHT)     lcd.print("OBSTACLE RIGHT");
     else if (checkdir == OBSTACLE_LEFT_RIGHT)lcd.print("OBSTACLE LEFT RIGHT");
     else                                     lcd.print("?");

     resp[0] = (uint16_t)checkdir;
     *resplen = 0+1;
     break;
     
 case CMD_MOVE_TILT_PAN:    
     Serial.print("CMD_MOVE_TILT_PAN: "); Serial.print(cmd[1]);Serial.print(cmd[2]);Serial.print(cmd[3]);Serial.println(cmd[4]);    
     if (cmd[2] == 0) HPos = (int)cmd[1] + 90; else HPos = 90 - (int)cmd[1]; 
     if (cmd[4] == 0) VPos = (int)cmd[3] + 90; else VPos = 90 - (int)cmd[3]; 
     Serial.print("CMD_MOVE_TILT_PAN, HPos VPos: "); Serial.print(HPos);Serial.println(VPos);   
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
       if (cmd[1] == 180)
     {       
           Serial.print("CMD_TURN_BACK");
           lcd.print("TURN BACK ");  
         
           ret = turnback (10);  // 10s max
           if (ret != SUCCESS){
           	  Serial.print("turnback error"); Serial.println(ret);
           	  lcd.setCursor(0,1); 
           	  lcd.print("turnback error: "); lcd.print(ret);
           	  error = 1;
           }
     }       
     else if (motor_state == STATE_GO)
     { 
           Serial.print("CMD_TURN, alpha: "); Serial.print((cmd[2] != 1) ? ('+'):('-')); Serial.println((int)cmd[1]); 
           lcd.print("TURN"); lcd.print((cmd[2] != 1) ? ('+'):('-')); lcd.print((int)cmd[1]);lcd.printByte(223); //degree   
         
           ret = turn ((double)((cmd[2] != 1) ? (cmd[1]):(-cmd[1])), 5);  // 5s max        
           if (ret != SUCCESS){
           	  Serial.print("turn error"); Serial.println(ret);
           	  lcd.setCursor(0,1); 
           	  lcd.print(" turn error: "); lcd.print(ret);
           	  error = 1;
           }           
    }
    
    *resplen = 0;
    break;        
     
     
 case CMD_GET_INFOS: 
     Serial.println("CMD_GET_INFOS");

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
     Serial.print("CMD_PICTURE, no_picture: ");
     no_picture++;
     Serial.println(no_picture);
     lcd.print("PICTURE ");
     
     motor_state_save = motor_state;
     if (motor_state == STATE_GO) {
        Serial.println("Stop"); 
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
        Serial.print("makePicture error: "); Serial.println(ret);
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
       
     if (motor_state_save == STATE_GO) {          
        Serial.println("Start");
        start_forward();                     
        motor_state = STATE_GO;
     }
        
     // byte 0: picture number
     resp[0] = no_picture;
     *resplen = 0+1;    
     break;

 case CMD_ALERT: 
     Serial.println("CMD_ALERT");
     lcd.print("Alert"); 
    
     blink(Led_Red);  
     buzz(5); 
     
     // If motor_state == STATE_GO => Stop           
     if (motor_state == STATE_GO) {
        Serial.println("Stop"); 
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
     
     Serial.print("makePicture, no_picture: ");
     no_picture++;
     Serial.println(no_picture);
     lcd.print("PICTURE ");
     
     ret = JPEGCamera.makePicture (no_picture);
     if (ret == SUCCESS)
     { 
        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        Serial.print("makePicture error: "); Serial.println(ret);
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
             
     if (ret == SUCCESS)
     { 
        HPos = 0;
        VPos = 90;
        TiltPan_move(HPos, VPos);

        Serial.print("makePicture, no_picture: ");
        no_picture++;
        Serial.println(no_picture);
        lcd.print("PICTURE ");
        
        ret = JPEGCamera.makePicture (no_picture);

        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        Serial.print("makePicture error: "); Serial.println(ret);
        lcd.setCursor(0,1); 
        lcd.print("error: "); lcd.print(ret);       
        error = 1;
     }
      
     if (ret == SUCCESS)
     { 
        HPos = 180;
        VPos = 90;
        TiltPan_move(HPos, VPos);
     
        Serial.print("makePicture, no_picture: ");
        no_picture++;
        Serial.println(no_picture);
        lcd.print("PICTURE ");
        
        ret = JPEGCamera.makePicture (no_picture);

        lcd.setCursor(0,1);
        lcd.print("picture: "); lcd.print(no_picture);
     }
     else
     {
        Serial.print("makePicture error: "); Serial.println(ret);
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
     Serial.print("CMD_CHECK");Serial.print(" - freq: ");Serial.println((int)cmd[1]);
     lcd.print("Check"); ;lcd.print((int)cmd[1]);
     
     freqCheck = (unsigned long)cmd[1] * 1000;
     
     if (freqCheck > 0) alert_status = check();
     
     if (alert_status != 0) {
           Serial.print("Alert detected: ");Serial.println(alert_status);
           lcd.setCursor(0,1); 
           lcd.print("Alert: "); lcd.print(alert_status);                
     }
     else
     {
           Serial.println("No alert detected: ");
           lcd.setCursor(0,1); 
           lcd.print(" No Alert");               
     } 
  
     // byte 0: alert
     resp[0] = alert_status;
     *resplen = 0+1;
     break; 
  
 case CMD_GO: 
     Serial.print("CMD_GO, nb seconds: "); Serial.println((int)cmd[1]);
     lcd.print("GO ");lcd.print((int)cmd[1]);lcd.print("secs");
     
     GOtimeout = (unsigned long)cmd[1]; 
     
     if (GOtimeout == 0) {    
        Serial.println("stop"); 
        stop();
        motor_state = STATE_STOP;
     }
     else if (motor_state != STATE_GO)
     {  
           Serial.println("start_forward");
           start_forward();
           motor_state = STATE_GO;
     }
     
     error = -1;
     start = millis();
     while((millis() - start < GOtimeout*1000) && (error == -1)) {
          ret = go(GOtimeout);  
     
          if ((ret != SUCCESS) && (ret != OBSTACLE) && (ret != OBSTACLE_LEFT) && (ret != OBSTACLE_RIGHT))
          {
              stop();
              motor_state = STATE_STOP;
     	      error = 1;
     	                   
              Serial.print("CMD_GO error: "); Serial.println(ret);
              Serial.println("Stop");              
              lcd.setCursor(0,1); 
              lcd.print("error: "); lcd.print(ret);                

          }
          else if ((ret == OBSTACLE) || (ret == OBSTACLE_LEFT) || (ret == OBSTACLE_RIGHT))
          {
              stop();
              motor_state = STATE_STOP;
              
              buzz(3);
              blink(Led_Red);     
              Serial.print("CMD_GO Obstacle: ");Serial.println(ret);
              Serial.println("Stop");
              lcd.setCursor(0,1); 
              if (ret == OBSTACLE_LEFT)        lcd.print("OBSTACLE LEFT");
              else if (ret == OBSTACLE_RIGHT)  lcd.print("OBSTACLE RIGHT");
              else if (ret == OBSTACLE)        lcd.print("OBSTACLE");
              else 

                              
              ret = SUCCESS;            
              checkdir = check_around();
         
              Serial.print("check_around, direction: "); Serial.println(checkdir);
              lcd.clear();
              lcd.print("check around");
              lcd.setCursor(0,1); 
              if      (checkdir == DIRECTION_LEFT)  lcd.print("LEFT");
              else if (checkdir == DIRECTION_RIGHT) lcd.print("RIGHT");
              else if (checkdir == OBSTACLE_LEFT)   lcd.print("OBSTACLE LEFT");
              else if (checkdir == OBSTACLE_RIGHT)  lcd.print("OBSTACLE RIGHT");
              else if (checkdir == OBSTACLE)        lcd.print("OBSTACLE");
              else                             lcd.print("?");;
         
              if (checkdir == DIRECTION_LEFT) {
                   start_forward();
                   motor_state = STATE_GO;
                   ret = turn (-45,  5); // turn  -45 degrees during 5s max
                   if (ret != SUCCESS)
                   {
                      stop();
                      motor_state = STATE_STOP;                   	  
                   	  error = 1;
                   	                     	  
                   	  Serial.print("turn error: "); Serial.println(ret);
                      Serial.println("Stop");                                         	  
                   	  lcd.clear();                   	  
                   	  lcd.print("turn left");
                   	  lcd.setCursor(0,1);
                   	  lcd.print("error: "); lcd.print(ret); 
                   }
                   else
                   {
                      lcd.clear();                   	  
                   	  lcd.print("turn left OK");                  	
                   }
              }
              else if (checkdir == DIRECTION_RIGHT) {
                   start_forward();
                   motor_state = STATE_GO;
                   ret = turn (+45,  5); // turn  +45 degrees during 5s max
                   if (ret != SUCCESS)
                   {
                   	  stop();
                      motor_state = STATE_STOP;  
                   	  error = 1;
                   	                     	 
                   	  Serial.print("turn error: "); Serial.println(ret);
                   	  Serial.println("Stop"); 
                   	  lcd.clear();                   	  
                   	  lcd.print("turn right");
                   	  lcd.setCursor(0,1);
                   	  lcd.print("error: "); lcd.print(ret); 
                   }
                   else
                   {
                      lcd.clear();                   	  
                   	  lcd.print("turn right OK");                  	
                   }                  
              }
              else 
              {
              	   buzz(3);
                   blink(Led_Red);
              	   motor_state = STATE_GO;
              	   ret = turnback (10); // turn back during 10s max
                   if (ret != SUCCESS)
                   {
                      stop();
                      motor_state = STATE_STOP;
                      error = 1; 
                      
                      Serial.print("turnback error"); Serial.println(ret);
                   	  Serial.println("Stop");
                   	  lcd.clear();                   	  
                   	  lcd.print("turnback");
                   	  lcd.setCursor(0,1);
                   	  lcd.print("error: "); lcd.print(ret);                    	                                           	  
                   }
                   else
                   {
                      lcd.clear();                   	  
                   	  lcd.print("turnback OK");                  	
                   }                   
              }                 
          }
          else
          {
              	   Serial.println("GO OK");
              	   lcd.clear();                   	  
                   lcd.print("GO OK");
                   error = 0;
          }           
     } // end while
     
     ret = infos (resp, &infolen);
              
     if (error == 0) {
         if (resp[MOTOR_STATE] == STATE_GO) {
            lcd.print("RUNNING");
          }    
         else
         {
             lcd.print("STOPPED");
         }
         lcd.setCursor(0,1);   
         lcd.print(resp[TEMPERATURE]); lcd.print((byte)lcd_celcius);lcd.write(lcd_pipe);   
         lcd.print(resp[DISTANCE]); lcd.print("cm");lcd.write(lcd_pipe);
         lcd.print(resp[DIRECTION]); lcd.printByte(223); //degree   
     } 
     
     *resplen = infolen;      
     break;
 
 case CMD_PI: 
     Serial.print("CMD_PI activated, type: "); Serial.print((int)cmd[1]);  Serial.print(" - freq: ");Serial.println((int)cmd[2]);
     lcd.print("PI activated ");lcd.print((int)cmd[1]);
 
     PI_activated = (int)cmd[1];
     Serial.print("PI_activated: "); Serial.println(PI_activated);
     if (PI_activated == PI_ALERT_INFOS) {
         PI_freqInfos = (unsigned long)cmd[2]* 1000;
         Serial.print("PI_freqInfos: "); Serial.println(PI_freqInfos);
     }
       
    *resplen = 0;     
     break;
 
 case CMD_TEST: 
     Serial.println("CMD_TEST");
     
     // reply each value incremented
     resp[ALERT_STATUS] = cmd[1]+1;
     Serial.print("alert status: ");Serial.println(resp[ALERT_STATUS]);
     
     resp[NO_PICTURE] = cmd[2]+1;
     Serial.print("no_picture: ");Serial.println(resp[NO_PICTURE]);
     
     resp[MOTOR_STATE] = cmd[3]+1;
     Serial.print("motor_state: ");Serial.println(resp[MOTOR_STATE]);
     
     resp[DIRECTION] = cmd[4]+1;
     Serial.print("direction: ");Serial.println(resp[DIRECTION]);

     resp[OBSTACLE_STATUS] = cmd[5]+1;
     Serial.print("obstacle_status: ");Serial.println(resp[OBSTACLE_STATUS]);

     resp[DISTANCE] = cmd[6]+1;
     Serial.print("distance: ");Serial.println(resp[DISTANCE]);

     resp[TEMPERATURE] = cmd[7]+1;
     Serial.print("temperature: ");Serial.println(resp[TEMPERATURE]);
     
     resp[HUMIDITY] = cmd[8]+1;
     Serial.print("humidity: ");Serial.println(resp[HUMIDITY]);  

     resp[BRIGHTNESS] = cmd[9]+1;
     Serial.print("brightness: ");Serial.println(resp[BRIGHTNESS]);
     
     resp[NOISE] = cmd[10]+1;
     Serial.print("noise: ");Serial.println(resp[NOISE]);


     lcd.setCursor(0,1);   
     lcd.print("TEST");lcd.write(lcd_pipe); 
 
     *resplen = RESP_SIZE;
     break;          
 
 default:
    Serial.println("invalid command");
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
 
 mem0 = freeRam();Serial.print("freeRam: ");Serial.println(mem0); 
 while (1) {  // No stop
       
       //check memory
       mem = freeRam();
       if (mem0 < mem){ Serial.print("Memory leak from: ");Serial.print(mem0);Serial.print(" to: ");Serial.println(mem); }
        
       currentTimeCheck = millis();
       if ( ((currentTimeCheck > previousTimeCheck + freqCheck) || (IntMotion == 1)) && (freqCheck > 0)) {  // check every freqCheck or if motion alert
             previousTimeCheck = currentTimeCheck; 
             Serial.print("IntMotion: ");Serial.println(IntMotion);              

             mcmd[0] = (uint16_t)CMD_CHECK;
             mcmd[1] = (uint16_t)freqCheck/1000;
             Serial.print("Call command CHECK every ");Serial.print(mcmd[1]);Serial.println(" seconds");
             ret = robot_command (mcmd, mresp, &mresplen);
 
             Serial.print("Call robot_command, ret: "); Serial.println(ret);	
             alert_status = (int)mresp[0];  
             Serial.print("Alert: "); Serial.println(alert_status);
             Serial.println(" "); 
       }
       
       PI_currentTimeInfos = millis();  
       if (alert_status > 0)
       {
             Serial.print  ("Alert trigerred: "); Serial.println(alert_status);	// robot has detected something...
             Serial.println("***************");
             Serial.println("");
                     
             Serial.println("Call command ALERT");
  	         mcmd[0] = (uint16_t)CMD_ALERT;
             ret = robot_command (mcmd, mresp, &mresplen);  
             Serial.print("Call robot_command, ret: "); Serial.println(ret);
             n_pict = (uint8_t)(mresp[NO_PICTURE]); // Last Picture number
             Serial.print("n_pict: "); Serial.println(n_pict);
                           
             // Send Infos + last 3 pictures in WIFI to the PI server if it is activated
             if ((PI_activated == PI_ALERT_ONLY)|| (PI_activated == PI_ALERT_INFOS)) {
                
                WakeUpESP();  // wake up ESP
                
                mresp[NO_PICTURE] = 0; // No picture send
                //Send the Infos message first time quickly
                Serial.println("Call IOTSsend 1 INFOS");
                ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                              
                //Send the 3 last pictures
                Serial.println("Call robot_Send_Picture ");
                if (n_pict > 3) ret = robot_Send_Picture(n_pict-2);                 
                if (n_pict > 2) ret = robot_Send_Picture(n_pict-1);                   
                if (n_pict > 1) ret = robot_Send_Picture(n_pict); 
                
                //Send the Infos message again to attach pictures
                mresp[NO_PICTURE] = n_pict;
                Serial.println("Call IOTSsend 1 INFOS");
                ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                Serial.print("Call IOTSsend, ret: "); Serial.println(ret);   
                
                ret = IOTSerial.IOTSsend (1, SLEEP);
                Serial.print("Call IOTSsend SLEEP, ret: "); Serial.println(ret);         
                           
                alert_status = 0;
             }
             else
             {
                Serial.println("PI communication not activated"); 
                alert_status = 0;   
             } 
             Serial.println(" ");                        
       }
       else if (IntIOT > 0) { // IOT wants to do something...
             Serial.println("Request received from IOT, call robot_IOT");	
             Serial.println("*****************************************");
             Serial.println("");
             IntIOT = 0;
 
             ret = robot_IOT(); 
             if (ret != SUCCESS) {
                   Serial.print("robot_IOT error: "); Serial.println(ret);
                   ret = IOTSerial.IOTSflush(2);
             }
             else
             {
                   Serial.println("robot_IOT OK"); 
             }
             Serial.println(" "); 

       }
       else if (PI_activated == PI_ALERT_INFOS)
       {
             if ((PI_freqInfos > 0) && (PI_currentTimeInfos > PI_previousTimeInfos + PI_freqInfos)) {
                   PI_previousTimeInfos = PI_currentTimeInfos;      

                   mcmd[0] = (uint16_t)CMD_GET_INFOS;
                   Serial.println("Call command CMD_GET_INFOS");
                   ret = robot_command (mcmd, mresp, &mresplen);
                   Serial.print("Call robot_command, ret: "); Serial.println(ret);
                   
                   WakeUpESP();  // wake up ESP
                   
                   Serial.println("Call IOTSsend 1 INFOS");
                   ret = IOTSerial.IOTSsend (1, INFOS, mresp, mresplen);
                   Serial.print("Call IOTSsend INFOS, ret: "); Serial.println(ret);
                   
                   ret = IOTSerial.IOTSsend (1, SLEEP);
                   Serial.print("Call IOTSsend SLEEP, ret: "); Serial.println(ret);

                   Serial.println(" "); 
             }
        
       }   
       else if (motor_state == STATE_GO)
       {       
             Serial.print("Call command GO for "); Serial.print(GOtimeout); Serial.println(" seconds");
             mcmd[0] = (uint16_t)CMD_GO;
             mcmd[1] = (uint16_t)GOtimeout;
             mcmd[2] = (uint16_t)0;   // na  
             ret = robot_command (mcmd, mresp, &mresplen);
             Serial.print("Call robot_command, ret: "); Serial.println(ret);
             Serial.println(" ");             
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
 
 Serial.println("Start robot_IOT");

 //Read the message received from IOT
 ret = IOTSerial.IOTSread(2, msg, &msg_len, 60000UL);  // timeout 60s
 Serial.print("Call IOTSread 2, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);

 if (ret != SUCCESS) {
       Serial.println("error IOTSread, Call IOTSsend 2 with RESP_KO");  
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -1;
 }
   
 IOTSerial.IOTSgetTags(msg, tag, value, &nbtags);   
 
 Serial.print("nbtags: "); Serial.print((int)nbtags);
 if (nbtags < 1) {
       Serial.println("nbtags < 1, Call IOTSsend 2 with RESP_KO");    
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -2;
 }
 
 Serial.print(", tag[0]: 0x"); Serial.print((int)tag[0],HEX); Serial.print(", value[0]: 0x"); Serial.println(value[0],HEX);
 if (tag[0] != TAG_CMDID) {
       Serial.println("TAG_CMDID Missing, Call IOTSsend 2 with RESP_KO");    
       ret = IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -3;
 }

 cmdId = value[0];
 Serial.print("cmdId: "); Serial.println((int)cmdId);
  
 if (tag[1] == TAG_CMD)
 {
       Serial.println("Tag CMD");
       if (nbtags > MAX_TAGS) {
               Serial.println("nbtags > MAX_TAGS, Call IOTSsend 2 with RESP_KO");    
               ret = IOTSerial.IOTSsend(2, RESP_KO);
               reset_leds();
               return -4;
       }     
             
       for (uint8_t j=0; j<(nbtags-1); j++)
       {
           cmd[j] = value[j+1];
           Serial.print("cmd["); Serial.print((int)j); Serial.print("]: 0x"); Serial.println((int)cmd[j],HEX);
       }
 
       // Execute the command received from IOT
       ret = robot_command (cmd, resp, &resplen);  
       Serial.print("Call robot_command, ret: "); Serial.println(ret);

       if (ret == SUCCESS) { 
        	 Serial.println("Call IOTSsend 2 with RESP_OK");    
        	 for (uint8_t k=0; k<resplen; k++)
             {
                Serial.print("resp["); Serial.print((int)k); Serial.print("]: "); Serial.println(resp[k]);
             }
             ret = IOTSerial.IOTSsend(2, RESP_OK, resp, resplen, cmdId);
       }
       else
       {
        	 Serial.println("Call IOTSsend 2 with RESP_KO");    
             ret = IOTSerial.IOTSsend(2, RESP_KO);
             reset_leds();
             return -5;
       }        
 }
 else {
       Serial.println("Call IOTSsend 2 with RESP_KO");    
       IOTSerial.IOTSsend(2, RESP_KO);
       reset_leds();
       return -6;
 }
   
 reset_leds();   
 Serial.println("End OK robot_IOT"); 
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
 
 Serial.print("Start robot_Send_Picture: "); Serial.println(n);
 
 // Open picture file
 sprintf(filename, "PICT%d.jpg", n);  
 if (!FilePicture.open(&root, filename, O_READ)) return FILE_OPEN_ERROR; 
 Serial.println("FilePicture open ok "); 
 
 //Send the Picture message 
 param[0] = (uint16_t)n;
 param[1] = (uint16_t)FilePicture.fileSize();
 paramlen = 2;

 ret = IOTSerial.IOTSsend(1, PICTURE, param, paramlen); 
 
 // CANNOT use TX swapped with Sparkfunk Thing  
       
 //Read the message replied to be sure that the client is ready to receive the picture
 //ret = IOTSerial.IOTSread(1, msg, &msg_len, 60000UL);  // timeout 60s
 //Serial.print("Call IOTSread 1, ret: "); Serial.print(ret); Serial.print(", msg_len: "); Serial.println((int)msg_len);

 //if (ret != SUCCESS) {
   //  Serial.println("error IOTSread");  
     //ret = IOTSerial.IOTSflush(1);
     //return 0;
 //}
 
 //Decode the message
 //IOTSerial.IOTSgetTags(msg, tag, value, &nbtags); // parse the response  
 //Serial.print("Call IOTSgetTags, nbtags: "); Serial.println((int)nbtags);
    
 //if (nbtags < 1)          return -1; 
    
 //Serial.print("tag[0]: 0x");Serial.println((int)tag[0],HEX); 
 //if (tag[0]!= TAG_CMDID)   return -2;
 //Serial.print("tag[1]: "); Serial.println((int)tag[1]);  
 //if (tag[1]!= TAG_RESP)   return -3;
 //Serial.print("value[1]: "); Serial.println(value[1]);        
 //if (value[1] == RESP_KO) return -4;
 //if (value[1] != RESP_OK) return -5;

 delay(10*1000);  // delay 10s because can't read ESP answer
 
 
 // read from the file until there's nothing else in it:
 while ((nbytes = FilePicture.read(buf, sizeof(buf))) > 0 && ret == SUCCESS) {
       for (uint16_t i = 0;i<nbytes;i++)
       {
         ret = IOTSerial.IOTSRawsend(1, buf [i]); 
       }
 
 }// while 
  
 //Close file
 if (!FilePicture.close()) return FILE_CLOSE_ERROR; 
 
    
 Serial.println("End OK robot_IOT"); 
 return SUCCESS;                     
}			 