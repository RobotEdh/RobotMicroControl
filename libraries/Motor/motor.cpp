#include <motor.h>
#include <SharpIR.h> // IR sensor
#include <CMPS12.h>     // Compas
#include <Servo.h>      // Servo
#include <LiquidCrystal_I2C.h> // LCD
#include <VL53L0X.h>     // TOF

// Logging mode
//#define  LOGSERIAL
#define LOGSDCARD  // log to SD Card
#define LOGTRACE   // Enable trace
#include <log.h>
File logFile;                     // The loging class

CMPS12Class CMPS12;               // The Compass class
SharpIRClass SharpIR;             // The IR sensor class
Servo IRServo;                    // The Servo class used for IR sensor
VL53L0XClass VL53L0Xleft;         // The ToF class for left direction
VL53L0XClass VL53L0Xfront;        // The ToF class for front direction
VL53L0XClass VL53L0Xright;        // The ToF class for right direction
LiquidCrystal_I2C lcd (LCD_ADDRESS,16,2); // The LCD class with 16 chars and 2 line display

int SpeedMotorRight = 0;      // Duty cycle PWM motor right between 0 and SPEEDMAX( 255)
int SpeedMotorLeft = 0;       // Duty cycle PWM motor left between 0 and SPEEDMAX (255)

#ifdef PID
// data updated during interrupts
volatile int TickRight = 0;   
volatile int TickLeft = 0;
#endif

int motor_begin()
{
  int ivalue = 0;
  uint8_t status = 0;
   
  PRINTs("Begin Motor Init")
  PRINTs("****************")

  // initialize the lcd 
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.printByte(lcd_bell);    lcd.print(" ");
  lcd.printByte(lcd_note);    lcd.print(" ");
  lcd.printByte(lcd_clock);   lcd.print(" ");
  lcd.printByte(lcd_smiley);  lcd.print(" ");
  lcd.printByte(lcd_duck);    lcd.print(" ");
  lcd.printByte(lcd_celcius); lcd.print(" ");
  lcd.printByte(lcd_pipe);

  delay (5*1000);
  lcd.clear();
  PRINTs("Init LCD OK")
  
  lcd.print("Begin Motor Init");
  lcd.setCursor(0,1); 
  lcd.print("Init LCD OK     ");
  
  
  // H bridge setup
  pinMode(InMotorRight1Pin, OUTPUT);      // set the pin as output
  pinMode(EnableMotorRight1Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(InMotorRight2Pin, OUTPUT);      // set the pin as output
  pinMode(EnableMotorRight2Pin, OUTPUT);  // set the analogig pin as output for PWM
  pinMode(InMotorLeft1Pin, OUTPUT);       // set the pin as output
  pinMode(EnableMotorLeft1Pin, OUTPUT);   // set the analogig pin as output for PWM
  pinMode(InMotorLeft2Pin, OUTPUT);       // set the pin as output
  pinMode(EnableMotorLeft2Pin, OUTPUT);   // set the analogig pin as output for PWM
  
  stop();
  lcd.print("Init motors OK");
  PRINTs("Init motors OK")
  delay(5*1000);lcd.clear(); 
   

  // initialize the pin connected to the Contact sensors 
  pinMode(ContactRightPin, INPUT);
  pinMode(ContactLeftPin, INPUT);
  
  PRINTs(" ")
  PRINTs("Test Contact sensors in 5s")
  delay(5*1000);     
  ivalue = digitalRead(ContactRightPin);  // read input value
  if (ivalue == LOW) PRINTs("-->obstacle right")
  else               PRINTs("-->No obstacle right")
  ivalue = digitalRead(ContactLeftPin);  // read input value
  if (ivalue == LOW) PRINTs("-->obstacle left")
  else               PRINTs("-->No obstacle left")
  PRINTs("Init Contact sensors OK")
    
  // initialize the pin connected to the IR sensor 
  SharpIR.SharpIR_init(SHARP_IR_PIN,(long)SHARP_MODEL); 
  double distance = SharpIR.SharpIR_distance();
  PRINTs(" ")
  PRINT("SharpIR sensor, Distance in mm (max 800mm): ",distance)
  lcd.print("IR:");lcd.print((int)distance);lcd.print(" mm");lcd.printByte(lcd_pipe);   
  PRINTs("Init SharpIR sensor OK")
  delay(5*1000);lcd.clear(); 

  // initialize the PWM pin connected to the servo used for the IR sensor and initialize the associate Timer interrupt
  PRINTs(" ")
  PRINTs("Move IR Servo")
  IRServo.attach(IRSERVO_PIN);  
 
  // test the servo position
  IRServo.write(0);    // turn servo left
  delay(15*90);        // waits the servo to reach the position 
       
  IRServo.write(180);  // turn servo right
  delay(15*180);       // waits the servo to reach the position 
  
  IRServo.write(90);   // reset servo position
  delay(15*90);        // waits the servo to reach the position 
  PRINTs("Init IR Servo OK") 


  // initialize the 3 devices Time Of Flight VL53LOX: left, front and right
  PRINTs(" ")
  PRINTs("Init of the 3 ToF VL53L0X")
    
  uint16_t reg16;
  uint8_t reg8;
  uint8_t address;
  uint16_t d;
  bool a;
  
  //1. Reset all sensors by setting all of their XSHUT pins low for delay(10), then set all XSHUT high to bring out of reset
  pinMode(VL53L0X_LEFT_XSHUT_PIN,  OUTPUT);
  pinMode(VL53L0X_FRONT_XSHUT_PIN, OUTPUT);
  pinMode(VL53L0X_RIGHT_XSHUT_PIN, OUTPUT);
  digitalWrite (VL53L0X_LEFT_XSHUT_PIN,  LOW);
  digitalWrite (VL53L0X_FRONT_XSHUT_PIN, LOW);
  digitalWrite (VL53L0X_RIGHT_XSHUT_PIN, LOW); 
    
  delay(10); 
  
  pinMode(VL53L0X_LEFT_XSHUT_PIN,  INPUT);  // XSHUT high avoiding high voltage 
  pinMode(VL53L0X_FRONT_XSHUT_PIN, INPUT);  // XSHUT high avoiding high voltage 
  pinMode(VL53L0X_RIGHT_XSHUT_PIN, INPUT);  // XSHUT high avoiding high voltage 

  //2. Keep sensor #1 awake by keeping XSHUT pin high, Put all other sensors into shutdown by pulling XSHUT pins low
  pinMode(VL53L0X_FRONT_XSHUT_PIN, OUTPUT);
  pinMode(VL53L0X_RIGHT_XSHUT_PIN, OUTPUT);
  digitalWrite (VL53L0X_FRONT_XSHUT_PIN, LOW);
  digitalWrite (VL53L0X_RIGHT_XSHUT_PIN, LOW); 
    
  PRINTs("Init ToF VL53L0Xleft")
  a = VL53L0Xleft.init((uint8_t)VL53L0X_LEFT_ADDRESS,(uint8_t)VL53L0X_LEFT_XSHUT_PIN);
  if (a)
  {
     PRINTs("Call init OK")
  }
  else
  {
     status = VL53L0Xleft.VL53L0X_getStatus();
     PRINT("Call init KO, error: ",status)
  }
  
  VL53L0Xleft.setTimeout(500);  // 500 ms
  VL53L0Xleft.setMeasurementTimingBudget(33000);  // Default 33 milliseconds; the minimum is 20 ms.
  
  reg16 = VL53L0Xleft.getModelId();
  status = VL53L0Xleft.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getModelId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("ModelId: ",reg16)
  }   
  reg8   = VL53L0Xleft.getRevisionId();
  status = VL53L0Xleft.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getRevisionId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("RevisionId: ",reg8)
     address = VL53L0Xleft.VL53L0X_getAddress();
     PRINTx("VL53L0Xleft Address: ",address)
     delay(2000); 
       
     d = VL53L0Xleft.VL53L0X_readMillimeters();
     PRINT("Distance in mm (max 1200mm): ",d)
     PRINTs("Init ToF VL53L0Xleft OK")     
  } 
   
  PRINTs("Init ToF VL53L0Xfront")
  a = VL53L0Xfront.init((uint8_t)VL53L0X_FRONT_ADDRESS,(uint8_t)VL53L0X_FRONT_XSHUT_PIN);
  if (a)
  {
     PRINTs("Call init OK")
  }
  else
  {
     status = VL53L0Xfront.VL53L0X_getStatus();
     PRINT("Call init KO, error: ",status)
  }  
  VL53L0Xfront.setTimeout(500);  // 500 ms
  VL53L0Xfront.setMeasurementTimingBudget(33000);  // Default 33 milliseconds; the minimum is 20 ms.
  
  reg16 = VL53L0Xfront.getModelId();
  status = VL53L0Xfront.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getModelId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("ModelId: ",reg16)
  }   
  reg8   = VL53L0Xfront.getRevisionId();
  status = VL53L0Xfront.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getRevisionId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("RevisionId: ",reg8)
     address = VL53L0Xfront.VL53L0X_getAddress();
     PRINTx("VL53L0Xfront Address: ",address)
     delay(2000); 
       
     d = VL53L0Xfront.VL53L0X_readMillimeters();
     PRINT("Distance in mm (max 1200mm): ",d)
     PRINTs("Init ToF VL53L0Xfront OK")     
  }
    
  PRINTs("Init ToF VL53L0Xright")
  a = VL53L0Xright.init((uint8_t)VL53L0X_RIGHT_ADDRESS,(uint8_t)VL53L0X_RIGHT_XSHUT_PIN);
  if (a)
  {
     PRINTs("Call init OK")
  }
  else
  {
     status = VL53L0Xright.VL53L0X_getStatus();
     PRINT("Call init KO, error: ",status)
  }  
 
  VL53L0Xright.setTimeout(500);  // 500 ms
  VL53L0Xright.setMeasurementTimingBudget(33000);  // Default 33 milliseconds; the minimum is 20 ms.
  
  reg16 = VL53L0Xright.getModelId();
  status = VL53L0Xright.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getModelId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("ModelId: ",reg16)
  }   
  reg8   = VL53L0Xright.getRevisionId();
  status = VL53L0Xright.VL53L0X_getStatus();
  if (status > 0)
  {
     PRINT("getRevisionId KO, I2C error: ",status)
  }
  else
  {
     PRINTx("RevisionId: ",reg8)
     address = VL53L0Xright.VL53L0X_getAddress();
     PRINTx("VL53L0Xright Address: ",address)
     delay(2000); 
       
     d = VL53L0Xright.VL53L0X_readMillimeters();
     PRINT("Distance in mm (max 1200mm): ",d)
     PRINTs("Init ToF VL53L0Xright OK")     
  }
   
    
  // initialize the Compass CMPS12
  PRINTs(" ")
  PRINTs("Init Compass CMPS12")
  uint8_t calib = CMPS12.CMPS12_init();
  if(calib == 0) 
  { 
     ivalue = (int)CMPS12.CMPS12_getCompassHighResolution();
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        PRINT("Direction: ",ivalue)
        PRINTs("Init compass OK")
 
        lcd.print(ivalue);lcd.printByte(223);lcd.printByte(lcd_pipe);
        lcd.setCursor(0,1); 
        lcd.print("Init Compass OK ");    
     }
     else
     { 
        PRINT("Init CMPS12 KO, I2C error: ",status)       
        lcd.setCursor(0,1); 
        lcd.print("Init Compass KO ");
 
     }        
  }
  else
  {  
     status = CMPS12.CMPS12_getStatus();
     if (status == 0)
     {
        PRINTb("Calibrate CMPS12 KO, calibrate status: ",calib) 
        lcd.setCursor(0,1); 
        lcd.print("Init Compass KO ");    
     }
     else 
     {           
        PRINT("Init CMPS12 KO, I2C error: ",status)
        lcd.setCursor(0,1); 
        lcd.print("Init Compass KO ");  
     }    
  }
  delay(5*1000);lcd.clear();
  
#ifdef PID      
  // interrupts setup
  pinMode(EncoderTickRightPin, INPUT);      // set the pin as input
  pinMode(EncoderTickLeftPin, INPUT);       // set the pin as input
  attachInterrupt(EncoderTickRightINT, IntrTickRight, FALLING);  // set right tick interrupt
  attachInterrupt(EncoderTickLeftINT, IntrTickLeft, FALLING);    // set left tick interrupt
 
  interrupts(); // enable all interrupts
#endif 
  
  lcd.print("End   Motor Init");
  delay(5*1000);lcd.clear(); 
 
  PRINTs("End Motor Init")
  PRINTs("**************")
  PRINTs("")
  
  return SUCCESS;
  
}


#ifdef PID 
void IntrTickRight()  // right tick interrupt
{
    TickRight++;
} 


void IntrTickLeft()  // left tick interrupt
{
    TickLeft++;
}


int get_TickRight()
{  
  return TickRight;  
}

int get_TickLeft()
{
  return TickLeft;  
}

void reset_TickRight()
{  
  TickRight = 0;  
}

void reset_TickLeft()
{
  TickLeft = 0;  
}
#endif

int get_SpeedMotorRight()
{  
  return SpeedMotorRight;  
}

int get_SpeedMotorLeft()
{
  return SpeedMotorLeft;  
}


void forward(int motor)
{
  if (motor == LEFT_MOTOR) {
       digitalWrite(InMotorLeft1Pin,  HIGH); 
       digitalWrite(InMotorLeft2Pin,  HIGH);       
  }
  else if (motor == RIGHT_MOTOR) {
       digitalWrite(InMotorRight1Pin, HIGH); 
       digitalWrite(InMotorRight2Pin, HIGH);    
  }
  else
  {    
       digitalWrite(InMotorLeft1Pin,  HIGH); 
       digitalWrite(InMotorLeft2Pin,  HIGH);   
       digitalWrite(InMotorRight1Pin, HIGH); 
       digitalWrite(InMotorRight2Pin, HIGH); 
  }  
}


void forward_test(int num) // for test only
{
  if (num == 1) {
        digitalWrite(InMotorRight1Pin, HIGH); 
  }
  if (num == 2) {
        digitalWrite(InMotorRight2Pin, HIGH); 
  }
  if (num == 3) {
        digitalWrite(InMotorLeft1Pin,  HIGH); 
  }      
  if (num == 4) {
        digitalWrite(InMotorLeft2Pin,  HIGH); 
  } 
}

void backward(int motor)
{
  if (motor == LEFT_MOTOR) {
       digitalWrite(InMotorLeft1Pin,  LOW); 
       digitalWrite(InMotorLeft2Pin,  LOW);       
  }
  else if (motor == RIGHT_MOTOR) {
       digitalWrite(InMotorRight1Pin, LOW); 
       digitalWrite(InMotorRight2Pin, LOW);    
  }
  else
  {    
       digitalWrite(InMotorLeft1Pin,  LOW); 
       digitalWrite(InMotorLeft2Pin,  LOW);   
       digitalWrite(InMotorRight1Pin, LOW); 
       digitalWrite(InMotorRight2Pin, LOW); 
  }  
}

void start_forward()
{
     
  forward(BOTH_MOTOR);
  
  SpeedMotorRight = SPEEDNOMINAL;
  SpeedMotorLeft  = SPEEDNOMINAL;
  
  analogWrite(EnableMotorRight1Pin, SpeedMotorRight);
  analogWrite(EnableMotorRight2Pin, SpeedMotorRight);    
  analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
  analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);
  
  return;  
}


void start_forward_test(int num) // for test only
{
     
  forward_test(num);
  
  if (num == 1) {
        analogWrite(EnableMotorRight1Pin, SPEEDNOMINAL);
  }      
  if (num == 2) {
        analogWrite(EnableMotorRight2Pin, SPEEDNOMINAL);    
  }
  if (num == 3) {
        analogWrite(EnableMotorLeft1Pin,  SPEEDNOMINAL);
  }
  if (num == 4) {
        analogWrite(EnableMotorLeft2Pin,  SPEEDNOMINAL);
  }
  return;  
}


void start_backward()
{
     
  backward(BOTH_MOTOR);
  
  SpeedMotorRight = SPEEDNOMINAL;
  SpeedMotorLeft  = SPEEDNOMINAL;
  
  analogWrite(EnableMotorRight1Pin, SpeedMotorRight); 
  analogWrite(EnableMotorRight2Pin, SpeedMotorRight);  
  analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
  analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);  
  return;  
}

void stop()
{
     
  SpeedMotorRight = 0;
  SpeedMotorLeft  = 0;
  
  analogWrite(EnableMotorRight1Pin, SpeedMotorRight);
  analogWrite(EnableMotorRight2Pin, SpeedMotorRight);    
  analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
  analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);
      
  return;  
}


int accelerate (int motor)
{
 if (motor == LEFT_MOTOR) {
       if  (SpeedMotorLeft < SPEEDMAX)   SpeedMotorLeft++;
       else return SPEED_ERROR; 
       analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
       analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);       
 }
 else if (motor == RIGHT_MOTOR) {
       if  (SpeedMotorRight < SPEEDMAX)  SpeedMotorRight++;
       else return SPEED_ERROR; 
       analogWrite(EnableMotorRight1Pin, SpeedMotorRight); 
       analogWrite(EnableMotorRight2Pin, SpeedMotorRight);         
 }
 else {
       if  (SpeedMotorRight < SPEEDMAX)  SpeedMotorRight++;
       else return SPEED_ERROR; 
       if  (SpeedMotorLeft < SPEEDMAX)   SpeedMotorLeft++; 
       else return SPEED_ERROR;  
       analogWrite(EnableMotorRight1Pin, SpeedMotorRight);
       analogWrite(EnableMotorRight2Pin, SpeedMotorRight);  
       analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
       analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);
       
 }
 return SUCCESS;
}


int deccelerate(int motor)
{
 if (motor == LEFT_MOTOR) {
       if  (SpeedMotorLeft > 0)   SpeedMotorLeft--;
       else return SPEED_ERROR; 
       analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
       analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);       
 }
 else if (motor == RIGHT_MOTOR) {
       if  (SpeedMotorRight > 0)  SpeedMotorRight--;
       else return SPEED_ERROR; 
       analogWrite(EnableMotorRight1Pin, SpeedMotorRight); 
       analogWrite(EnableMotorRight2Pin, SpeedMotorRight);          
 }
 else {
       if  (SpeedMotorRight > 0)  SpeedMotorRight--;
       else return SPEED_ERROR; 
       if  (SpeedMotorLeft >0)   SpeedMotorLeft--; 
       else return SPEED_ERROR;  
       analogWrite(EnableMotorRight1Pin, SpeedMotorRight); 
       analogWrite(EnableMotorRight2Pin, SpeedMotorRight);         
       analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
       analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);       
 }     
 return SUCCESS; 
}


int accelerate_n(int motor, int n)
{
 int ret = SUCCESS;
  
 for (int i=0;i<n;i++)
 {
   ret = accelerate(motor);
        
   if (ret == SPEED_ERROR) return i;  //stop if any error
 }     
 return n; 
}


int deccelerate_n(int motor, int n)
{
 int ret = SUCCESS;
  
 for (int i=0;i<n;i++)
 {
   ret = deccelerate(motor);
        
   if (ret == SPEED_ERROR) return i;  //stop if any error
 }     
 return n; 
}

void change_speed(int speed)
{
 SpeedMotorRight = speed;
 SpeedMotorLeft  = speed;
  
 analogWrite(EnableMotorRight1Pin, SpeedMotorRight);
 analogWrite(EnableMotorRight2Pin, SpeedMotorRight);  
 analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
 analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);
 
 return; 
}

int go(unsigned long timeout)
{
 uint16_t distanceLeft  = 0;
 uint16_t distanceFront = 0;
 uint16_t distanceRight = 0;
 int inputpin = HIGH; 

#ifdef PID  
 TickLeft  = 0;  // reset ticks
 TickRight = 0;
#endif
 
 unsigned long start = millis();
 while (millis() - start < timeout*1000UL) {  // go during maximum timeout seconds  
    
#ifdef PID 
       int ret = SUCCESS;
       int pid;
       if (TickLeft > TickRight) {
             pid = computePID (TickLeft - TickRight); // compute PID
             adjustMotor (LEFT_MOTOR, pid);     // Adjust according PID
       }      
       if (TickLeft < TickRight) {
             pid = computePID (TickRight - TickLeft);  // compute PID
             adjustMotor (RIGHT_MOTOR, pid);     // Adjust according PID
       }
#endif
    
       // Check Contacts sensors, HIGH in normal situation
       inputpin = digitalRead(ContactRightPin);  // read input value
       if (inputpin == LOW) {
            PRINTs("->obstacle right")
            return OBSTACLE_RIGHT;   
       }  
       inputpin = digitalRead(ContactLeftPin);  // read input value
       if (inputpin == LOW) { 
           PRINTs("->obstacle left")
           return OBSTACLE_LEFT;   
       }
            

       distanceFront = VL53L0Xfront.VL53L0X_readMillimeters(); // Check distance minimum
       PRINT("->distance front (mm): ",distanceFront)
                        
       if ((distanceFront > 0) && (distanceFront < DISTANCE_MIN)) // obstacle
       {
          PRINTs("->obstacle")
          return OBSTACLE;       
       }
       else if ((distanceFront > 0) && (distanceFront < DISTANCE_NOMINAL)) // obstacle near
       {
          distanceLeft = VL53L0Xleft.VL53L0X_readMillimeters(); 
          PRINT("->distance left(mm): ",distanceLeft)
          distanceRight = VL53L0Xright.VL53L0X_readMillimeters(); 
          PRINT("->distance right(mm): ",distanceRight)
                
          if      ((distanceLeft  > 0) && (distanceLeft > distanceRight) && (distanceLeft  > distanceFront))  adjustMotor (LEFT_MOTOR,  SPEEDTICK); // small turn to left
          else if ((distanceRight > 0) && (distanceRight > distanceLeft) && (distanceRight > distanceFront))  adjustMotor (RIGHT_MOTOR, SPEEDTICK); // small turn to right     
       }                                   
       
 }  // end while (millis() - start < timeout)
 
 return SUCCESS; 
}


int check_around()
{
    double distanceFront = 0.0;
    double distanceLeft  = 0.0;
    double distanceRight = 0.0;
    int inputpin = HIGH; 
    
    // Check Contacts sensors, HIGH in normal situation
    inputpin = digitalRead(ContactRightPin);  // read input value
    if (inputpin == LOW) { 
        PRINTs("ContactRightPin LOW")
        return OBSTACLE_RIGHT;   
    }
    
    inputpin = digitalRead(ContactLeftPin);  // read input value
    if (inputpin == LOW) { 
        PRINTs("ContactLeftPin LOW")       
        return OBSTACLE_LEFT;   
    }
       
    distanceFront = VL53L0Xfront.VL53L0X_readMillimeters(); 
    PRINT("distance front(mm): ",distanceFront)
    distanceLeft = VL53L0Xleft.VL53L0X_readMillimeters(); 
    PRINT("distance left(mm): ",distanceLeft)
    distanceRight = VL53L0Xright.VL53L0X_readMillimeters(); 
    PRINT("distance right(mm): ",distanceRight)
   
    if      (distanceFront > DISTANCE_MIN)                                    return NO_OBSTACLE;  
    if      ((distanceLeft > DISTANCE_MIN) && (distanceLeft > distanceRight)) return DIRECTION_MID_LEFT;     
    else if (distanceRight > DISTANCE_MIN)                                    return DIRECTION_MID_RIGHT; 

       
    IRServo.write(0);    // turn servo left
    delay(15*90);        // waits 1350 ms the servo to reach the position 
    distanceLeft = SharpIR.SharpIR_distance(); // Check distance on right side
    PRINT("SharpIR sensor, distance left(mm): ",distanceLeft)
          
    IRServo.write(180);  // turn servo right
    delay(15*180);       // waits 2700 ms the servo to reach the position 
    distanceRight = SharpIR.SharpIR_distance(); // Check distance on left side
    PRINT("SharpIR sensor, distance right(mm): ",distanceRight)
    
    IRServo.write(90);   // reset servo position
    delay(15*90);        // waits 1350 ms the servo to reach the position 
	 
    if      ((distanceLeft > DISTANCE_MIN) && (distanceLeft > distanceRight)) return DIRECTION_LEFT;     
    else if (distanceRight > DISTANCE_MIN)                                    return DIRECTION_RIGHT; 
    else                                                                      return OBSTACLE_LEFT_RIGHT;      
}


void adjustMotor (int motor, int pid)
{
  PRINT("->adjustMotor ",motor)
  if (motor == LEFT_MOTOR) {
       if  ((SpeedMotorLeft - pid) > SPEEDNOMINAL){
             SpeedMotorLeft = SpeedMotorLeft - pid;
       }
       else if (SpeedMotorLeft > SPEEDNOMINAL){     
             SpeedMotorLeft = SPEEDNOMINAL;
             SpeedMotorRight = SpeedMotorRight + pid - SPEEDNOMINAL;            
       } 
       else {     
             SpeedMotorRight = SpeedMotorRight + pid;            
       }      
  }
  else
  {
       if  ((SpeedMotorRight - pid) > SPEEDNOMINAL){
             SpeedMotorRight = SpeedMotorRight - pid;
       }
       else if (SpeedMotorRight > SPEEDNOMINAL){     
             SpeedMotorRight = SPEEDNOMINAL;
             SpeedMotorLeft = SpeedMotorLeft + pid - SPEEDNOMINAL;         
       } 
       else {     
             SpeedMotorLeft = SpeedMotorLeft + pid;            
       }      
  } 
  
  // check speed max
  
  if (SpeedMotorRight - SPEEDMAX > 0) {
       SpeedMotorRight = SPEEDMAX;
       SpeedMotorLeft = SpeedMotorLeft - (SPEEDMAX - SpeedMotorRight) ;
  } 
  if (SpeedMotorLeft < 0) SpeedMotorLeft = 0;
    
  if (SpeedMotorLeft - SPEEDMAX > 0) {
       SpeedMotorLeft = SPEEDMAX;
       SpeedMotorRight = SpeedMotorRight - (SPEEDMAX - SpeedMotorLeft) ;
  }
  if (SpeedMotorRight < 0) SpeedMotorRight = 0;
       
  analogWrite(EnableMotorRight1Pin,  SpeedMotorRight); 
  analogWrite(EnableMotorRight2Pin,  SpeedMotorRight); 
  analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
  analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);

  PRINT("->SpeedMotorLeft: ",SpeedMotorLeft) 
  PRINT("->SpeedMotorRight: ",SpeedMotorRight) 
}

 
int turn(double alpha, unsigned long timeout)
{
  double direction = 0.0;  // direction between 0 and 360    
  double direction_target = 0.0; 
  int end_turn = 0;
  int sens = 0;
  
  PRINTs("Start turn")
  if ((alpha == 0.0) || (alpha < -180.0) || (alpha > 180.0)) return BAD_ANGLE; // alpha between -180 and +180 and <> 0
  
  change_speed(SPEEDTURN);
  
  direction = CMPS12.CMPS12_getCompassHighResolution(); // get initial direction between 0 and 360 
  if (direction < 0.0)  return COMPASS_ERROR;
  
  direction_target = direction + alpha; // compute target direction between 0 and 360 
  if (direction_target < 0.0)   direction_target = direction_target + 360.0;  
  if (direction_target > 360.0) direction_target = direction_target - 360.0;  
    
  if (abs(direction_target - direction) < 180.0) {
     if (direction_target > direction) sens =  1;
     else                              sens = -1;
  }
  else {
     if (direction_target > direction) sens = -2;
     else                              sens =  2;
  }  
  
  PRINT("alpha: ",alpha)
  PRINT("direction: ",direction)
  PRINT("direction_target: ",direction_target)
  PRINT("sens: ",sens)
    
  if (sens > 0) backward (RIGHT_MOTOR);  // turn right
  else          backward (LEFT_MOTOR);   // turn left
  
  unsigned long start = millis();
  while ((millis() - start < timeout*1000UL) && end_turn == 0) {  // turn during maximum timeout milliseconds   
        direction = CMPS12.CMPS12_getCompassHighResolution(); // get current direction between 0 and 360 
        if ( ((sens == 1) && (direction > direction_target)) || ((sens == -1 ) && (direction < direction_target)) 
           ||((sens == 2) && (direction > direction_target) && (direction <180.0)) || ((sens == -2 ) && (direction < direction_target) && (direction >180.0)) ) end_turn = 1;            
  } //  end while
  
  if (sens > 0) forward (RIGHT_MOTOR); // stop turns right  
  else          forward (LEFT_MOTOR);  // stop turns left 
   
  if(end_turn == 1)
  {
      change_speed(SPEEDNOMINAL);
      PRINTs("Turn end OK")
      return SUCCESS;
  }   
  else
  {
      PRINTs("Turn timeout")
      return TIMEOUT;
  }   
}


int turnback(unsigned long timeout)
{
  int dir = 0;
  int end_turn = 0;
  int ret = SUCCESS;
  
  PRINTs("Start turnback")
  start_backward();
  change_speed(SPEEDBACK);
   
  unsigned long start = millis();
  while ((millis() - start < timeout*1000UL) && end_turn == 0) {  // turn back during maximum timeout milliseconds   
          dir = check_around();
         
          PRINT("check_around, direction: ",dir)
         
          if (dir == DIRECTION_LEFT)
          {
               start_forward();
               ret = turn (-45,  5); // turn  -45 degrees during 5s max
               if (ret != SUCCESS)
               {
               	  PRINT("turn error: ",ret)
               }
               end_turn = 1;
           }
           else if (dir == DIRECTION_RIGHT)
           {
               start_forward();
               ret = turn (+45,  5); // turn  +45 degrees during 5s max
               if (ret != SUCCESS)
               {
               	  PRINT("turn error: ",ret)
               }
               end_turn = 1;
          }     

  }
   
  PRINT("End turnback, end_turn: ",end_turn)
   
  if(end_turn == 1)        return ret;
  else                     return TIMEOUT; 
}