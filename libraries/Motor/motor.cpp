#include <motor.h>
#include <SharpIR.h> // IR sensor
#include <CMPS12.h>     // Compas
#include <Servo.h>      // Servo
#include <LiquidCrystal_I2C.h> // LCD
#include <VL53L0X.h>     // TOF

LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x20 for a 16 chars and 2 line display

int SpeedMotorRight = 0;      // Duty cycle PWM motor right between 0 and SPEEDMAX( 255)
int SpeedMotorLeft = 0;       // Duty cycle PWM motor left between 0 and SPEEDMAX (255)

#ifdef PID
// data updated during interrupts
volatile int TickRight = 0;   
volatile int TickLeft = 0;
#endif

CMPS12Class CMPS12;           // The Compass class
SharpIRClass SharpIR;         // The IR sensor class
Servo IRServo;                // The Servo class used for IR sensor
VL53L0XClass VL53L0X;         // The ToF class


int motor_begin()
{
  int ivalue = 0;
 
  Serial.println("Begin Motor Init");
  Serial.println("****************");

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
  Serial.println("Init LCD OK");
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
  Serial.println("Init motors OK");
  delay(5*1000);lcd.clear(); 
   

  // initialize the pin connected to the Contact sensors 
  pinMode(ContactRightPin, INPUT);
  pinMode(ContactLeftPin, INPUT);
  
  Serial.println(" ");
  Serial.println("Test Contact sensors in 5s"); 
  delay(5*1000);     
  ivalue = digitalRead(ContactRightPin);  // read input value
  if (ivalue == LOW) Serial.println("-->obstacle right");
  else Serial.println("-->No obstacle right");
  ivalue = digitalRead(ContactLeftPin);  // read input value
  if (ivalue == LOW) Serial.println("-->obstacle left");
  else Serial.println("-->No obstacle left");
  Serial.println("Init Contact sensors OK");
    
  // initialize the pin connected to the IR sensor 
  SharpIR.SharpIR_init(SHARP_IR_PIN,(long)SHARP_MODEL); 
  ivalue = SharpIR.SharpIR_distance();
  Serial.println(" ");
  Serial.print("SharpIR sensor, Distance: "); Serial.print(ivalue); Serial.println("cm");
  lcd.print("IR:");lcd.print(ivalue);lcd.print(" cm");lcd.printByte(lcd_pipe);   
  Serial.println("Init SharpIR sensor OK");
  delay(5*1000);lcd.clear(); 

  // initialize the PWM pin connected to the servo used for the IR sensor and initialize the associate Timer interrupt
  Serial.println(" ");
  Serial.println("Move IR Servo");
  IRServo.attach(IRSERVO_Pin);  
 
  // test the servo position
  IRServo.write(0);    // turn servo left
  delay(15*90);        // waits the servo to reach the position 
       
  IRServo.write(180);  // turn servo right
  delay(15*180);       // waits the servo to reach the position 
  
  IRServo.write(90);   // reset servo position
  delay(15*90);        // waits the servo to reach the position 
  Serial.println("Init IR Servo OK");  


  // initialize the Time Of Flight VL53LOX
  Serial.println (" ");
  Serial.println("Init ToF VL53L0X");
  VL53L0X.init();
  VL53L0X.setTimeout(500);
  VL53L0X.setMeasurementTimingBudget(200000);
  
  uint16_t reg16 = VL53L0X.getModelId();
  uint8_t reg8   = VL53L0X.getRevisionId();
  Serial.print ("ModelId: 0x"); Serial.println (reg16,HEX);
  Serial.print ("RevisionId: 0x"); Serial.println (reg8,HEX);
  delay(2000);
  
  uint16_t d = VL53L0X.readRangeSingleMillimeters();
  if (VL53L0X.timeoutOccurred()) Serial.print(" TIMEOUT");
  else
  { 
     Serial.print("Distance: "); Serial.print(d); Serial.println("mm");  
  } 
  Serial.println("Init ToF VL53L0X OK");
    
    
  // initialize the Compass CMPS12
  Serial.println(" ");
  if(CMPS12.CMPS12_init()) 
  { 
     ivalue = (int)CMPS12.CMPS12_getCompassHighResolution();
     Serial.print("Direction: ");
     Serial.println(ivalue); 
     lcd.print(ivalue);lcd.print((char)223);lcd.printByte(lcd_pipe);
     lcd.setCursor(0,1); 
     lcd.print("Init Compass OK ");    
     Serial.println("Init compass OK");        
  }
  else
  {  
     Serial.print("Init compass K0"); 
     lcd.setCursor(0,1); 
     lcd.print("Init Compass KO "); 
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
  Serial.println("End Motor Init");
  Serial.println("**************");
  Serial.println("");
  delay(5*1000);lcd.clear(); 
  
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
 uint16_t distance = 0;
 int inputpin = HIGH; 

#ifdef PID  
 TickLeft  = 0;  // reset ticks
 TickRight = 0;
#endif
 
 unsigned long start = millis();
 unsigned long current = millis();
 while (millis() - start < timeout*1000) {  // go during maximum timeout seconds  
    
#ifdef PID 
       int ret = SUCCESS;
       int pid;
       if (TickLeft > TickRight) {
             pid = computePID (TickLeft - TickRight); // compute PID
             ret = adjustMotor (LEFT_MOTOR, pid);     // Adjust according PID
             if (ret == SPEED_ERROR) return SPEED_ERROR;
       }      
       if (TickLeft < TickRight) {
             pid = computePID (TickRight - TickLeft);  // compute PID
             ret = adjustMotor (RIGHT_MOTOR, pid);     // Adjust according PID
             if (ret == SPEED_ERROR) return SPEED_ERROR;
       }
#endif
    
       // Check Contacts sensors, HIGH in normal situation
       inputpin = digitalRead(ContactRightPin);  // read input value
       if (inputpin == LOW) {
            Serial.println("-->obstacle right");
            return OBSTACLE_RIGHT;   
       }  
       inputpin = digitalRead(ContactLeftPin);  // read input value
       if (inputpin == LOW) { 
           Serial.println("-->obstacle left");
           return OBSTACLE_LEFT;   
       }
            
       if (millis() - current > 1*1000) { // check every 1 second
             current = millis();
             
             distance = VL53L0X.readRangeSingleMillimeters(); // Check distance minimum
             if (VL53L0X.timeoutOccurred()) Serial.print(" TIMEOUT");
             else
             { 
                Serial.print("-->distance: ");
                Serial.print(distance); 
                Serial.println("mm");    

                if ((distance > 0) && (distance < (10 *DISTANCE_MIN))) 
                {
                   return OBSTACLE;       
                }                                  
             }
        }  // end check every 1 second  
       
 }  // end while (millis() - start < timeout)
 
 return SUCCESS; 
}


int check_around()
{
    int distance_right = 0;
    int distance_left = 0;
    int inputpin = HIGH; 
    
    // Check Contacts sensors, HIGH in normal situation
    inputpin = digitalRead(ContactRightPin);  // read input value
    if (inputpin == LOW) { 
        return OBSTACLE_RIGHT;   
    }
    
    inputpin = digitalRead(ContactLeftPin);  // read input value
    if (inputpin == LOW) { 
        return OBSTACLE_LEFT;   
    }
       
    IRServo.write(0);    // turn servo left
    delay(15*90);        // waits the servo to reach the position 
    distance_left = SharpIR.SharpIR_distance(); // Check distance on right side
    if ((distance_left > 0) && (distance_left < DISTANCE_MIN)) distance_left = 0;  // Robot need a min distance to turn
       
    IRServo.write(180);  // turn servo right
    delay(15*180);       // waits the servo to reach the position 
    distance_right = SharpIR.SharpIR_distance(); // Check distance on left side
    if ((distance_right > 0) && (distance_right < DISTANCE_MIN)) distance_right = 0;  // Robot need a min distance to turn
  
    IRServo.write(90);   // reset servo position
    delay(15*90);        // waits the servo to reach the position 
	 
    if (distance_left == -1) 
    {
         return DIRECTION_LEFT;
    }     
    else if (distance_right == -1) 
    {
         return DIRECTION_RIGHT;
    }    
    else if ((distance_left > 0) && (distance_left > distance_right)) {
         return DIRECTION_LEFT;     
    }
    else if (distance_right > 0) 
    {
         return DIRECTION_RIGHT; 
    }
    else
    {
         return OBSTACLE; 
    }      
}


int adjustMotor (int motor, int pid)
{
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
  
  if (SpeedMotorRight - SPEEDMAX > SpeedMotorLeft) return SPEED_ERROR;
  if (SpeedMotorRight - SPEEDMAX > 0) {
       SpeedMotorRight = SPEEDMAX;
       SpeedMotorLeft = SpeedMotorLeft - (SPEEDMAX - SpeedMotorRight) ;
  } 
  
  if (SpeedMotorLeft - SPEEDMAX > SpeedMotorRight) return SPEED_ERROR;
  if (SpeedMotorLeft - SPEEDMAX > 0) {
       SpeedMotorLeft = SPEEDMAX;
       SpeedMotorRight = SpeedMotorRight - (SPEEDMAX - SpeedMotorLeft) ;
  }
   
  analogWrite(EnableMotorRight1Pin,  SpeedMotorRight); 
  analogWrite(EnableMotorRight2Pin,  SpeedMotorRight); 
  analogWrite(EnableMotorLeft1Pin,  SpeedMotorLeft);
  analogWrite(EnableMotorLeft2Pin,  SpeedMotorLeft);
  
  return SUCCESS;
   
}

 
int turn(double alpha, unsigned long timeout)
{
  double direction = 0.0;  // direction between 0 and 360    
  double direction_target = 0.0; 
  int end_turn = 0;
  
  if ((alpha == 0.0) || (alpha < -180.0) || (alpha > 180.0)) return BAD_ANGLE; // alpha between -180 and +180 and <> 0
  if (alpha < 0.0) alpha = alpha + 360.0;  
  
  change_speed(SPEEDTURN);
  
  direction = CMPS12.CMPS12_getCompassHighResolution(); // get initial direction between 0 and 360 
  if (direction < 0.0)  return COMPASS_ERROR;
  
  direction_target = direction + alpha; // compute target direction between 0 and 360 
  
  if (alpha < 180.0 ) {
        backward (RIGHT_MOTOR);
  }
  else
  {
        backward (LEFT_MOTOR);  
  }
  
  unsigned long start = millis();
  while ((millis() - start < timeout*1000) && end_turn == 0) {  // turn during maximum timeout milliseconds   
        direction = CMPS12.CMPS12_getCompassHighResolution(); // get current direction between 0 and 360 
        if ( ((alpha < 180.0 ) && (direction > direction_target)) || ((alpha > 180.0 ) && (direction < direction_target)) ) end_turn = 1;
  } 
  
  if (alpha < 180.0 )
  {
          forward (RIGHT_MOTOR); // stop turns right  
  }
  else
  {
          forward (LEFT_MOTOR); // stop turns left 
  }
   
  if (direction < 0)
  {  
      return COMPASS_ERROR;
  }    
  else if(end_turn == 1)
  {
      change_speed(SPEEDNOMINAL);
      return SUCCESS;
  }   
  else
  {
      return TIMEOUT;
  }   
}


int turnback(unsigned long timeout)
{
  int dir = 0;
  int end_turn = 0;
  int ret = SUCCESS;
  
  start_backward();
  change_speed(SPEEDBACK);
   
  unsigned long start = millis();
  while ((millis() - start < timeout*1000) && end_turn == 0) {  // turn back during maximum timeout milliseconds   
          dir = check_around();
         
          Serial.print("check_around, direction: ");
          Serial.println(dir);
         
          if (dir == DIRECTION_LEFT)
          {
               start_forward();
               ret = turn (-45,  5); // turn  -45 degrees during 5s max
               if (ret != SUCCESS)
               {
               	  Serial.print("turn error: ");
               	  Serial.println(ret);
               }
               end_turn = 1;
           }
           else if (dir == DIRECTION_RIGHT)
           {
               start_forward();
               ret = turn (+45,  5); // turn  +45 degrees during 5s max
               if (ret != SUCCESS)
               {
               	  Serial.print("turn error: ");
               	  Serial.println(ret);
               }
               end_turn = 1;
          }     

  }
   
  if(end_turn == 1)        return ret;
  else                     return TIMEOUT; 
}
