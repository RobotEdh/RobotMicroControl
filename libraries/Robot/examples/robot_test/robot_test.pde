
#define IOT_PIN 3 //INT1 MEGA2560
 
volatile int IntIOT = 0; 

void IntrIOT()  // IOT interrupt
{
    IntIOT = 1;
}
 
void setup()
{
   
  Serial.begin(9600); // initialize serial port

  // initialize the IOT, interrupt setting
  pinMode(IOT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(IOT_PIN), IntrIOT, RISING);         // set IOT interrupt
 
  interrupts(); // enable all interrupts
  
  
  Serial.println ("Init IOT OK");
}

void loop()
{

  Serial.print("IntIOT: "); Serial.println(IntIOT);

             
 }