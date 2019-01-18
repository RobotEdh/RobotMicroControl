#include <Wire.h> 
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x20,16,2);  // set the LCD address to 0x20 for a 16 chars and 2 line display

void setup()
{
  Serial.begin(9600); // initialize serial port
   
  Wire.begin(); // initialize I2C  

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

  lcd.setCursor(0,1); 
  lcd.print("Init LCD OK     ");
  
  Serial.println("Init LCD OK");

}

void loop()
{
   lcd.clear();
   lcd.print("Hello world !     "); 
   
   lcd.setCursor(0,1);   
   lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley);  lcd.printByte(lcd_smiley); 
   
   delay (5000);
}