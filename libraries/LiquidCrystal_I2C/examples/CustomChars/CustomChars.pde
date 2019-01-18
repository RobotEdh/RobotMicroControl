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

// display all keycodes
void displayKeyCodes(void) {
  uint8_t i = 0;
  while (1) {
    lcd.clear();
    lcd.print("Codes 0x"); lcd.print(i, HEX);
    lcd.print("-0x"); lcd.print(i+16, HEX);
    lcd.setCursor(0, 1);
    for (int j=0; j<16; j++) {
      lcd.printByte(i+j);
    }
    i+=16;
    
    delay(4000);
  }
}

void loop()
{
  displayKeyCodes();
}

