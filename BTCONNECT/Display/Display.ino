/*
  LiquidCrystal Library - Serial Input

 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.

 This sketch displays text sent over the serial port
 (e.g. from the Serial Monitor) on an attached LCD.

 The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 modified 7 Nov 2016
 by Arturo Guadalupi

 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystalSerialDisplay

*/

// include the library code:
#include <SoftwareSerial.h>
#include <LiquidCrystal.h>

SoftwareSerial mySerial(11, 12); // RX, TX

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 9, en = 8, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
bool hasHadData = false;

void setup() {
  mySerial.begin(9600);

  hasHadData = false;
  lcd.begin(16, 2);
  lcd.print("hello, world!");
  delay(5000);
}

void loop() {
  // when characters arrive over the serial port...
  if (mySerial.available()) {
    if(hasHadData) {
      delay(100);
      lcd.clear();
    }
    
    while (mySerial.available() > 0) {
      // display each character to the LCD
      hasHadData = true;
      char x = mySerial.read();
      lcd.write(x);
    }
  } 
  
  if (!hasHadData){
    lcd.clear();
    delay(1000);
    lcd.print('.');
    delay(1000);
  }
}
