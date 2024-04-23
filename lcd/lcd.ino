#include <LiquidCrystal.h>

LiquidCrystal lcd(15, 4, 23,22,21,19);

void setup() {
  lcd.begin(16, 2);
 lcd.setContrast(50);
  lcd.print("Hello, World!");
}

void loop() {
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  lcd.print(millis() / 1000);
}

