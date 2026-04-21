#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensorPin = A0;

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
}

void loop() {
  int value = analogRead(sensorPin);

  lcd.setCursor(0, 0);
  lcd.print("Sensor:       ");
  lcd.setCursor(8, 0);
  lcd.print(value);

  lcd.setCursor(0, 1);
  if (value > 500) {
    lcd.print("WET           ");
  } else {
    lcd.print("DRY           ");
  }

  Serial.println(value);
  delay(500);
}