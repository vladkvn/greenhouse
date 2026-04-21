#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// UART to ESP8266 (Petoi): Uno D2 = RX <- ESP TX, Uno D3 = TX -> ESP RX (use 5V to 3.3V level shift on D3)
SoftwareSerial espLink(2, 3);

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int sensorPin = A0;

static const long ESP_BAUD = 57600;

void setup() {
  Serial.begin(9600);
  espLink.begin(ESP_BAUD);

  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("Starting...");
  delay(1000);
  lcd.clear();
}

void loop() {
  const int value = analogRead(sensorPin);

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
  espLink.println(value);

  delay(500);
}
