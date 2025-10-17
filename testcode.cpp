/*
  Smart Energy Meter Test Code (Standalone)
  - RMS Voltage & Current Measurement
  - Real-time Power (W) & Energy (Wh) accumulation
  - Tamper detection (Hall Sensor) -> relay trip & alarm
  - Local Display (I2C LCD)
  - No Blynk or website integration
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ======== LCD Setup ========
LiquidCrystal_I2C lcd(0x27, 16, 2); // Adjust I2C address if needed

// ======== Pin Definitions ========
#define VOLTAGE_PIN 34   // Analog pin for voltage sensor
#define CURRENT_PIN 35   // Analog pin for current sensor
#define HALL_PIN 32      // Hall sensor for tamper detection
#define RELAY_PIN 33     // Relay to cut power on theft detection

// ======== Variables ========
float voltageRMS = 0.0;
float currentRMS = 0.0;
float power = 0.0;
float energy = 0.0;
bool theftDetected = false;
bool tamperDetected = false;

unsigned long previousMillis = 0;
const unsigned long interval = 1000; // 1 second

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Energy Meter");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // ===== Read Sensors =====
    // Replace these analogRead lines with your real sensor calibration
    voltageRMS = analogRead(VOLTAGE_PIN) * (230.0 / 4095.0); // Example: 0-230V
    currentRMS = analogRead(CURRENT_PIN) * (5.0 / 4095.0);   // Example: 0-5A

    // ===== Calculate Power & Energy =====
    power = voltageRMS * currentRMS;
    energy += power / 3600.0; // Wh per second

    // ===== Tamper/Theft Detection =====
    tamperDetected = digitalRead(HALL_PIN) == LOW; // Hall sensor triggered
    theftDetected = tamperDetected; // Add your bypass logic if needed

    if (theftDetected) {
      digitalWrite(RELAY_PIN, HIGH); // Cut power
    } else {
      digitalWrite(RELAY_PIN, LOW);
    }

    // ===== Print to Serial =====
    Serial.print("Voltage: "); Serial.print(voltageRMS); Serial.print(" V, ");
    Serial.print("Current: "); Serial.print(currentRMS); Serial.print(" A, ");
    Serial.print("Power: "); Serial.print(power); Serial.print(" W, ");
    Serial.print("Energy: "); Serial.print(energy, 3); Serial.print(" Wh, ");
    Serial.print("Theft: "); Serial.println(theftDetected ? "YES" : "NO");

    // ===== Display on LCD =====
    lcd.setCursor(0, 0);
    lcd.print("V:"); lcd.print(voltageRMS, 1); lcd.print("V ");
    lcd.print("I:"); lcd.print(currentRMS, 2); lcd.print("A");
    lcd.setCursor(0, 1);
    lcd.print("P:"); lcd.print(power, 1); lcd.print("W ");
    lcd.print(theftDetected ? "THEFT!" : "OK   ");
  }
}
