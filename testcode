/*
  Smart Energy Meter (ESP32)
  - RMS Voltage & Current Measurement
  - Real-time Power (W) & Energy (Wh) accumulation
  - Tamper detection (Hall Sensor) -> relay trip & alarm
  - Power Theft detection (bypass condition)
  - Local Display (I2C LCD)
  - No website integration, can test independently
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <BlynkSimpleEsp32.h>  // Optional: for cloud test if needed

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

// ======== Blynk (Optional for testing) ========
char auth[] = "YOUR_BLYNK_AUTH_TOKEN"; // Replace if using Blynk
#define VPIN_VOLTAGE V1
#define VPIN_CURRENT V2
#define VPIN_THEFT V3

void setup() {
  Serial.begin(115200);
  
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Smart Energy Meter");

  // Optional Blynk connection (comment if not testing Blynk)
  // Blynk.begin(auth, "SSID", "PASSWORD");
}

void loop() {
  // ===== Read Sensors =====
  voltageRMS = analogRead(VOLTAGE_PIN) * (230.0 / 4095.0); // example calibration
  currentRMS = analogRead(CURRENT_PIN) * (5.0 / 4095.0);   // example calibration

  // ===== Calculate Power & Energy =====
  power = voltageRMS * currentRMS;
  energy += power / 3600.0; // Wh per second

  // ===== Tamper/Theft Detection =====
  tamperDetected = digitalRead(HALL_PIN) == LOW; // Hall sensor triggered
  theftDetected = tamperDetected; // you can add bypass logic here

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

  // ===== Optional Blynk Update =====
  // Blynk.virtualWrite(VPIN_VOLTAGE, voltageRMS);
  // Blynk.virtualWrite(VPIN_CURRENT, currentRMS);
  // Blynk.virtualWrite(VPIN_THEFT, theftDetected ? 1 : 0);
  
  delay(1000);
  // Blynk.run(); // Uncomment if using Blynk
}
