/*
  Smart Energy Meter (ESP32)
  - 20x4 LCD display
  - RMS Voltage & Current measurement
  - Power (W) & Energy (Wh) calculation
  - Tamper/Theft detection via Hall sensor
  - Relay control
  - Proper formulas for typical voltage sensor & ACS712 current sensor
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ======== LCD Setup ========
LiquidCrystal_I2C lcd(0x27, 20, 4); // 20x4 LCD

// ======== Pin Definitions ========
#define VOLTAGE_PIN 34    // Voltage sensor analog pin
#define CURRENT_PIN 35    // ACS712 current sensor analog pin
#define HALL_PIN 32       // Hall sensor pin
#define RELAY_PIN 33      // Relay output

// ======== Sensor constants ========
// Adjust according to your sensors
#define ADC_RESOLUTION 4095.0    // 12-bit ADC
#define ADC_VOLTAGE 3.3          // ESP32 ADC reference voltage
#define VOLTAGE_SENSOR_MAX 230.0 // Max AC voltage measured
#define ACS_SENSITIVITY 0.100    // V/A for ACS712 5A version
#define ACS_ZERO_VOLT 1.65       // Voltage at 0A (half of Vcc)

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
  lcd.setCursor(0,0);
  lcd.print("Smart Energy Meter");
}

void loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // ===== Read raw sensor values =====
    int rawVoltage = analogRead(VOLTAGE_PIN);
    int rawCurrent = analogRead(CURRENT_PIN);

    // ===== Convert raw ADC to actual voltage =====
    float sensorVoltage = (rawVoltage / ADC_RESOLUTION) * ADC_VOLTAGE; 
    voltageRMS = (sensorVoltage / ADC_VOLTAGE) * VOLTAGE_SENSOR_MAX; // scale to 0-230V

    // ===== Convert raw ADC to current (ACS712) =====
    float currentVoltage = (rawCurrent / ADC_RESOLUTION) * ADC_VOLTAGE; 
    currentRMS = (currentVoltage - ACS_ZERO_VOLT) / ACS_SENSITIVITY;

    // ===== Power & Energy =====
    power = voltageRMS * currentRMS;
    energy += power / 3600.0; // Wh per second

    // ===== Tamper / Theft detection =====
    tamperDetected = digitalRead(HALL_PIN) == LOW;
    theftDetected = tamperDetected;

    if (theftDetected) {
      digitalWrite(RELAY_PIN, HIGH); // Cut power
    } else {
      digitalWrite(RELAY_PIN, LOW);
    }

    // ===== Serial Output =====
    Serial.print("Raw V: "); Serial.print(rawVoltage);
    Serial.print(" | Raw I: "); Serial.print(rawCurrent);
    Serial.print(" | Voltage: "); Serial.print(voltageRMS); Serial.print(" V");
    Serial.print(" | Current: "); Serial.print(currentRMS); Serial.print(" A");
    Serial.print(" | Power: "); Serial.print(power); Serial.print(" W");
    Serial.print(" | Energy: "); Serial.print(energy,3); Serial.print(" Wh");
    Serial.print(" | Theft: "); Serial.println(theftDetected ? "YES" : "NO");

    // ===== Display on 20x4 LCD =====
    lcd.setCursor(0,0);
    lcd.print("Voltage: "); lcd.print(voltageRMS,1); lcd.print(" V     ");
    lcd.setCursor(0,1);
    lcd.print("Current: "); lcd.print(currentRMS,2); lcd.print(" A     ");
    lcd.setCursor(0,2);
    lcd.print("Power: "); lcd.print(power,1); lcd.print(" W     ");
    lcd.setCursor(0,3);
    lcd.print("Energy: "); lcd.print(energy,3); lcd.print(" Wh ");
    lcd.print(theftDetected ? "THEFT!" : "OK    ");
  }
}
